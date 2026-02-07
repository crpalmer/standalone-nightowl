#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

/*
  Standalone NightOwl / ERB RP2040 firmware (2 lanes)
  - Buffer feed with hysteresis latch (start at LOW, stop at HIGH)
  - Autoswap + autoload (non-blocking)

  All switches/buttons wired C/NO to GND -> active LOW with pull-ups.
*/

// ---------------------------- CONFIG ----------------------------

// Firmware tag (prints in debug so you KNOW the flash changed)
#define FW_TAG "BUFFER_HYST_DBG_V3"

// Switch pins (active low, pull-up)
#define PIN_L1_IN      24
#define PIN_L1_OUT     25
#define PIN_L2_IN      22
#define PIN_L2_OUT     12
#define PIN_Y_SPLIT    2
#define PIN_BUF_LOW    6
#define PIN_BUF_HIGH   7

// Steppers
#define PIN_M1_EN      8
#define PIN_M1_DIR     9
#define PIN_M1_STEP    10
#define PIN_M2_EN      14
#define PIN_M2_DIR     15
#define PIN_M2_STEP    16

#define M1_DIR_INVERT  0
#define M2_DIR_INVERT  1
#define EN_ACTIVE_LOW  1

// Autoload speed (fixed)
#define AUTOLOAD_STEPS_PER_SEC  5000

// Timing
#define STEP_PULSE_US           3
#define LOW_DELAY_S             0.40f
#define SWAP_COOLDOWN_S         0.50f
#define AUTOLOAD_TIMEOUT_S      6.0f
#define DEBOUNCE_MS             10

#define REQUIRE_Y_CLEAR_FOR_SWAP  1

// Debug
#define DEBUG_PRINTS      1
#define DEBUG_PERIOD_US   500000

// Step catch-up guard: max pulses per loop per lane
#define STEP_CATCHUP_GUARD  50

// Main loop idle sleep (smaller => higher max step rate)
#define MAIN_LOOP_SLEEP_US  100

// -------------------------- END CONFIG --------------------------

static inline int clamp_i(int v, int lo, int hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

// ------------------------ Debounced input -----------------------

typedef struct {
    uint pin;
    bool stable;
    bool last_raw;
    absolute_time_t last_edge;
} din_t;

static inline void din_init(din_t *d, uint pin) {
    d->pin = pin;
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_up(pin);

    bool raw = gpio_get(pin);
    d->stable = raw;
    d->last_raw = raw;
    d->last_edge = get_absolute_time();
}

static inline void din_update(din_t *d) {
    absolute_time_t now = get_absolute_time();
    bool raw = gpio_get(d->pin);

    if (raw != d->last_raw) {
        d->last_raw = raw;
        d->last_edge = now;
    }

    if (raw != d->stable) {
        if (absolute_time_diff_us(d->last_edge, now) >= (int64_t)DEBOUNCE_MS * 1000) {
            d->stable = raw;
        }
    }
}

static inline bool active_low_on(const din_t *d) {
    return d->stable == 0;
}

// ---------------------------- Stepper ---------------------------

typedef struct {
    uint en, dir, step;
    bool dir_invert;
} stepper_t;

static inline void stepper_init(stepper_t *m, uint en, uint dir, uint step, bool dir_invert) {
    m->en = en; m->dir = dir; m->step = step; m->dir_invert = dir_invert;

    gpio_init(m->en);
    gpio_init(m->dir);
    gpio_init(m->step);

    gpio_set_dir(m->en, GPIO_OUT);
    gpio_set_dir(m->dir, GPIO_OUT);
    gpio_set_dir(m->step, GPIO_OUT);

    // disable by default
    if (EN_ACTIVE_LOW) gpio_put(m->en, 1);
    else               gpio_put(m->en, 0);

    gpio_put(m->step, 0);
    gpio_put(m->dir, 0);
}

static inline void stepper_enable(stepper_t *m, bool on) {
    if (EN_ACTIVE_LOW) gpio_put(m->en, on ? 0 : 1);
    else               gpio_put(m->en, on ? 1 : 0);
}

static inline void stepper_set_dir(stepper_t *m, bool forward) {
    bool d = forward ^ m->dir_invert;
    gpio_put(m->dir, d ? 1 : 0);
}

static inline void stepper_pulse(stepper_t *m) {
    gpio_put(m->step, 1);
    sleep_us(STEP_PULSE_US);
    gpio_put(m->step, 0);
}

// ---------------------------- Lane ------------------------------

typedef enum {
    TASK_IDLE = 0,
    TASK_AUTOLOAD,
    TASK_FEED,
} task_mode_t;

typedef struct {
    din_t in_sw;
    din_t out_sw;
    stepper_t m;

    bool prev_in_present;

    task_mode_t mode;
    absolute_time_t next_step;
    absolute_time_t autoload_deadline;

    int steps_per_sec;
    bool forward;
} lane_t;

static inline bool lane_in_present(lane_t *L)  { return active_low_on(&L->in_sw); }
static inline bool lane_out_present(lane_t *L) { return active_low_on(&L->out_sw); }

static void lane_init(lane_t *L,
                      uint pin_in, uint pin_out,
                      uint pin_en, uint pin_dir, uint pin_step,
                      bool dir_invert) {
    din_init(&L->in_sw, pin_in);
    din_init(&L->out_sw, pin_out);
    stepper_init(&L->m, pin_en, pin_dir, pin_step, dir_invert);

    L->prev_in_present = false;
    L->mode = TASK_IDLE;
    L->next_step = get_absolute_time();
    L->autoload_deadline = get_absolute_time();
    L->steps_per_sec = 0;
    L->forward = true;
}

static inline int32_t step_interval_us(int sps) {
    if (sps <= 0) return 1000000;
    int32_t base = (int32_t)(1000000 / sps);
    int32_t adj  = base - (int32_t)STEP_PULSE_US;
    if (adj < 10) adj = 10;
    return adj;
}

static inline void lane_start_task(lane_t *L, task_mode_t mode, int sps, bool forward, float timeout_s) {
    L->mode = mode;
    L->steps_per_sec = sps;
    L->forward = forward;

    stepper_enable(&L->m, true);
    stepper_set_dir(&L->m, forward);

    L->next_step = get_absolute_time();

    if (mode == TASK_AUTOLOAD && timeout_s > 0) {
        L->autoload_deadline = delayed_by_us(get_absolute_time(), (int64_t)(timeout_s * 1000000));
    }
}

static inline void lane_stop_task(lane_t *L) {
    L->mode = TASK_IDLE;
    stepper_enable(&L->m, false);
}

static void lane_update_inputs(lane_t *L) {
    din_update(&L->in_sw);
    din_update(&L->out_sw);
}

static void lane_process(lane_t *L) {
    if (L->mode == TASK_AUTOLOAD) {
        if (lane_out_present(L) || time_reached(L->autoload_deadline)) {
            lane_stop_task(L);
            return;
        }
    }

    if (L->mode == TASK_IDLE) return;

    int32_t interval = step_interval_us(L->steps_per_sec);

    int guard = 0;
    while (time_reached(L->next_step) && guard++ < STEP_CATCHUP_GUARD) {
        stepper_pulse(&L->m);
        L->next_step = delayed_by_us(L->next_step, interval);
    }

    if (guard >= STEP_CATCHUP_GUARD) {
        L->next_step = get_absolute_time();
    }
}

// ---------------------------- MAIN -----------------------------

#if DEBUG_PRINTS
  #define DBG_PRINTF(...) printf(__VA_ARGS__)
#else
  #define DBG_PRINTF(...) do{}while(0)
#endif

int main() {
    stdio_init_all();
    sleep_ms(1500);

    // Inputs
    din_t y_split, buf_low, buf_high;
    din_init(&y_split, PIN_Y_SPLIT);
    din_init(&buf_low, PIN_BUF_LOW);
    din_init(&buf_high, PIN_BUF_HIGH);

    // Lanes
    lane_t L1, L2;
    lane_init(&L1, PIN_L1_IN, PIN_L1_OUT, PIN_M1_EN, PIN_M1_DIR, PIN_M1_STEP, M1_DIR_INVERT);
    lane_init(&L2, PIN_L2_IN, PIN_L2_OUT, PIN_M2_EN, PIN_M2_DIR, PIN_M2_STEP, M2_DIR_INVERT);

    int active_lane = 1;
    bool swap_armed = false;

    absolute_time_t swap_cooldown_until = get_absolute_time();
    absolute_time_t low_since = get_absolute_time();

    // Buffer hysterese latch
    bool feeding_latched = false;

#if DEBUG_PRINTS
    absolute_time_t last_dbg = {0};
#endif

    // Pot throttling + live feed rate
    absolute_time_t next_pot_read = get_absolute_time();
    int feed_sps = 5000;

    while (true) {
        absolute_time_t now = get_absolute_time();
        int64_t t_us = to_us_since_boot(now);

        // Update inputs
        lane_update_inputs(&L1);
        lane_update_inputs(&L2);
        din_update(&y_split);
        din_update(&buf_low);
        din_update(&buf_high);

        bool l1_in_present  = lane_in_present(&L1);
        bool l2_in_present  = lane_in_present(&L2);
        bool l1_out_present = lane_out_present(&L1);
        bool l2_out_present = lane_out_present(&L2);

        bool buffer_low  = active_low_on(&buf_low);
        bool buffer_high = active_low_on(&buf_high);

        bool y_present = active_low_on(&y_split);
        bool y_clear = !y_present;

	// Autoload on IN rising edge
	if (l1_in_present && !L1.prev_in_present && !l1_out_present && L1.mode == TASK_IDLE) {
	    lane_start_task(&L1, TASK_AUTOLOAD, AUTOLOAD_STEPS_PER_SEC, true, AUTOLOAD_TIMEOUT_S);
	}
	if (l2_in_present && !L2.prev_in_present && !l2_out_present && L2.mode == TASK_IDLE) {
	    lane_start_task(&L2, TASK_AUTOLOAD, AUTOLOAD_STEPS_PER_SEC, true, AUTOLOAD_TIMEOUT_S);
	}

	// -------- Buffer hysterese latch --------
	// Start feed when LOW persists, keep feeding until HIGH.
	if (!buffer_low) {
	    low_since = now; // reset timer whenever LOW is not active
	}
	bool low_persist =
	    absolute_time_diff_us(low_since, now) > (int64_t)(LOW_DELAY_S * 1000000);

	// HIGH wins: if HIGH active, always stop feeding
	if (buffer_high) {
	    feeding_latched = false;
	} else {
	    if (!feeding_latched) {
		if (buffer_low && low_persist) {
		    feeding_latched = true;
		}
	    }
	}

	bool need_feed = feeding_latched;

	// Arm swap when active lane IN empty
	if (active_lane == 1 && !l1_in_present) swap_armed = true;
	if (active_lane == 2 && !l2_in_present) swap_armed = true;

	bool in_cooldown = !time_reached(swap_cooldown_until);

	// Execute swap
	bool allow_swap = need_feed && swap_armed;
#if REQUIRE_Y_CLEAR_FOR_SWAP
	allow_swap = allow_swap && y_clear;
#endif
	if (!in_cooldown && allow_swap) {
	    if (active_lane == 1 && l2_out_present) {
		active_lane = 2;
		swap_armed = false;
		swap_cooldown_until = delayed_by_ms(now, (int32_t)(SWAP_COOLDOWN_S * 1000));
	    } else if (active_lane == 2 && l1_out_present) {
		active_lane = 1;
		swap_armed = false;
		swap_cooldown_until = delayed_by_ms(now, (int32_t)(SWAP_COOLDOWN_S * 1000));
	    }
	}

	// Feed management (pot controls feed_sps)
	lane_t *A = (active_lane == 1) ? &L1 : &L2;
	bool A_out_ok = (active_lane == 1) ? l1_out_present : l2_out_present;

	if (!in_cooldown && need_feed && A_out_ok) {
	    if (A->mode == TASK_IDLE) {
		lane_start_task(A, TASK_FEED, feed_sps, true, 0.0f);
	    } else if (A->mode == TASK_FEED) {
		A->steps_per_sec = feed_sps; // live update from pot
	    }
	} else {
	    if (A->mode == TASK_FEED) lane_stop_task(A);
	}

        // Update prev flags
        L1.prev_in_present = l1_in_present;
        L2.prev_in_present = l2_in_present;

        // Process lanes (pulses + autoload stop)
        lane_process(&L1);
        lane_process(&L2);

#if DEBUG_PRINTS
        if (absolute_time_diff_us(last_dbg, now) > DEBUG_PERIOD_US) {
            last_dbg = now;

            int mid = (!buffer_low && !buffer_high) ? 1 : 0;
            int low_persist_dbg =
                (absolute_time_diff_us(low_since, now) > (int64_t)(LOW_DELAY_S * 1000000)) ? 1 : 0;

            DBG_PRINTF("FW=%s\n", FW_TAG);

            DBG_PRINTF(
                "A=%d armed=%d need=%d lat=%d lowP=%d mid=%d cooldown=%d  "
                "L1[in=%d out=%d mode=%d]  L2[in=%d out=%d mode=%d]  "
                "Y=%d clr=%d  bufL=%d bufH=%d feed_sps=%d\n",
                active_lane, swap_armed,
                (int)(feeding_latched ? 1 : 0),
                (int)(feeding_latched ? 1 : 0),
                low_persist_dbg, mid,
                (!time_reached(swap_cooldown_until)) ? 1 : 0,
                l1_in_present, l1_out_present, (int)L1.mode,
                l2_in_present, l2_out_present, (int)L2.mode,
                y_present, y_clear,
                buffer_low, buffer_high,
                feed_sps
            );
        }
#endif

        sleep_us(MAIN_LOOP_SLEEP_US);
    }

    return 0;
}
