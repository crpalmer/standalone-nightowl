#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

#include "pico/stdlib.h"
#include "hardware/gpio.h"
#include "hardware/timer.h"
#include "hardware/adc.h"

#include "gp-input.h"
#include "gp-output.h"
#include "pi-threads.h"

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
#define PIN_TURTLENECK_FULL  6
#define PIN_TURTLENECK_EMPTY 7

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

class TurtleNeck : public PiThread {
public:
    TurtleNeck(Input *full, Input *empty) : PiThread("turtle-neck"), full(full), empty(empty) {
	lock = new PiMutex();
	start();
    }

    void main() override {
	while (1) {
	    lock->lock();

	    if (active) {
		if (full->get()) {
		    active = false;
		    if (sleep_us > MIN_SLEEP_US) sleep_us -= SLEEP_US_DELTA;
		    else sleep_us = MIN_SLEEP_US;
		}
		if (empty->get()) {
		    sleep_us += SLEEP_US_DELTA;
		}
	    } else if (! full->get()) {
		active = true;
	    }

	    lock->unlock();

	    ms_sleep(1);
	}
    }
	
    bool should_feed() {
	lock->lock();
	bool ret = active;
	lock->unlock();

	return ret;
    }

    int get_sleep_us() {
	lock->lock();
	int ret = sleep_us;
	lock->unlock();

	return ret;
    }

private:
    Input *full;
    Input *empty;

    PiMutex *lock;
    bool active = true;
    int sleep_us = 500;

    static const int MIN_SLEEP_US = 100;
    static const int SLEEP_US_DELTA = 10;
};
	
// ---------------------------- Stepper ---------------------------

class NighthawkStepper {
public:
    NighthawkStepper(Output *en, Output *dir, Output *step_pin) : en(en), dir(dir), step_pin(step_pin) {
    }

    void enable() {
	en->set(true);
    }

    void disable() {
	en->set(false);
    }

    void step(bool forward = true) {
	dir->set(forward);
	step_pin->set(true);
	us_sleep(STEP_PULSE_US);
	step_pin->set(false);
    }

private:
    Output *en;
    Output *dir;
    Output *step_pin;
};

// ---------------------------- Lane ------------------------------

typedef enum {
    TASK_IDLE = 0,
    TASK_AUTOLOAD,
    TASK_FEED,
} task_mode_t;

class Lane {
public:
    Lane(Input *present, Input *loaded, NighthawkStepper *stepper) : present(present), loaded(loaded), stepper(stepper) {
	next_step = get_absolute_time();
	autoload_deadline = get_absolute_time();
    }

    bool is_present() { return present->get(); }
    bool is_loaded() { return loaded->get(); }

    void start_task_if_idle(task_mode_t mode, int sps, bool forward, float timeout_s) {
	if (mode == TASK_IDLE) start_task(mode, sps, forward, timeout_s);
    }

    void start_task(task_mode_t mode, int sps, bool forward, float timeout_s) {
	this->mode = mode;
	this->steps_per_sec = sps;
	this->forward = forward;

	stepper->enable();
	stepper->step(forward);

	next_step = get_absolute_time();

	if (mode == TASK_AUTOLOAD && timeout_s > 0) {
	    autoload_deadline = delayed_by_us(get_absolute_time(), (int64_t)(timeout_s * 1000000));
	}
    }

    void stop_task() {
	mode = TASK_IDLE;
	stepper->disable();
    }

    void stop_feeding() {
	if (mode == TASK_FEED) stop_task();
    }

    void autoload_if_ready() {
	if (is_present() && ! prev_in_present && ! is_loaded() && mode == TASK_IDLE) {
	    start_task(TASK_AUTOLOAD, AUTOLOAD_STEPS_PER_SEC, true, AUTOLOAD_TIMEOUT_S);
	}
    }

    void process() {
        // Update prev flags
        prev_in_present = is_present();

	if (mode == TASK_IDLE) return;

	if (mode == TASK_AUTOLOAD) {
	    if (is_loaded() || time_reached(autoload_deadline)) {
		stop_task();
		return;
	    }
	}

	int32_t interval = step_interval_us(steps_per_sec);

	int guard = 0;
	while (time_reached(next_step) && guard++ < STEP_CATCHUP_GUARD) {
	    stepper->step(forward);
	    next_step = delayed_by_us(next_step, interval);
	}

	if (guard >= STEP_CATCHUP_GUARD) {
	    next_step = get_absolute_time();
	}
    }

private:
    inline int32_t step_interval_us(int sps) {
	if (sps <= 0) return 1,000,000;
	int32_t base = (int32_t)(1000000 / sps);
	int32_t adj  = base - (int32_t)STEP_PULSE_US;
	if (adj < 10) adj = 10;
	return adj;
    }

private:
    Input *present;
    Input *loaded;
    NighthawkStepper *stepper;

    bool prev_in_present = false;

    task_mode_t mode = TASK_IDLE;
    absolute_time_t next_step;
    absolute_time_t autoload_deadline;

    int steps_per_sec = 0;
    bool forward = true;
};

// ---------------------------- MAIN -----------------------------

int main() {
    stdio_init_all();
    sleep_ms(1500);

    Input *full = new GPInput(PIN_TURTLENECK_FULL);
    Input *empty = new GPInput(PIN_TURTLENECK_EMPTY);
    TurtleNeck *turtle_neck = new TurtleNeck(full, empty);

    // Lanes
    Output *L1_enable = new GPOutput(PIN_M1_EN);
    Output *L1_dir = new GPOutput(PIN_M1_DIR);
    Output *L1_step = new GPOutput(PIN_M1_STEP);
    L1_step->set_is_inverted(M1_DIR_INVERT);
    NighthawkStepper *L1_stepper = new NighthawkStepper(L1_enable, L1_dir, L1_step);

    Input *L1_present = new GPInput(PIN_L1_IN);
    Input *L1_loaded = new GPInput(PIN_L1_OUT);
    Lane *L1 = new Lane(L1_present, L1_loaded, L1_stepper);

    Output *L2_enable = new GPOutput(PIN_M2_EN);
    Output *L2_dir = new GPOutput(PIN_M2_DIR);
    Output *L2_step = new GPOutput(PIN_M2_STEP);
    L2_step->set_is_inverted(M2_DIR_INVERT);
    NighthawkStepper *L2_stepper = new NighthawkStepper(L2_enable, L2_dir, L2_step);

    Input *L2_present = new GPInput(PIN_L2_IN);
    Input *L2_loaded = new GPInput(PIN_L2_OUT);
    Lane *L2 = new Lane(L2_present, L2_loaded, L2_stepper);

    int active_lane = 1;
    bool swap_armed = false;

    absolute_time_t swap_cooldown_until = get_absolute_time();
    absolute_time_t low_since = get_absolute_time();

    // Pot throttling + live feed rate
    absolute_time_t next_pot_read = get_absolute_time();
    const int FEED_SPS = 5000;

    while (true) {
        absolute_time_t now = get_absolute_time();
        int64_t t_us = to_us_since_boot(now);

        bool l1_in_present  = L1->is_present();
        bool l2_in_present  = L2->is_present();
        bool l1_out_present = L1->is_loaded();
        bool l2_out_present = L2->is_loaded();

	L1->autoload_if_ready();
	L2->autoload_if_ready();

	// Arm swap when active lane IN empty
	if (active_lane == 1 && !l1_in_present) swap_armed = true;
	if (active_lane == 2 && !l2_in_present) swap_armed = true;

	bool in_cooldown = !time_reached(swap_cooldown_until);

	// Execute swap
	bool allow_swap = turtle_neck->should_feed() && swap_armed;
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

	Lane *A = (active_lane == 1) ? L1 : L2;
	bool A_out_ok = (active_lane == 1) ? l1_out_present : l2_out_present;

	if (!in_cooldown && turtle_neck->should_feed() && A_out_ok) {
	    A->start_task_if_idle(TASK_FEED, FEED_SPS, true, 0.0f);
	} else {
	    A->stop_feeding();
	}

        L1->process();
        L2->process();

        sleep_us(MAIN_LOOP_SLEEP_US);
    }

    return 0;
}
