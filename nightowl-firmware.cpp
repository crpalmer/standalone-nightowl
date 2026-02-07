#include "pi.h"
#include <cstring>
#include "gp-input.h"
#include "gp-output.h"
#include "pi-threads.h"
#include "time-utils.h"

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

// -------------------------- END CONFIG --------------------------

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

    void dump_state() {
	lock->lock();
	printf("turtleneck: %s sleep_us %d\n", active ? "active" : "idle", sleep_us);
	lock->unlock();
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

class Stepper {
public:
    Stepper(Output *en, Output *dir, Output *step_pin) : en(en), dir(dir), step_pin(step_pin) {
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

    static const int STEP_PULSE_US = 5;
};

// ---------------------------- Lane ------------------------------

typedef enum {
    TASK_IDLE = 0,
    TASK_AUTOLOAD,
    TASK_FEED,
} task_mode_t;

class Lane : public PiThread {
public:
    Lane(Input *present, Input *loaded, Stepper *stepper, TurtleNeck *turtleneck, const char *name) : PiThread(name), name(name), present(present), loaded(loaded), stepper(stepper), turtleneck(turtleneck) {
	lock = new PiMutex();
	start();
    }

    void main() override {
	while (1) {
	    bool feed = false;

	    lock->lock();

	    switch (state) {
	    case EMPTY:
		if (present->get()) {
		    stepper->enable();
		    state = PRE_LOADING;
		}
		break;
	    case PRE_LOADING:
		// TODO: add a timeout
		if (! present->get()) {
		    stepper->disable();
		    state = EMPTY;
		} else if (loaded->get()) {
		    stepper->disable();
		    state = READY;
		}
		feed = true;
		break;
	    case READY:
		break;
	    case LOADING:
		// TODO: I need the y output switch, if triggered then switch to active
		// TODO: add a timeout
		break;
	    case ACTIVE:
		// TODO: This should be the y output switch
		if (! loaded->get()) {
		    stepper->disable();
		    state = EMPTY;
		}
		feed = turtleneck->should_feed();
		break;
	    }

	    lock->unlock();

	    if (feed) {
		stepper->step();
		us_sleep(turtleneck->get_sleep_us());
	    } else {
		ms_sleep(1);
	    }
	}
    }

    bool is_active() {
	lock->lock();
	bool ret = (state == ACTIVE);
	lock->unlock();

	return ret;
    }
	
    bool take_over_feeding() {
	bool can_take_over = false;

	lock->lock();
	if (state == READY) {
	    can_take_over = true;
	    stepper->enable();
	    state = LOADING;
	}
	lock->unlock();

	return can_take_over;
    }

    void dump_state() {
	lock->lock();

	printf("%s: ", name);
	switch (state) {
	case EMPTY: printf("empty"); break;
	case PRE_LOADING: printf("pre-loading"); break;
	case READY: printf("ready"); break;
	case LOADING: printf("loading"); break;
	case ACTIVE: printf("active"); break;
	}
	printf("\n");

	lock->unlock();
    }

private:
    const char *name;
    Input *present;
    Input *loaded;
    Stepper *stepper;
    TurtleNeck *turtleneck;

    PiMutex *lock;
    enum { EMPTY, PRE_LOADING, READY, LOADING, ACTIVE } state = EMPTY;
};

// ---------------------------- MAIN -----------------------------

static TurtleNeck *create_turtleneck() {
    Input *full = new GPInput(PIN_TURTLENECK_FULL);
    Input *empty = new GPInput(PIN_TURTLENECK_EMPTY);
    return new TurtleNeck(full, empty);
}

static Lane *create_lane_1(TurtleNeck *turtleneck) {
    Output *L1_enable = new GPOutput(PIN_M1_EN);
    Output *L1_dir = new GPOutput(PIN_M1_DIR);
    Output *L1_step = new GPOutput(PIN_M1_STEP);
    L1_step->set_is_inverted(M1_DIR_INVERT);
    Stepper *L1_stepper = new Stepper(L1_enable, L1_dir, L1_step);

    Input *L1_present = new GPInput(PIN_L1_IN);
    Input *L1_loaded = new GPInput(PIN_L1_OUT);
    return new Lane(L1_present, L1_loaded, L1_stepper, turtleneck, "lane-1");
}

static Lane *create_lane_2(TurtleNeck *turtleneck) {
    Output *L2_enable = new GPOutput(PIN_M2_EN);
    Output *L2_dir = new GPOutput(PIN_M2_DIR);
    Output *L2_step = new GPOutput(PIN_M2_STEP);
    L2_step->set_is_inverted(M2_DIR_INVERT);
    Stepper *L2_stepper = new Stepper(L2_enable, L2_dir, L2_step);

    Input *L2_present = new GPInput(PIN_L2_IN);
    Input *L2_loaded = new GPInput(PIN_L2_OUT);
    return new Lane(L2_present, L2_loaded, L2_stepper, turtleneck, "lane-2");
}

class Coordinator : public PiThread {
public:
    Coordinator() : PiThread("coordinator") {
	turtleneck = create_turtleneck();
	lane_1 = create_lane_1(turtleneck);
	lane_2 = create_lane_2(turtleneck);

	start();
    }

    void main() override {
	while (true) {
	    if (! lane_1->is_active() && ! lane_2->is_active()) {
		if (! lane_1->take_over_feeding()) {
		    lane_2->take_over_feeding();
		}
	    }
	    ms_sleep(100);
	}
    }

    void dump_state() {
	turtleneck->dump_state();
	lane_1->dump_state();
	lane_2->dump_state();
    }

private:
	TurtleNeck *turtleneck;
	Lane *lane_1;
	Lane *lane_2;
};

static void threads_main(int argc, char **argv) {
    Coordinator *coordinator = new Coordinator();
    while (1) {
	static char line[1024];

	if (pi_readline(line, sizeof(line)) != NULL) {
	    if (strcmp(line, "threads") == 0) {
		pi_threads_dump_state();
	    } else if (strcmp(line, "state") == 0) {
		coordinator->dump_state();
	    } else if (strcmp(line, "help") == 0) {
		printf("state: dump state\n");
		printf("threads: dump thread state\n");
	    }
	}
    }
}

int main(int argc, char **argv) {
    pi_init_with_threads(threads_main, argc, argv);
    return 0;
}


