#include "pi.h"
#include <cstring>
#include "gp-input.h"
#include "gp-output.h"
#include "pi-threads.h"
#include "time-utils.h"

// Switch pins (active low, pull-up)
#define PIN_L1_IN      18
#define PIN_L1_OUT     12
#define PIN_L2_IN       2
#define PIN_L2_OUT      3
#define PIN_TURTLENECK_FULL  6
#define PIN_TURTLENECK_EMPTY 7
#define PIN_Y_OUTPUT    4

// Steppers
#define PIN_M1_EN      14
#define PIN_M1_DIR     15
#define PIN_M1_STEP    16
#define PIN_M2_EN       8
#define PIN_M2_DIR      9
#define PIN_M2_STEP    10

#define M1_DIR_INVERT  1
#define M2_DIR_INVERT  0

// -------------------------- END CONFIG --------------------------

#define TRACE_STATE(x) printf("%s => %s\n", name, x)

class Turtleneck : public PiThread {
public:
    Turtleneck(Input *full, Input *empty) : PiThread("turtle-neck"), full(full), empty(empty) {
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
    int sleep_us = 50;

    static const int MIN_SLEEP_US = 50;
    static const int SLEEP_US_DELTA = 10;
};
	
// ---------------------------- Stepper ---------------------------

class Stepper {
public:
    Stepper(Output *en, Output *dir, Output *step_pin) : en(en), dir(dir), step_pin(step_pin) {
    }

    void enable() {
	en->set(false);
    }

    void disable() {
	en->set(true);
    }

    void step(bool forward = true, int delay_us = 0) {
	dir->set(forward);
	step_pin->set(true);
	us_sleep(STEP_PULSE_US);
	step_pin->set(false);
	if (delay_us > STEP_PULSE_US) us_sleep(delay_us - STEP_PULSE_US);
    }

private:
    Output *en;
    Output *dir;
    Output *step_pin;

    static const int STEP_PULSE_US = 5;
};

class Lane : public PiThread {
public:
    Lane(Input *present, Input *loaded, Input *y_output, Stepper *stepper, Turtleneck *turtleneck, const char *name) : PiThread(name), name(name), present(present), loaded(loaded), y_output(y_output), stepper(stepper), turtleneck(turtleneck) {
	lock = new PiMutex();
	start();
    }

    void main() override {
	if (present->get() && loaded->get()) {
	    stepper->enable();
	    state = ACTIVE;
	    TRACE_STATE("active (init)");
	} else if (! present->get() && loaded->get()) {
	    stepper->enable();
	    state = EMPTYING;
	    TRACE_STATE("emptying (init)");
	}

	while (1) {
	    bool feed = false;
	    bool feed_dir = true;


	    lock->lock();

	    switch (state) {
	    case EMPTY:
		if (present->get()) {
		    stepper->enable();
		    state = PRE_LOADING;
		    TRACE_STATE("pre-loading");
		}
		break;
	    case PRE_LOADING:
		// TODO: add a timeout
		if (! present->get()) {
		    stepper->disable();
		    state = EMPTY;
		    TRACE_STATE("empty");
		} else if (loaded->get()) {
		    state = PRE_LOADING_RETRACT;
		    TRACE_STATE("pre-loading (retract)");
		} else {
		    feed = true;
		}
		break;
	    case PRE_LOADING_RETRACT:
		if (! loaded->get()) {
		    stepper->disable();
		    state = READY;
		    TRACE_STATE("ready");
		} else {
		    feed = true;
		    feed_dir = false;
		}
		break;
	    case READY:
		break;
	    case LOADING:
		// TODO: add a timeout in case the filament just isn't loadable and then do something (what??)
		if (y_output->get()) {
		    state = ACTIVE;
		    TRACE_STATE("active");
		} else {
		    feed = true;
		}
		break;
	    case ACTIVE:
		if (! present->get()) {
		    stepper->disable();
		    state = EMPTYING;
		    TRACE_STATE("emptying");
		} else {
		    feed = turtleneck->should_feed();
feed = true;
		}
		break;
	    case EMPTYING:
		if (! y_output->get()) {
		    state = EMPTY;
		    TRACE_STATE("empty");
		} else {
		    feed = turtleneck->should_feed();
feed = true;
		}
		break;
	    case MANUAL:
		break;
	    }

	    lock->unlock();

	    if (feed) {
		stepper->step(feed_dir, turtleneck->get_sleep_us());
	    } else {
		ms_sleep(1);
	    }
	}
    }

    bool is_active() {
	lock->lock();
	bool ret = (state == LOADING || state == ACTIVE || state == EMPTYING);
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
	    TRACE_STATE("loading (take over feeding)");
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
	case PRE_LOADING_RETRACT: printf("pre-loading(retract)"); break;
	case READY: printf("ready"); break;
	case LOADING: printf("loading"); break;
	case ACTIVE: printf("active"); break;
	case EMPTYING: printf("emptying"); break;
	case MANUAL: printf("manual"); break;
	}
	if (present->get()) printf(" present");
	if (loaded->get()) printf(" loaded");
	if (y_output->get()) printf(" y-output");
	printf("\n");

	lock->unlock();
    }

private:
    const char *name;
    Input *present;
    Input *loaded;
    Input *y_output;
    Stepper *stepper;
    Turtleneck *turtleneck;

    PiMutex *lock;
    enum { EMPTY, PRE_LOADING, PRE_LOADING_RETRACT, READY, LOADING, ACTIVE, EMPTYING, MANUAL } state = EMPTY;
};

// ---------------------------- MAIN -----------------------------

static GPInput *new_gpinput(int pin) {
    GPInput *input = new GPInput(pin);
    input->set_pullup_up();
    input->set_debounce(1);
    input->set_is_inverted();
    return input;
}

static Turtleneck *create_turtleneck() {
    Input *full = new_gpinput(PIN_TURTLENECK_FULL);
    Input *empty = new_gpinput(PIN_TURTLENECK_EMPTY);
    return new Turtleneck(full, empty);
}

static Lane *create_lane_1(Input *y_output, Turtleneck *turtleneck) {
    Output *L1_enable = new GPOutput(PIN_M1_EN);
    Output *L1_dir = new GPOutput(PIN_M1_DIR);
    Output *L1_step = new GPOutput(PIN_M1_STEP);
    L1_dir->set_is_inverted(M1_DIR_INVERT);
    Stepper *L1_stepper = new Stepper(L1_enable, L1_dir, L1_step);

    Input *L1_present = new_gpinput(PIN_L1_IN);
    Input *L1_loaded = new_gpinput(PIN_L1_OUT);
    return new Lane(L1_present, L1_loaded, y_output, L1_stepper, turtleneck, "lane-1");
}

static Lane *create_lane_2(Input *y_output, Turtleneck *turtleneck) {
    Output *L2_enable = new GPOutput(PIN_M2_EN);
    Output *L2_dir = new GPOutput(PIN_M2_DIR);
    Output *L2_step = new GPOutput(PIN_M2_STEP);
    L2_dir->set_is_inverted(M2_DIR_INVERT);
    Stepper *L2_stepper = new Stepper(L2_enable, L2_dir, L2_step);

    Input *L2_present = new_gpinput(PIN_L2_IN);
    Input *L2_loaded = new_gpinput(PIN_L2_OUT);
    return new Lane(L2_present, L2_loaded, y_output, L2_stepper, turtleneck, "lane-2");
}

class Coordinator : public PiThread {
public:
    Coordinator() : PiThread("coordinator") {
	Input *y_output = new_gpinput(PIN_Y_OUTPUT);
	turtleneck = create_turtleneck();
	lane_1 = create_lane_1(y_output, turtleneck);
	lane_2 = create_lane_2(y_output, turtleneck);

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
	Turtleneck *turtleneck;
	Lane *lane_1;
	Lane *lane_2;
};

static void threads_main(int argc, char **argv) {
    ms_sleep(2000);
    printf("Starting\n");
    Coordinator *coordinator = new Coordinator();
    printf("Created coordinator, entering interactive loop.\n");
    while (1) {
	static char line[1024];

	if (pi_readline(line, sizeof(line)) != NULL) {
	    if (strcmp(line, "bootsel") == 0) {
		printf("Rebooting to bootloader.\n"); fflush(stdout);
		pi_reboot_bootloader();
	    } else if (strcmp(line, "state") == 0) {
		coordinator->dump_state();
	    } else if (strcmp(line, "threads") == 0) {
		pi_threads_dump_state();
	    } else if (strcmp(line, "help") == 0 || strcmp(line, "?") == 0) {
		printf("bootsel: reboot to bootloader mode\n");
		printf("state: dump state\n");
		printf("threads: dump thread state\n");
	    } else {
		printf("help or ? for usage\n");
	    }
	}
    }
}

int main(int argc, char **argv) {
    pi_init_with_threads(threads_main, argc, argv);
    return 0;
}
