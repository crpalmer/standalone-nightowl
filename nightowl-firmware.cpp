 #include "pi.h"
#include <cstring>
#include <math.h>
#include "gp-input.h"
#include "gp-output.h"
#include "pi-threads.h"
#include "string-utils.h"
#include "thread-interrupt-notifier.h"
#include "time-utils.h"

// Switch pins (active low, pull-up)
#define PIN_L1_IN      18
#define PIN_L1_OUT     12
#define PIN_L2_IN       2
#define PIN_L2_OUT      3
#define PIN_TURTLENECK_FULL  25
#define PIN_TURTLENECK_EMPTY 24
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

// Configuration Values

#define STEPS_PER_MM	415
#define PRELOAD_SPEED	2		// mm/sec
#define LOADING_SPEED	20
#define REFILL_SPEED	20

// -------------------------- END CONFIG --------------------------

#define TRACE_STATE(x) printf("%s => %s\n", name, x)

static us_time_t us_delay_for(int mm_sec) {
    us_time_t steps_per_sec = mm_sec * STEPS_PER_MM;
    return 1000000/steps_per_sec;
}

class Turtleneck : public ThreadInterruptNotifier {
public:
    Turtleneck(Input *full_switch, Input *empty_switch, Input *y_output_switch) : ThreadInterruptNotifier("turtleneck"), full_switch(full_switch), empty_switch(empty_switch), y_output_switch(y_output_switch) {
	on_change_safe();

	full_switch->set_notifier(this);
	empty_switch->set_notifier(this);
	y_output_switch->set_notifier(this);
    }

    void on_change_safe() override {
	full = full_switch->get();
	empty = empty_switch->get();
	y_output = y_output_switch->get();

	switch (state) {
	case NO_FILAMENT:
	    if (y_output) state = FEEDING;
	    break;
	case FEEDING:
	    if (! y_output) state = NO_FILAMENT;
	    else if (full) state = WAITING;
	    break;
	case WAITING:
	    if (! y_output) state = NO_FILAMENT;
	    else if (empty) state = FEEDING;
	    break;
	}
    }

    bool has_filament() {
	return state != NO_FILAMENT;
    }

    bool should_feed() {
	return state == FEEDING;
    }

    void dump_state() {
	printf("turtleneck: %s [", state_to_string());
	if (full) printf(" full");
	if (empty) printf(" empty");
	printf(" ], y-output: %s\n", y_output ? "filament-in-y" : "no-filament-in-y");
    }

private:
    Input *full_switch;
    Input *empty_switch;
    Input *y_output_switch;

    bool full = false;
    bool empty = false;
    bool y_output = false;

    enum { NO_FILAMENT, FEEDING, WAITING } state = NO_FILAMENT;

    const char *state_to_string() {
	switch(state) {
	case NO_FILAMENT: return "no-filament";
	case FEEDING: return "feeding";
	case WAITING: return "waiting";
	}
	return "** INVALID STATE **";
    }
};
	
// ---------------------------- Stepper ---------------------------

class Stepper {
public:
    Stepper(Output *en, Output *dir, Output *step_pin) : en(en), dir(dir), step_pin(step_pin) {
	en->set(! enabled);
    }

    void disable() {
	if (enabled) {
	    en->set(true);
	    enabled = false;
	}
    }

    void step(int feed) {
	if (! feed) return;

	enable();
	dir->set(feed > 0);
	step_pin->set(true);
	us_sleep(STEP_PULSE_US);
	step_pin->set(false);

	us_time_t delay_us = us_delay_for(fabs(feed));
	if (delay_us > STEP_PULSE_US) us_sleep(delay_us - STEP_PULSE_US);
    }

private:
    Output *en;
    Output *dir;
    Output *step_pin;

    bool enabled = false;

    static const int STEP_PULSE_US = 5;

private:
    void enable() {
	if (! enabled) {
	    en->set(false);
	    enabled = true;
	}
    }

};

class LaneFilamentState : public ThreadInterruptNotifier {
public:
    LaneFilamentState(Input *present, Input *loaded, const char *name) : ThreadInterruptNotifier(name), present(present), loaded(loaded) {
	lock = new PiMutex();
	present_value = present->get();
	loaded_value = loaded->get();

	present->set_notifier(this);
	loaded->set_notifier(this);
    }

    void on_change_safe() {
	bool p = present->get();
	bool l = loaded->get();
	lock->lock();
	present_value = p;
	loaded_value = l;
	lock->unlock();
    }

    bool is_present() {
	lock->lock();
	bool ret = present_value;
	lock->unlock();
	return ret;
    }

    bool is_loaded() {
	lock->lock();
	bool ret = loaded_value;
	lock->unlock();
	return ret;
    }

    void dump_state() {
	lock->lock();
	if (present_value) printf(" present");
	if (loaded_value) printf(" loaded");
	lock->unlock();
    }

private:
    Input *present;
    Input *loaded;
    PiMutex *lock;
    bool present_value;
    bool loaded_value;
};

class Lane : public PiThread {
public:
    Lane(Input *present, Input *loaded, Stepper *stepper, Turtleneck *turtleneck, const char *name) : PiThread(name), name(name), stepper(stepper), turtleneck(turtleneck) {
	char *state_name = maprintf("state-%s", name);
	filament = new LaneFilamentState(present, loaded, state_name);
	start();
    }

    void main() override {
	if (filament->is_present() && filament->is_loaded()) state = LOADING;
	else if (! filament->is_present() && filament->is_loaded()) state = EMPTYING;
	else state = EMPTY;

        printf("Initial state: %s\n", state_to_string(state));

	while (1) {
	    int feed = 0;
	    enum State old_state = state;

	    switch (state) {
	    case EMPTY:
		if (filament->is_present()) state = PRE_LOADING;
		break;
	    case PRE_LOADING:
		// TODO: add a timeout
		if (! filament->is_present()) state = EMPTY;
		else if (filament->is_loaded()) state = PRE_LOADING_RETRACT;
		else feed = PRELOAD_SPEED;
		break;
	    case PRE_LOADING_RETRACT:
		if (! filament->is_loaded()) state = READY;
		else feed = -PRELOAD_SPEED;
		break;
	    case READY:
		break;
	    case LOADING:
		// TODO: add a timeout in case the filament just isn't loadable and then do something (what??)
		if (turtleneck->has_filament()) state = ACTIVE;
		else feed = LOADING_SPEED;
		break;
	    case ACTIVE:
		if (! filament->is_present()) state = EMPTYING;
		else if (turtleneck->should_feed()) feed = REFILL_SPEED;
		break;
	    case EMPTYING:
		if (! turtleneck->has_filament()) state = EMPTY;
		else if (turtleneck->should_feed()) feed = REFILL_SPEED;
		break;
	    case MANUAL:
		break;
	    }

	    trace_state(old_state);

	    if (feed) {
		stepper->step(feed);
		yield();
	    } else {
		stepper->disable();
		ms_sleep(1);
	    }
	}
    }

    bool is_active() {
	bool ret = (state == LOADING || state == ACTIVE || state == EMPTYING);

	return ret;
    }
	
    bool take_over_feeding() {
	bool can_take_over = false;

	if (state == READY) {
	    can_take_over = true;
	    state = LOADING;
	    trace_state(READY);
	}

	return can_take_over;
    }

    void dump_state() {
	printf("%s: %s", name, state_to_string(state));
	filament->dump_state();
	printf("\n");
    }

private:
    const char *name;
    Stepper *stepper;
    Turtleneck *turtleneck;

    LaneFilamentState *filament;
    enum State { EMPTY, PRE_LOADING, PRE_LOADING_RETRACT, READY, LOADING, ACTIVE, EMPTYING, MANUAL } state = EMPTY;

    const char *state_to_string(enum State state) {
	switch (state) {
	case EMPTY: return "empty";
	case PRE_LOADING: return "pre-loading";
	case PRE_LOADING_RETRACT: return "pre-loading(retract)";
	case READY: return "ready";
	case LOADING: return "loading";
	case ACTIVE: return "active";
	case EMPTYING: return "emptying";
	case MANUAL: return "manual";
	}
	return "** INVALID STATE**";
    }

    inline void trace_state(enum State old_state) {
	if (state != old_state) printf("%s: %s => %s\n", name, state_to_string(old_state), state_to_string(state));
    }
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
    Input *y_output = new_gpinput(PIN_Y_OUTPUT);
    return new Turtleneck(full, empty, y_output);
}

static Lane *create_lane_1(Turtleneck *turtleneck) {
    Output *L1_enable = new GPOutput(PIN_M1_EN);
    Output *L1_dir = new GPOutput(PIN_M1_DIR);
    Output *L1_step = new GPOutput(PIN_M1_STEP);
    L1_dir->set_is_inverted(M1_DIR_INVERT);
    Stepper *L1_stepper = new Stepper(L1_enable, L1_dir, L1_step);

    Input *L1_present = new_gpinput(PIN_L1_IN);
    Input *L1_loaded = new_gpinput(PIN_L1_OUT);
    return new Lane(L1_present, L1_loaded, L1_stepper, turtleneck, "lane-1");
}

static Lane *create_lane_2(Turtleneck *turtleneck) {
    Output *L2_enable = new GPOutput(PIN_M2_EN);
    Output *L2_dir = new GPOutput(PIN_M2_DIR);
    Output *L2_step = new GPOutput(PIN_M2_STEP);
    L2_dir->set_is_inverted(M2_DIR_INVERT);
    Stepper *L2_stepper = new Stepper(L2_enable, L2_dir, L2_step);

    Input *L2_present = new_gpinput(PIN_L2_IN);
    Input *L2_loaded = new_gpinput(PIN_L2_OUT);
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
