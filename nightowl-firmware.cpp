 #include "pi.h"
#include <cstring>
#include <math.h>
#include "gp-input.h"
#include "gp-output.h"
#include "pi-threads.h"
#include "stepper.h"
#include "string-utils.h"
#include "thread-interrupt-notifier.h"
#include "time-utils.h"
#include "tmc2209.h"

// Switch pins (active low, pull-up)
#define PIN_L1_IN      26
#define PIN_L1_OUT     27
#define PIN_L2_IN       4
#define PIN_L2_OUT      3
#define PIN_TURTLENECK_FULL  16
#define PIN_TURTLENECK_EMPTY 22
#define PIN_Y_OUTPUT    25

// Steppers
#define TMC_UART_TX	8
#define TMC_UART_RX	9
// M1 is on E0
#define PIN_M1_EN      15
#define PIN_M1_DIR     13
#define PIN_M1_STEP    14
#define PIN_M1_ADDRESS  3

// M2 ia on X
#define PIN_M2_EN      12
#define PIN_M2_DIR     10
#define PIN_M2_STEP    11
#define PIN_M2_ADDRESS  0

#define M1_DIR_INVERT  0
#define M2_DIR_INVERT  0

// Configuration Values

static const int microstepping = 16;
#define STEPS_PER_MM	(680*microstepping/16)

#define ACTIVE_INIT_MM	2500

#define PRELOAD_SPEED	5		// mm/sec
#define LOADING_SPEED	10
#define REFILL_SPEED	10

// -------------------------- END CONFIG --------------------------

#define TRACE_STATE(x) printf("%s => %s\n", name, x)

class Switch {
public:
    Switch(int pin, ThreadInterruptNotifier *notifier) : pin(pin) {
	input = new GPInput(pin);
	input->set_pullup_up();
	input->set_debounce_us(50);
	input->set_is_inverted();
	input->set_notifier(notifier);
	update();
    }

    bool update() {
	bool new_value = input->get();
	bool updated = new_value != value;
	value = new_value;
	return updated;
    }

    bool get() {
	return value;
    }

private:
    int pin;
    Input *input;
    bool value;
};

class LaneSwitches {
public:
    LaneSwitches(int present_pin, int loaded_pin, ThreadInterruptNotifier *notifier) {
	present_switch = new Switch(present_pin, notifier);
	loaded_switch = new Switch(loaded_pin, notifier);
	update();
    }

    bool update() {
	bool p = present_switch->update();
	bool l = loaded_switch->update();
	return p || l;
    }

    bool is_present() { return present_switch->get(); }
    bool is_loaded() { return loaded_switch->get(); }

    void dump_state() {
	if (is_present()) printf(" is-present");
	if (is_loaded()) printf(" is-loaded");
    }

private:
    Switch *present_switch;
    Switch *loaded_switch;
};

class OutputSwitches {
public:
    OutputSwitches(ThreadInterruptNotifier *notifier) {
	y_output_switch = new Switch(PIN_Y_OUTPUT, notifier);
	buffer_full_switch = new Switch(PIN_TURTLENECK_FULL, notifier);
	buffer_empty_switch = new Switch(PIN_TURTLENECK_EMPTY, notifier);
	update();
    }

    bool update() {
	bool o = y_output_switch->update();
	bool f = buffer_full_switch->update();
	bool e = buffer_empty_switch->update();
	return o || f || e;
    }

    bool has_y_output() { return y_output_switch->get(); }
    bool buffer_is_full() { return buffer_full_switch->get(); }
    bool buffer_is_empty() { return buffer_empty_switch->get(); }

    void dump_state() {
	if (has_y_output()) printf(" has-y-output");
	if (buffer_is_full()) printf(" buffer-full");
	if (buffer_is_empty()) printf(" buffer-empty");
    }

private:
    Switch *y_output_switch;
    Switch *buffer_full_switch;
    Switch *buffer_empty_switch;
};

class LaneStateNotifier {
public:
    virtual void lane_is_inactive(class Lane *) = 0;
    virtual void lane_is_ready(class Lane *) = 0;
};

class Lane {
public:
    Lane(LaneSwitches *lane_switches, OutputSwitches *output_switches, Stepper *stepper, const char *name) : name(name), lane_switches(lane_switches), output_switches(output_switches), stepper(stepper) {
	bool is_present = lane_switches->is_present();
	bool is_loaded = lane_switches->is_loaded();

	state = EMPTY;
	if (is_present && is_loaded) state = EARLY_ACTIVE_INIT;
	if (! is_present && is_loaded) state = EMPTYING;

        printf("%s: initial state: %s\n", name, state_to_string(state));
    }

    void update() {
	bool is_present = lane_switches->is_present();
        bool is_loaded = lane_switches->is_loaded();
	bool buffer_is_full = output_switches->buffer_is_full();
	bool buffer_is_empty = output_switches->buffer_is_empty();
	bool has_y_output = output_switches->has_y_output();

	int feed;
	enum State old_state;

	do {
	    feed = 0;
	    old_state = state;

	    switch (state) {
	    case EMPTY:
		if (is_present) state = PRE_LOADING;
		break;
	    case PRE_LOADING:
		// TODO: add a timeout
		if (! is_present) state = EMPTY;
		else if (is_loaded) state = PRE_LOADING_RETRACT;
		else feed = PRELOAD_SPEED;
		break;
	    case PRE_LOADING_RETRACT:
		if (! is_present) state = EMPTY;
		else if (! is_loaded) state = READY;
		else feed = -PRELOAD_SPEED;
		break;
	    case READY:
		break;
	    case ACTIVATING:
		if (is_loaded) state = LOADING;
		else if (! is_present) state = EMPTY;
		else feed = LOADING_SPEED;
		stepper->reset_n_steps();
		break;
	    case LOADING:
		// TODO: add a timeout in case the filament just isn't loadable and then do something (what??)
		if (! is_loaded && ! has_y_output) state = EMPTY;
		else if (has_y_output) state = EARLY_ACTIVE_INIT;
		// TODO: else if (buffer_is_empty) really short filament in here somewhere??
		else feed = LOADING_SPEED;
		break;
	    case EARLY_ACTIVE_INIT:
		active_init_until = stepper->get_n_steps() + (ACTIVE_INIT_MM * STEPS_PER_MM);
		state = EARLY_ACTIVE;
		break;
	    case EARLY_ACTIVE:
		if (! is_present) state = EMPTYING;
		else if (stepper->get_n_steps() >= active_init_until) state = ACTIVE;
		else if (buffer_is_full) state = EARLY_ACTIVE_WAITING;
		else feed = REFILL_SPEED;
		break;
	    case EARLY_ACTIVE_WAITING:
		if (! is_present) state = EMPTYING;
		else if (! buffer_is_full) state = EARLY_ACTIVE;
		break;
	    case ACTIVE:
		if (! is_present) state = EMPTYING;
		else if (buffer_is_full) state = WAITING;
		else feed = REFILL_SPEED;
		break;
	    case WAITING:
		if (! is_present) state = EMPTYING;
		else if (buffer_is_empty) state = ACTIVE;
		break;
	    case EMPTYING:
		if (! has_y_output) state = EMPTY;
		else if (buffer_is_full) state = EMPTYING_WAITING;
		else feed = REFILL_SPEED;
		break;
	    case EMPTYING_WAITING:
		if (! has_y_output) state = EMPTY;
		else if (buffer_is_empty) state = EMPTYING;
		break;
	    case MANUAL:
		break;
	    }

	    trace_state(old_state);
        } while (state != old_state);

	stepper->set_speed(feed);
    }

public:
    bool is_active() {
	return state > READY;
    }

    bool is_ready() {
	return state == READY;
    }

    void activate() {
	assert (state == READY);
	state = ACTIVATING;
	update();
    }

    void dump_state() {
	printf("%s: %s", name, state_to_string(state));
	lane_switches->dump_state();
	if (is_active()) output_switches->dump_state();
	printf(" << ");
	stepper->dump_state();
	printf(" >>");
    }

    void error() {
	stepper->set_speed(0);
    }

private:
    const char *name;
    LaneSwitches *lane_switches;
    OutputSwitches *output_switches;
    Stepper *stepper;

    int64_t active_init_until = 0;

    enum State {
	    EMPTY, PRE_LOADING, PRE_LOADING_RETRACT, READY,
	    ACTIVATING, LOADING,
	    EARLY_ACTIVE_INIT, EARLY_ACTIVE, EARLY_ACTIVE_WAITING,
	    ACTIVE, WAITING,
	    EMPTYING, EMPTYING_WAITING,
	    MANUAL
	} state = EMPTY;

private:
    const char *state_to_string(enum State state) {
	switch (state) {
	case EMPTY: return "empty";
	case PRE_LOADING: return "pre-loading";
	case PRE_LOADING_RETRACT: return "pre-loading(retract)";
	case READY: return "ready";
	case ACTIVATING: return "activating";
	case LOADING: return "loading";
	case EARLY_ACTIVE_INIT: return "init(early)";
	case EARLY_ACTIVE: return "active(early)";
	case EARLY_ACTIVE_WAITING: return "waiting(early)";
	case ACTIVE: return "active";
	case WAITING: return "waiting";
	case EMPTYING: return "emptying";
	case EMPTYING_WAITING: return "emptying-waiting";
	case MANUAL: return "manual";
	}
	return "** INVALID STATE**";
    }

    inline void trace_state(enum State old_state) {
	if (state != old_state) printf("%s: %s => %s\n", name, state_to_string(old_state), state_to_string(state));
    }
};

// ---------------------------- MAIN -----------------------------

static void configure_tmc(UART_Tx *tx, int address) {
    TMC2209 *tmc = new TMC2209(tx, address);
    tmc->set_microstepping(microstepping);
    tmc->set_rms_current(200);
}

static Stepper *create_lane_1_stepper(UART_Tx *tx) {
    Output *enable = new GPOutput(PIN_M1_EN);
    Output *dir = new GPOutput(PIN_M1_DIR);
    Output *step = new GPOutput(PIN_M1_STEP);
    dir->set_is_inverted(M1_DIR_INVERT);
    configure_tmc(tx, PIN_M1_ADDRESS);
    Stepper *stepper = new Stepper(enable, dir, step, "stepper-1");
    stepper->set_steps_per_mm(STEPS_PER_MM);
    return stepper;
}

static Stepper *create_lane_2_stepper(UART_Tx *tx) {
    Output *enable = new GPOutput(PIN_M2_EN);
    Output *dir = new GPOutput(PIN_M2_DIR);
    Output *step = new GPOutput(PIN_M2_STEP);
    dir->set_is_inverted(M2_DIR_INVERT);
    configure_tmc(tx, PIN_M2_ADDRESS);
    Stepper *stepper = new Stepper(enable, dir, step, "stepper-2");
    stepper->set_steps_per_mm(STEPS_PER_MM);
    return stepper;
}

class Coordinator : ThreadInterruptNotifier {
public:
    Coordinator() : ThreadInterruptNotifier("coordinator") {
	tx = new UART_Tx(TMC_UART_TX, 115200);
	output_switches = new OutputSwitches(this);
	lane_1_switches = new LaneSwitches(PIN_L1_IN, PIN_L1_OUT, this);
	lane_2_switches = new LaneSwitches(PIN_L2_IN, PIN_L2_OUT, this);
	lane_1 = new Lane(lane_1_switches, output_switches, create_lane_1_stepper(tx), "lane-1");
	lane_2 = new Lane(lane_2_switches, output_switches, create_lane_2_stepper(tx), "lane-2");

	update(true);

	// TODO: What to do here!?!?!
	while (lane_1->is_active() && lane_2->is_active()) {
	    lane_1->error();
	    lane_2->error();
	    printf("\n\nFATAL ERROR: both lanes think they are active!!\n");
	    ms_sleep(1000);
	}

	if (lane_1->is_active()) active_lane = lane_1;
	if (lane_2->is_active()) active_lane = lane_2;
    }

    void on_change_safe() override {
	update();
    }

    void update(bool force = true) {
	bool output_changed = output_switches->update();
	bool l1_changed = lane_1_switches->update();
	bool l2_changed = lane_2_switches->update();

	if (force || l1_changed || (active_lane == lane_1 && output_changed)) lane_1->update();
	if (force || l2_changed || (active_lane == lane_2 && output_changed)) lane_2->update();

	if (active_lane && ! active_lane->is_active()) active_lane = NULL;

	if (! active_lane) {
	    if (lane_1->is_ready()) active_lane = lane_1;
	    else if (lane_2->is_ready()) active_lane = lane_2;
	    if (active_lane) {
		active_lane->activate();
		printf("activated: ");
		active_lane->dump_state();
	    }
	}
    }
	
    void dump_state() {
	printf("======== Current State ===============\n");
	if (active_lane) printf("%s: ", lane_1 == active_lane ? "ACTIVE" : "      ");
	lane_1->dump_state();
	printf("\n");
	if (active_lane) printf("%s: ", lane_2 == active_lane ? "ACTIVE" : "      ");
	lane_2->dump_state();
	printf("\n");
	printf("all switches: lane_1:");
	lane_1_switches->dump_state();
	printf(" || lane_2:");
	lane_2_switches->dump_state();
	printf(" || output:");
	output_switches->dump_state();
	printf("\n");
    }

private:
    void activate() {
	if (active_lane) return;
    }

private:
    UART_Tx *tx;
    OutputSwitches *output_switches;
    LaneSwitches *lane_1_switches;
    LaneSwitches *lane_2_switches;
    Lane *lane_1;
    Lane *lane_2;

    Lane *active_lane = NULL;
};

class StateDumper : public PiThread {
public:
    StateDumper(Coordinator *coordinator) : PiThread("state-dumper"), coordinator(coordinator) {
	lock = new PiMutex();
	cond = new PiCond();
	start();
    }

    void main(void) {
	lock->lock();
	while (1) {
	    while (! enabled) {
		cond->wait(lock);
	    }
	    coordinator->dump_state();
	    ms_sleep(5000);
	}
    }

    void enable() {
	enabled = true;
	cond->signal();
    }

    void disable() {
	enabled = false;
    }

private:
    Coordinator *coordinator;
    PiMutex *lock;
    PiCond *cond;
    bool enabled = false;
};

static void threads_main(int argc, char **argv) {
    ms_sleep(2000);
    printf("Starting\n");
    Coordinator *coordinator = new Coordinator();
    StateDumper *state_dumper = new StateDumper(coordinator);

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
	    } else if (strncmp(line, "state-dumper", 12) == 0) {
		int enabled = true;
		sscanf(&line[12], "%d", &enabled);
		if (enabled) state_dumper->enable();
		else state_dumper->disable();
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
