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
#define PIN_L1_IN      12
#define PIN_L1_OUT     18
#define PIN_L2_IN       2
#define PIN_L2_OUT      3
#define PIN_TURTLENECK_FULL  25
#define PIN_TURTLENECK_EMPTY 24
#define PIN_Y_OUTPUT    4

// Steppers
#define PIN_M1_EN      14
#define PIN_M1_DIR     15
#define PIN_M1_STEP    16
#define PIN_M1_UART    17

#define PIN_M2_EN       8
#define PIN_M2_DIR      9
#define PIN_M2_STEP    10
#define PIN_M2_UART    11

#define M1_DIR_INVERT  1
#define M2_DIR_INVERT  0

// Thread priorities

#define STEPPER_PRIORITY    3
#define COORDINATOR_PRIORITY 2

// Configuration Values

#define STEPS_PER_MM	415
#define PRELOAD_SPEED	2		// mm/sec
#define LOADING_SPEED	20
#define REFILL_SPEED	20

// -------------------------- END CONFIG --------------------------

#define TRACE_STATE(x) printf("%s => %s\n", name, x)

class Switch {
public:
    Switch(int pin, ThreadInterruptNotifier *notifier) {
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

	if (is_present && ! is_loaded) state = LOADING;
	else if (! is_present && is_loaded) state = EMPTYING;
	else state = EMPTY;

        printf("Initial state: %s\n", state_to_string(state));
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
	    case LOADING:
		// TODO: add a timeout in case the filament just isn't loadable and then do something (what??)
		if (has_y_output) state = ACTIVE;
		// TODO: else if (buffer_is_empty) really short filament in here somewhere??
		else feed = LOADING_SPEED;
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
	state = ACTIVE;
	update();
    }

    void dump_state() {
	printf("%s: %s", name, state_to_string(state));
	lane_switches->dump_state();
	if (is_active()) output_switches->dump_state();
	printf("\n");
    }

private:
    const char *name;
    LaneSwitches *lane_switches;
    OutputSwitches *output_switches;
    Stepper *stepper;

    enum State { EMPTY, PRE_LOADING, PRE_LOADING_RETRACT, READY, LOADING, ACTIVE, WAITING, EMPTYING, EMPTYING_WAITING, MANUAL } state = EMPTY;

private:
    const char *state_to_string(enum State state) {
	switch (state) {
	case EMPTY: return "empty";
	case PRE_LOADING: return "pre-loading";
	case PRE_LOADING_RETRACT: return "pre-loading(retract)";
	case READY: return "ready";
	case LOADING: return "loading";
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

static Stepper *create_lane_1_stepper() {
    Output *enable = new GPOutput(PIN_M1_EN);
    Output *dir = new GPOutput(PIN_M1_DIR);
    Output *step = new GPOutput(PIN_M1_STEP);
    dir->set_is_inverted(M1_DIR_INVERT);
    //UART_Tx *tx = new UART_Tx(PIN_M1_UART, 115200);
    //TMC2209 *tmc = new TMC2209(tx, 3);
    Stepper *stepper = new Stepper(enable, dir, step, "stepper-1", STEPPER_PRIORITY);
    stepper->set_steps_per_mm(STEPS_PER_MM);
    return stepper;
}

static Stepper *create_lane_2_stepper() {
    Output *enable = new GPOutput(PIN_M2_EN);
    Output *dir = new GPOutput(PIN_M2_DIR);
    Output *step = new GPOutput(PIN_M2_STEP);
    dir->set_is_inverted(M2_DIR_INVERT);
    //UART_Tx *tx = new UART_Tx(PIN_M2_UART, 115200);
    //TMC2209 *tmc = new TMC2209(tx, 3);
    Stepper *stepper = new Stepper(enable, dir, step, "stepper-2", STEPPER_PRIORITY);
    stepper->set_steps_per_mm(STEPS_PER_MM);
    return stepper;
}

class Coordinator : ThreadInterruptNotifier {
public:
    Coordinator() : ThreadInterruptNotifier("coordinator", COORDINATOR_PRIORITY) {
	output_switches = new OutputSwitches(this);
	lane_1_switches = new LaneSwitches(PIN_L1_IN, PIN_L1_OUT, this);
	lane_2_switches = new LaneSwitches(PIN_L2_IN, PIN_L2_OUT, this);
	lane_1 = new Lane(lane_1_switches, output_switches, create_lane_1_stepper(), "lane-1");
	lane_2 = new Lane(lane_2_switches, output_switches, create_lane_2_stepper(), "lane-2");

	update(true);
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
	if (! active_lane) activate();
    }
	
    void dump_state() {
	lane_1->dump_state();
	lane_2->dump_state();
    }

private:
    void activate() {
	if (active_lane) return;
	if (lane_1->is_ready()) active_lane = lane_1;
	else if (lane_2->is_ready()) active_lane = lane_2;
	if (active_lane) active_lane->activate();
    }

private:
    OutputSwitches *output_switches;
    LaneSwitches *lane_1_switches;
    LaneSwitches *lane_2_switches;
    Lane *lane_1;
    Lane *lane_2;

    Lane *active_lane = NULL;
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
