VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	EM_LAST : UDINT;
	T_SETTLE : TON;
	T_GRIP : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
	GONE : UDINT;
	MEM_CV1 : BOOL;
	IND_CV1_LAST : BOOL;
	MEM_CV2 : BOOL;
	IND_CV2_LAST : BOOL;
	MEM_LIFT : BOOL;
	IND_LIFT_LAST : BOOL;
	MEM_TRAV : BOOL;
	IND_TRAV_LAST : BOOL;
	MEM_GRIP : BOOL;
	IND_GRIP_LAST : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// gripper-transfer: a part rides the infeed belt to the pick beam and stops. A 2-finger gripper on
// a pneumatic lift on a pneumatic traverse picks it, carries it over the outfeed belt (running
// across, the other way) and sets it down. Repeat until STOP. The same sequence runs without a
// PLC in gripper-transfer.ctl.js.
//
// A gripper has no "part gripped" switch: after the closing time the fingers are either stopped
// by the part (neither switch on) or fully closed on nothing (CLOSED on). A missed grip is a
// FAULT, not a cycle with an empty gripper.
//
// Waiting steps wait for an INVARIANT, not for a counter to move: every part loaded has left.
//
// The panel has a selector AUTO / INDIVIDUAL: START only in AUTO, the individual buttons only in
// INDIVIDUAL, and a change while the sequence runs is a FAULT, not a pause.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_CSTOP THEN
	STOP_REQ := TRUE;
END_IF;
SEL_CHANGED := SEL_AUTO <> SEL_LAST;
SEL_LAST := SEL_AUTO;

// The master circuit, as on the cell panel: E-STOP is a latching mushroom, MASTER ON energises
// the machine, and nothing runs without it. An E-STOP also loses the home position.
ESTOP := PB_ESTOP;
MASTER_EDGE := PB_MASTER AND NOT PB_MASTER_LAST;
PB_MASTER_LAST := PB_MASTER;
HOME_EDGE := PB_HOME AND NOT PB_HOME_LAST;
PB_HOME_LAST := PB_HOME;
IF ESTOP THEN
	MASTER_ON := FALSE;
	HOMED := FALSE;
ELSIF MASTER_EDGE THEN
	MASTER_ON := TRUE;
END_IF;
READY := MASTER_ON AND NOT ESTOP;

// A write-off goes STALE when the part turns up after all, and then RM + GONE runs PAST EM and
// the step waiting on that invariant stops waiting at all. GONE is a UDINT, so the subtraction
// underflows to about 4 billion when RM is ahead: guard before you subtract.
IF RM_CNT >= EM_CNT THEN
	GONE := 0;
ELSIF RM_CNT + GONE > EM_CNT THEN
	GONE := EM_CNT - RM_CNT;
END_IF;
CASE ST1_STEP OF
	0:	// idle: lift up, traverse in, gripper open
		CV1_RUN := FALSE;
		CV2_RUN := FALSE;
		EM_EMIT := FALSE;
		GRIP_CLOSE := FALSE;
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		SOL_Y_OUT := FALSE;
		SOL_Y_IN := TRUE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_Z_UP AND AS_Y_IN AND GRIP_OPEN THEN
			STOP_REQ := FALSE;
			EM_LAST := EM_CNT;
			ST1_STEP := 10;
		END_IF;
	10:	// outfeed belt on, feed one part
		CV2_RUN := TRUE;
		EM_EMIT := TRUE;
		IF EM_CNT <> EM_LAST THEN
			EM_EMIT := FALSE;
			ST1_STEP := 15;
		END_IF;
	15:	// the pick beam must be clear before this part runs to it
		CV1_RUN := TRUE;
		IF NOT PE_PICK THEN
			ST1_STEP := 20;
		END_IF;
	20:	// carry it to the pick beam
		CV1_RUN := TRUE;
		IF PE_PICK THEN
			CV1_RUN := FALSE;
			ST1_STEP := 30;
		END_IF;
	30:	// let it settle under the gripper
		IF T_SETTLE.Q THEN
			ST1_STEP := 40;
		END_IF;
	40:	// lift down
		SOL_Z_UP := FALSE;
		SOL_Z_DN := TRUE;
		IF AS_Z_DN THEN
			ST1_STEP := 50;
		END_IF;
	50:	// close: stopped by the part = gripped, fully closed = missed
		GRIP_CLOSE := TRUE;
		IF T_GRIP.Q THEN
			IF GRIP_CLOSED THEN
				ST1_STEP := 900;
			ELSE
				ST1_STEP := 60;
			END_IF;
		END_IF;
	60:	// lift up with the part
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		IF AS_Z_UP THEN
			ST1_STEP := 70;
		END_IF;
	70:	// traverse out over the outfeed belt
		SOL_Y_IN := FALSE;
		SOL_Y_OUT := TRUE;
		IF AS_Y_OUT THEN
			ST1_STEP := 80;
		END_IF;
	80:	// lift down onto the belt
		SOL_Z_UP := FALSE;
		SOL_Z_DN := TRUE;
		IF AS_Z_DN THEN
			ST1_STEP := 90;
		END_IF;
	90:	// open: the part is on the belt
		GRIP_CLOSE := FALSE;
		IF GRIP_OPEN THEN
			ST1_STEP := 100;
		END_IF;
	100:	// lift up
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		IF AS_Z_UP THEN
			ST1_STEP := 110;
		END_IF;
	110:	// traverse in
		SOL_Y_OUT := FALSE;
		SOL_Y_IN := TRUE;
		IF AS_Y_IN THEN
			ST1_STEP := 120;
		END_IF;
	120:	// the outfeed belt carries the part away: everything loaded must have left
		IF RM_CNT + GONE >= EM_CNT THEN
			CYCLE_CNT := CYCLE_CNT + 1;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSE
				EM_LAST := EM_CNT;
				ST1_STEP := 10;
			END_IF;
		END_IF;
	800:	// HOME: lift up and traverse in. The gripper is left alone: a real machine does not drop the part it holds
		CV1_RUN := FALSE;
		CV2_RUN := FALSE;
		EM_EMIT := FALSE;
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		SOL_Y_OUT := FALSE;
		SOL_Y_IN := TRUE;
		IF AS_Z_UP AND AS_Y_IN THEN
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds. The holders keep their parts: a real machine does not let go when the power drops.
		CV1_RUN := FALSE;
		EM_EMIT := FALSE;
		SOL_Y_OUT := FALSE;
		SOL_Y_IN := FALSE;
		SOL_Z_DN := FALSE;
		SOL_Z_UP := FALSE;
		CV2_RUN := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving, the gripper closed on nothing, or the selector was turned
		// while running. Belts and feeding off; the gripper and the cylinders stay where they are.
		CV1_RUN := FALSE;
		CV2_RUN := FALSE;
		EM_EMIT := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off whatever never reached the outfeed. Only HERE.
			GONE := EM_CNT - RM_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_SETTLE(In := (ST1_STEP = 30), PT := T#300ms);
T_GRIP(In := (ST1_STEP = 50), PT := T#400ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	CV1_RUN := FALSE;
	CV2_RUN := FALSE;
	EM_EMIT := FALSE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	CV1_RUN := FALSE;
	EM_EMIT := FALSE;
	SOL_Y_OUT := FALSE;
	SOL_Y_IN := FALSE;
	SOL_Z_DN := FALSE;
	SOL_Z_UP := FALSE;
	CV2_RUN := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory per actuator, cleared when INDIVIDUAL
// ends. The feed button pulses one part.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_CV1 AND NOT IND_CV1_LAST THEN
		MEM_CV1 := NOT MEM_CV1;
	END_IF;
	IF IND_CV2 AND NOT IND_CV2_LAST THEN
		MEM_CV2 := NOT MEM_CV2;
	END_IF;
	IF IND_LIFT AND NOT IND_LIFT_LAST THEN
		MEM_LIFT := NOT MEM_LIFT;
	END_IF;
	IF IND_TRAV AND NOT IND_TRAV_LAST THEN
		MEM_TRAV := NOT MEM_TRAV;
	END_IF;
	IF IND_GRIP AND NOT IND_GRIP_LAST THEN
		MEM_GRIP := NOT MEM_GRIP;
	END_IF;
	CV1_RUN := MEM_CV1;
	CV2_RUN := MEM_CV2;
	EM_EMIT := IND_FEED;
	SOL_Z_DN := MEM_LIFT;
	SOL_Z_UP := NOT MEM_LIFT;
	SOL_Y_OUT := MEM_TRAV;
	SOL_Y_IN := NOT MEM_TRAV;
	GRIP_CLOSE := MEM_GRIP;
ELSE
	MEM_CV1 := FALSE;
	MEM_CV2 := FALSE;
	MEM_LIFT := FALSE;
	MEM_TRAV := FALSE;
	MEM_GRIP := FALSE;
END_IF;
IND_CV1_LAST := IND_CV1;
IND_CV2_LAST := IND_CV2;
IND_LIFT_LAST := IND_LIFT;
IND_TRAV_LAST := IND_TRAV;
IND_GRIP_LAST := IND_GRIP;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
