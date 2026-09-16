VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	EM_LAST : UDINT;
	T_SETTLE : TON;
	T_DEMAND : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
	GONE : UDINT;
	MEM_CV : BOOL;
	IND_CV_LAST : BOOL;
	MEM_HOLD : BOOL;
	IND_HOLD_LAST : BOOL;
	MEM_GATE : BOOL;
	IND_GATE_LAST : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// buffer-queue: a stop-and-go escapement. Two stopper cylinders stand 300 mm apart over the belt:
// the HOLD pin meters parts into the pocket between them, the GATE pin holds the metered part
// until the downstream station asks for it. Each pin has its own up and down reed switches, and
// each position has its own photo-eye. The same sequence runs without a PLC in buffer-queue.ctl.js.
//
// ONE part is in the line at a time, and that is not a simplification: a pin cannot come down
// between parts that TOUCH, so an escapement can only work where the parts arrive with a gap
// (measured, tests/rapier.test.js). The loader is therefore metered by the sequence, and the
// HOLD pin always comes down on free belt.
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

CASE ST1_STEP OF
	0:	// idle: belt off, both pins retracted
		CV_RUN := FALSE;
		EM_EMIT := FALSE;
		SOL_HOLD := FALSE;
		SOL_GATE := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_HOLD_UP AND AS_GATE_UP THEN
			STOP_REQ := FALSE;
			ST1_STEP := 10;
		END_IF;
	10:	// belt on, and the gate pin goes down first: nothing may leave until the escapement is armed
		CV_RUN := TRUE;
		SOL_GATE := TRUE;
		IF AS_GATE_DN THEN
			ST1_STEP := 15;
		END_IF;
	15:	// hold pin down: the line is now blocked at both ends
		SOL_HOLD := TRUE;
		IF AS_HOLD_DN THEN
			ST1_STEP := 20;
		END_IF;
	20:	// let one part into the line; the command is held until the loader's counter answers it
		EM_EMIT := TRUE;
		IF EM_CNT <> EM_LAST THEN
			EM_EMIT := FALSE;
			ST1_STEP := 25;
		END_IF;
	25:	// it runs down the belt to the hold pin
		IF PE_HOLD THEN
			ST1_STEP := 30;
		END_IF;
	30:	// let it settle against the pin
		IF T_SETTLE.Q THEN
			ST1_STEP := 40;
		END_IF;
	40:	// GO: the hold pin lifts and this one part runs into the pocket between the pins
		SOL_HOLD := FALSE;
		IF AS_HOLD_UP THEN
			ST1_STEP := 45;
		END_IF;
	45:	// wait until the part has cleared the hold eye
		IF NOT PE_HOLD THEN
			ST1_STEP := 50;
		END_IF;
	50:	// STOP: the pin comes down again on FREE BELT. It can only do that because the next part
		// has not been fed yet - a pin cannot land between parts that touch.
		SOL_HOLD := TRUE;
		IF AS_HOLD_DN THEN
			ST1_STEP := 60;
		END_IF;
	60:	// the part arrives at the gate pin
		IF PE_GATE THEN
			ST1_STEP := 70;
		END_IF;
	70:	// it waits in the pocket until the downstream station asks for it
		IF T_DEMAND.Q THEN
			ST1_STEP := 80;
		END_IF;
	80:	// the gate lifts and the part leaves the pocket
		SOL_GATE := FALSE;
		IF NOT PE_GATE THEN
			ST1_STEP := 90;
		END_IF;
	90:	// gate down again, ready for the next one
		SOL_GATE := TRUE;
		IF AS_GATE_DN THEN
			ST1_STEP := 95;
		END_IF;
	95:	// cycle complete once every part let in has left the belt
		IF RM_CNT + GONE >= EM_CNT THEN
			CYCLE_CNT := CYCLE_CNT + 1;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSE
				EM_LAST := EM_CNT;
				ST1_STEP := 20;
			END_IF;
		END_IF;
	800:	// HOME: both pins retracted, so the line is clear, and the belt is quiet
		CV_RUN := FALSE;
		EM_EMIT := FALSE;
		SOL_HOLD := FALSE;
		SOL_GATE := FALSE;
		IF AS_HOLD_UP AND AS_GATE_UP THEN
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving (a part taken, jammed or lost), or the selector was
		// turned while running.
		CV_RUN := FALSE;
		EM_EMIT := FALSE;
		SOL_HOLD := FALSE;
		SOL_GATE := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off whatever never reached the outfeed. Only HERE.
			GONE := EM_CNT - RM_CNT;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds.
		CV_RUN := FALSE;
		EM_EMIT := FALSE;
		SOL_HOLD := FALSE;
		SOL_GATE := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_SETTLE(In := (ST1_STEP = 30), PT := T#300ms);
T_DEMAND(In := (ST1_STEP = 70), PT := T#1500ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	CV_RUN := FALSE;
	EM_EMIT := FALSE;
	SOL_HOLD := FALSE;
	SOL_GATE := FALSE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	CV_RUN := FALSE;
	EM_EMIT := FALSE;
	SOL_HOLD := FALSE;
	SOL_GATE := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory per actuator, cleared when
// INDIVIDUAL ends. The feed button pulses one part.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_CV AND NOT IND_CV_LAST THEN
		MEM_CV := NOT MEM_CV;
	END_IF;
	IF IND_HOLD AND NOT IND_HOLD_LAST THEN
		MEM_HOLD := NOT MEM_HOLD;
	END_IF;
	IF IND_GATE AND NOT IND_GATE_LAST THEN
		MEM_GATE := NOT MEM_GATE;
	END_IF;
	CV_RUN := MEM_CV;
	EM_EMIT := IND_FEED;
	SOL_HOLD := MEM_HOLD;
	SOL_GATE := MEM_GATE;
ELSE
	MEM_CV := FALSE;
	MEM_HOLD := FALSE;
	MEM_GATE := FALSE;
END_IF;
IND_CV_LAST := IND_CV;
IND_HOLD_LAST := IND_HOLD;
IND_GATE_LAST := IND_GATE;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
