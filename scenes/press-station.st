VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	EM_LAST : UDINT;
	T_PRESS : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
	GONE : UDINT;
	MEM_CLAMP : BOOL;
	IND_CLAMP_LAST : BOOL;
	MEM_PRESS : BOOL;
	IND_PRESS_LAST : BOOL;
	MEM_EJECT : BOOL;
	IND_EJECT_LAST : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// press-station: the feeder drops a part into the clamped nest, the press comes down onto it
// and holds for the press time, comes up, the nest unclamps and the ejector pushes the part
// off the nest onto the chute into the bin. Repeat until STOP. The same sequence runs without
// a PLC in press-station.ctl.js.
//
// The clamp goes on BEFORE the feed: a nest that holds the part locates it square on the pocket
// floor, so the press stroke meets it where the drawing says.
//
// Waiting steps wait for an INVARIANT, not for a counter to move: every part fed has reached
// the bin.
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
	0:	// idle: press up, ejector back, nest free
		CLAMP := FALSE;
		EM_EMIT := FALSE;
		SOL_EJECT := FALSE;
		SOL_P_DN := FALSE;
		SOL_P_UP := TRUE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_P_UP AND AS_EJ_BACK THEN
			STOP_REQ := FALSE;
			EM_LAST := EM_CNT;
			ST1_STEP := 10;
		END_IF;
	10:	// clamp, then feed one part into the nest
		CLAMP := TRUE;
		EM_EMIT := TRUE;
		IF EM_CNT <> EM_LAST THEN
			EM_EMIT := FALSE;
			ST1_STEP := 20;
		END_IF;
	20:	// the nest reports the part
		IF NEST_P THEN
			ST1_STEP := 30;
		END_IF;
	30:	// press down
		SOL_P_UP := FALSE;
		SOL_P_DN := TRUE;
		IF AS_P_DN THEN
			ST1_STEP := 40;
		END_IF;
	40:	// press time
		IF T_PRESS.Q THEN
			ST1_STEP := 50;
		END_IF;
	50:	// press up
		SOL_P_DN := FALSE;
		SOL_P_UP := TRUE;
		IF AS_P_UP THEN
			ST1_STEP := 60;
		END_IF;
	60:	// unclamp and eject onto the chute
		CLAMP := FALSE;
		SOL_EJECT := TRUE;
		IF AS_EJ_OUT THEN
			ST1_STEP := 70;
		END_IF;
	70:	// ejector back, and every part fed has reached the bin
		SOL_EJECT := FALSE;
		IF AS_EJ_BACK AND (RM_CNT + GONE >= EM_CNT) THEN
			ST1_STEP := 80;
		END_IF;
	80:	// cycle complete
		CYCLE_CNT := CYCLE_CNT + 1;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			EM_LAST := EM_CNT;
			ST1_STEP := 10;
		END_IF;
	800:	// HOME: press up, ejector back. The clamp is left alone: a real machine does not drop the part it holds
		EM_EMIT := FALSE;
		SOL_EJECT := FALSE;
		SOL_P_DN := FALSE;
		SOL_P_UP := TRUE;
		IF AS_P_UP AND AS_EJ_BACK THEN
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds. The holders keep their parts: a real machine does not let go when the power drops.
		EM_EMIT := FALSE;
		SOL_P_DN := FALSE;
		SOL_P_UP := FALSE;
		SOL_EJECT := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving (part taken out of the nest, never arrived, never reached
		// the bin), or the selector was turned while running. Feeding off, press up, ejector
		// back; the clamp keeps whatever it holds.
		EM_EMIT := FALSE;
		SOL_EJECT := FALSE;
		SOL_P_DN := FALSE;
		SOL_P_UP := TRUE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off whatever never reached the bin. Only HERE.
			GONE := EM_CNT - RM_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_PRESS(In := (ST1_STEP = 40), PT := T#500ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	EM_EMIT := FALSE;
	SOL_EJECT := FALSE;
	SOL_P_DN := FALSE;
	SOL_P_UP := TRUE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	EM_EMIT := FALSE;
	SOL_P_DN := FALSE;
	SOL_P_UP := FALSE;
	SOL_EJECT := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory per actuator, cleared when INDIVIDUAL
// ends. The feed button pulses one part.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_CLAMP AND NOT IND_CLAMP_LAST THEN
		MEM_CLAMP := NOT MEM_CLAMP;
	END_IF;
	IF IND_PRESS AND NOT IND_PRESS_LAST THEN
		MEM_PRESS := NOT MEM_PRESS;
	END_IF;
	IF IND_EJECT AND NOT IND_EJECT_LAST THEN
		MEM_EJECT := NOT MEM_EJECT;
	END_IF;
	EM_EMIT := IND_FEED;
	CLAMP := MEM_CLAMP;
	SOL_P_DN := MEM_PRESS;
	SOL_P_UP := NOT MEM_PRESS;
	SOL_EJECT := MEM_EJECT;
ELSE
	MEM_CLAMP := FALSE;
	MEM_PRESS := FALSE;
	MEM_EJECT := FALSE;
END_IF;
IND_CLAMP_LAST := IND_CLAMP;
IND_PRESS_LAST := IND_PRESS;
IND_EJECT_LAST := IND_EJECT;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
