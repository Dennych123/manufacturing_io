VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	EM_B_LAST : UDINT;
	EM_L_LAST : UDINT;
	NEXT_ST : INT;
	T_PRESS : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
	MEM_PRESS : BOOL;
	IND_PRESS_LAST : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// assembler: a 4-station cam indexer. Every station works during the same dwell - load a base,
// drop a lid on it, press it - and then the table turns one step, carrying each assembly to the
// next station. The unload station simply takes whatever reaches it. The same sequence runs
// without a PLC in assembler.ctl.js.
//
// The index waits for an INVARIANT, TBL_STATION = NEXT_ST, not for the station to change: a
// change cannot say which index it belonged to, and OPC UA sampling can miss a short state.
// Each feeder command is held until that feeder's own counter answers it.
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
	0:	// idle, table stopped in a dwell, press up
		TBL_RUN := FALSE;
		EM_B_EMIT := FALSE;
		EM_L_EMIT := FALSE;
		SOL_PRESS_DN := FALSE;
		SOL_PRESS_UP := TRUE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND TBL_INPOS AND AS_PRESS_UP THEN
			STOP_REQ := FALSE;
			EM_B_LAST := EM_B_CNT;
			ST1_STEP := 10;
		END_IF;
	10:	// station 0: load a base into the nest
		EM_B_EMIT := TRUE;
		IF EM_B_CNT <> EM_B_LAST THEN
			EM_B_EMIT := FALSE;
			EM_L_LAST := EM_L_CNT;
			ST1_STEP := 20;
		END_IF;
	20:	// station 1: drop a lid onto the base that arrived there
		EM_L_EMIT := TRUE;
		IF EM_L_CNT <> EM_L_LAST THEN
			EM_L_EMIT := FALSE;
			ST1_STEP := 30;
		END_IF;
	30:	// station 2: press down
		SOL_PRESS_UP := FALSE;
		SOL_PRESS_DN := TRUE;
		IF AS_PRESS_DN THEN
			ST1_STEP := 40;
		END_IF;
	40:	// hold the press on the lid
		IF T_PRESS.Q THEN
			ST1_STEP := 50;
		END_IF;
	50:	// press up, and pick the station this index must reach
		SOL_PRESS_DN := FALSE;
		SOL_PRESS_UP := TRUE;
		IF AS_PRESS_UP THEN
			NEXT_ST := (TBL_STATION + 1) MOD 4;
			ST1_STEP := 60;
		END_IF;
	60:	// turn the table one station
		TBL_RUN := TRUE;
		IF TBL_INPOS AND TBL_STATION = NEXT_ST THEN
			TBL_RUN := FALSE;
			CYCLE_CNT := CYCLE_CNT + 1;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSE
				EM_B_LAST := EM_B_CNT;
				ST1_STEP := 10;
			END_IF;
		END_IF;
	800:	// HOME: press up, and the table turns until its origin station is back under the loader: a real home
		EM_B_EMIT := FALSE;
		EM_L_EMIT := FALSE;
		SOL_PRESS_DN := FALSE;
		SOL_PRESS_UP := TRUE;
		TBL_RUN := NOT TBL_ORIGIN;
		IF AS_PRESS_UP AND TBL_ORIGIN THEN
			TBL_RUN := FALSE;
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds. The holders keep their parts: a real machine does not let go when the power drops.
		TBL_RUN := FALSE;
		EM_B_EMIT := FALSE;
		EM_L_EMIT := FALSE;
		SOL_PRESS_DN := FALSE;
		SOL_PRESS_UP := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a feeder never answered, the table never landed, or the selector was turned
		// while running. Table and feeders off, press up. The nests keep what they hold.
		TBL_RUN := FALSE;
		EM_B_EMIT := FALSE;
		EM_L_EMIT := FALSE;
		SOL_PRESS_DN := FALSE;
		SOL_PRESS_UP := TRUE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_PRESS(In := (ST1_STEP = 40), PT := T#300ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	TBL_RUN := FALSE;
	EM_B_EMIT := FALSE;
	EM_L_EMIT := FALSE;
	SOL_PRESS_DN := FALSE;
	SOL_PRESS_UP := TRUE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	TBL_RUN := FALSE;
	EM_B_EMIT := FALSE;
	EM_L_EMIT := FALSE;
	SOL_PRESS_DN := FALSE;
	SOL_PRESS_UP := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory for the press, cleared when INDIVIDUAL
// ends. The feed buttons pulse one part each; the index button is held and the cam turns while it is down.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_PRESS AND NOT IND_PRESS_LAST THEN
		MEM_PRESS := NOT MEM_PRESS;
	END_IF;
	EM_B_EMIT := IND_FEED_B;
	EM_L_EMIT := IND_FEED_L;
	SOL_PRESS_DN := MEM_PRESS;
	SOL_PRESS_UP := NOT MEM_PRESS;
	TBL_RUN := IND_INDEX;
ELSE
	MEM_PRESS := FALSE;
END_IF;
IND_PRESS_LAST := IND_PRESS;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
