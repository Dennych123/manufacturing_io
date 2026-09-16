VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	T_DWELL : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
	MEM_CYL : BOOL;
	IND_CYL_LAST : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// cyl-on-slide: slide to the press position, press down, dwell, press up, slide home, repeat
// until STOP. The same sequence runs without a PLC in cyl-on-slide.ctl.js.
//
// The servo handshake is MC_MoveAbsolute's: Execute is a LEVEL held until Done, and Done must
// be seen to drop before the next Execute. The plant samples PLC outputs every ~16 ms, so a
// one-scan pulse would be missed.
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
	0:	// idle at home, cylinder up
		SOL_ST1_PRSS_CYL_DN := FALSE;
		SOL_ST1_PRSS_CYL_UP := TRUE;
		SV1_EXEC := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_ST1_PRSS_CYL_UP AND SV1_INPOS AND NOT SV1_DONE THEN
			STOP_REQ := FALSE;
			ST1_STEP := 10;
		END_IF;
	10:	// slide to the press position
		SV1_TGT := 400.0;
		SV1_EXEC := TRUE;
		IF SV1_DONE THEN
			SV1_EXEC := FALSE;
			ST1_STEP := 20;
		END_IF;
	20:	// wait until the plant has seen Execute drop
		IF NOT SV1_DONE THEN
			ST1_STEP := 30;
		END_IF;
	30:	// press down
		SOL_ST1_PRSS_CYL_UP := FALSE;
		SOL_ST1_PRSS_CYL_DN := TRUE;
		IF AS_ST1_PRSS_CYL_DN THEN
			ST1_STEP := 40;
		END_IF;
	40:	// dwell on the part
		IF T_DWELL.Q THEN
			ST1_STEP := 50;
		END_IF;
	50:	// press up
		SOL_ST1_PRSS_CYL_DN := FALSE;
		SOL_ST1_PRSS_CYL_UP := TRUE;
		IF AS_ST1_PRSS_CYL_UP THEN
			ST1_STEP := 60;
		END_IF;
	60:	// slide home
		SV1_TGT := 0.0;
		SV1_EXEC := TRUE;
		IF SV1_DONE THEN
			SV1_EXEC := FALSE;
			ST1_STEP := 70;
		END_IF;
	70:	// cycle complete
		IF NOT SV1_DONE THEN
			CYCLE_CNT := CYCLE_CNT + 1;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSE
				ST1_STEP := 10;
			END_IF;
		END_IF;
	800:	// HOME: press cylinder up, slide back to zero
		SOL_ST1_PRSS_CYL_DN := FALSE;
		SOL_ST1_PRSS_CYL_UP := TRUE;
		SV1_TGT := 0.0;
		SV1_EXEC := TRUE;
		IF AS_ST1_PRSS_CYL_UP AND SV1_DONE THEN
			SV1_EXEC := FALSE;
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds.
		SV1_EXEC := FALSE;
		SOL_ST1_PRSS_CYL_DN := FALSE;
		SOL_ST1_PRSS_CYL_UP := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving, or the selector was turned while running. Servo
		// stopped, cylinder up, wait for START in AUTO.
		SV1_EXEC := FALSE;
		SOL_ST1_PRSS_CYL_DN := FALSE;
		SOL_ST1_PRSS_CYL_UP := TRUE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_DWELL(In := (ST1_STEP = 40), PT := T#300ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	SV1_EXEC := FALSE;
	SOL_ST1_PRSS_CYL_DN := FALSE;
	SOL_ST1_PRSS_CYL_UP := TRUE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	SV1_EXEC := FALSE;
	SOL_ST1_PRSS_CYL_DN := FALSE;
	SOL_ST1_PRSS_CYL_UP := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory per actuator, cleared when INDIVIDUAL
// ends. The servo buttons are held: Execute is a level, and the axis finishes the move on its own.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_CYL AND NOT IND_CYL_LAST THEN
		MEM_CYL := NOT MEM_CYL;
	END_IF;
	SOL_ST1_PRSS_CYL_DN := MEM_CYL;
	SOL_ST1_PRSS_CYL_UP := NOT MEM_CYL;
ELSE
	MEM_CYL := FALSE;
END_IF;
IND_CYL_LAST := IND_CYL;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

// Jog: the axis creeps while the button is held, and only on INDIVIDUAL.
SV1_JOG_P := NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) AND IND_SV1_P;
SV1_JOG_N := NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) AND IND_SV1_N;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
