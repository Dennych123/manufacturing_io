VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	T_DWELL : TON;
	T_WD : TON;
	EM_LAST : UDINT;
	STEP_LAST : UDINT;
	GONE : UDINT;
	MEM_CV : BOOL;
	IND_CV_LAST : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// a-to-b: load one part, carry it to the end sensor, dwell, discharge it to the unloader, repeat
// until STOP. The same sequence runs without a PLC in a-to-b.ctl.js.
//
// Waiting steps wait for an INVARIANT, not for a counter to move: a change cannot say which
// part moved (measured on the simulator: parts piled up, then pipelined).
//
// The panel has a selector AUTO / INDIVIDUAL (rb4axis): START only in AUTO, the individual buttons
// only in INDIVIDUAL, and a change while the sequence runs is a FAULT, not a pause.

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
	0:	// idle
		CV1_RUN := FALSE;
		EM1_EMIT := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED THEN
			STOP_REQ := FALSE;
			EM_LAST := EM1_CNT;
			ST1_STEP := 10;
		END_IF;
	10:	// load one part: the command is held until the loader's counter answers it
		EM1_EMIT := TRUE;
		IF EM1_CNT <> EM_LAST THEN
			EM1_EMIT := FALSE;
			ST1_STEP := 15;
		END_IF;
	15:	// the beam must be clear before this part runs to it
		CV1_RUN := TRUE;
		IF NOT PE_END THEN
			ST1_STEP := 20;
		END_IF;
	20:	// carry it to the end sensor
		CV1_RUN := TRUE;
		IF PE_END THEN
			CV1_RUN := FALSE;
			ST1_STEP := 30;
		END_IF;
	30:	// dwell
		IF T_DWELL.Q THEN
			ST1_STEP := 40;
		END_IF;
	40:	// discharge: every part loaded has left the belt (counting the write-offs)
		CV1_RUN := TRUE;
		IF RM1_CNT + GONE >= EM1_CNT THEN
			CV1_RUN := FALSE;
			ST1_STEP := 50;
		END_IF;
	50:	// cycle complete
		CYCLE_CNT := CYCLE_CNT + 1;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			EM_LAST := EM1_CNT;
			ST1_STEP := 10;
		END_IF;
	800:	// HOME: nothing here is positioned, so home only has to leave the line quiet
		CV1_RUN := FALSE;
		EM1_EMIT := FALSE;
		HOMED := TRUE;
		ST1_STEP := 0;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds.
		CV1_RUN := FALSE;
		EM1_EMIT := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving. Hold everything until START acknowledges it.
		CV1_RUN := FALSE;
		EM1_EMIT := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off whatever never reached the unloader. Only HERE: at every
			// START it would write off parts still legitimately on the belt, and then an older
			// part's removal ends this cycle - the pipelining bug that cost two live rounds.
			GONE := EM1_CNT - RM1_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_DWELL(In := (ST1_STEP = 30), PT := T#500ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	CV1_RUN := FALSE;
	EM1_EMIT := FALSE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	CV1_RUN := FALSE;
	EM1_EMIT := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: the buttons are momentary; the PLC keeps a toggle
// memory per actuator and clears it when INDIVIDUAL ends, so nothing stays latched into AUTO.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_CV AND NOT IND_CV_LAST THEN
		MEM_CV := NOT MEM_CV;
	END_IF;
	CV1_RUN := MEM_CV;
	EM1_EMIT := IND_FEED;
ELSE
	MEM_CV := FALSE;
END_IF;
IND_CV_LAST := IND_CV;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
