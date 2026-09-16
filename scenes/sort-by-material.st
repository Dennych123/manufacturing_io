VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	METAL_TURN : BOOL := TRUE;
	METAL : BOOL;
	EM_M_LAST : UDINT;
	EM_P_LAST : UDINT;
	T_SETTLE : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
	GONE_M : UDINT;
	GONE_P : UDINT;
	MEM_CV : BOOL;
	IND_CV_LAST : BOOL;
	MEM_PUSH : BOOL;
	IND_PUSH_LAST : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// sort-by-material: feed a steel part and a plastic part by turns. Both look the same to the stop
// beam; only the inductive proximity sensor upstream sees the steel one, and its pulse is LATCHED
// while the belt carries the part to the pusher. Steel is pushed off into the bin, plastic rides
// on to the outfeed. The same sequence runs without a PLC in sort-by-material.ctl.js.
//
// Waiting steps wait for INVARIANTS - every steel part fed has reached the bin, every plastic part
// the outfeed - and the conveying step first waits for the beam to clear, because a part still in
// it belongs to the last cycle.
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
IF RM_M_CNT >= EM_M_CNT THEN
	GONE_M := 0;
ELSIF RM_M_CNT + GONE_M > EM_M_CNT THEN
	GONE_M := EM_M_CNT - RM_M_CNT;
END_IF;
IF RM_P_CNT >= EM_P_CNT THEN
	GONE_P := 0;
ELSIF RM_P_CNT + GONE_P > EM_P_CNT THEN
	GONE_P := EM_P_CNT - RM_P_CNT;
END_IF;
CASE ST1_STEP OF
	0:	// idle
		CV_RUN := FALSE;
		EM_M_EMIT := FALSE;
		EM_P_EMIT := FALSE;
		SOL_PUSH := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_PUSH_RET THEN
			STOP_REQ := FALSE;
			EM_M_LAST := EM_M_CNT;
			EM_P_LAST := EM_P_CNT;
			ST1_STEP := 10;
		END_IF;
	10:	// feed one part, steel and plastic by turns; the metal latch starts clean
		METAL := FALSE;
		IF METAL_TURN THEN
			EM_M_EMIT := TRUE;
			IF EM_M_CNT <> EM_M_LAST THEN
				EM_M_EMIT := FALSE;
				ST1_STEP := 15;
			END_IF;
		ELSE
			EM_P_EMIT := TRUE;
			IF EM_P_CNT <> EM_P_LAST THEN
				EM_P_EMIT := FALSE;
				ST1_STEP := 15;
			END_IF;
		END_IF;
	15:	// the beam must be clear before this part runs to it
		CV_RUN := TRUE;
		IF NOT PE_STOP THEN
			ST1_STEP := 20;
		END_IF;
	20:	// carry it past the proximity sensor to the stop beam
		CV_RUN := TRUE;
		IF PE_STOP THEN
			CV_RUN := FALSE;
			ST1_STEP := 30;
		END_IF;
	30:	// let it settle in front of the pusher
		IF T_SETTLE.Q THEN
			ST1_STEP := 40;
		END_IF;
	40:	// the latch says what passed the inductive sensor
		IF METAL THEN
			ST1_STEP := 50;
		ELSE
			ST1_STEP := 60;
		END_IF;
	50:	// push the steel part off the belt
		SOL_PUSH := TRUE;
		IF AS_PUSH_EXT THEN
			ST1_STEP := 55;
		END_IF;
	55:	// pusher back, and every steel part fed has reached the bin
		SOL_PUSH := FALSE;
		IF AS_PUSH_RET AND (RM_M_CNT + GONE_M >= EM_M_CNT) THEN
			ST1_STEP := 70;
		END_IF;
	60:	// carry the plastic part to the outfeed
		CV_RUN := TRUE;
		IF RM_P_CNT + GONE_P >= EM_P_CNT THEN
			CV_RUN := FALSE;
			ST1_STEP := 70;
		END_IF;
	70:	// cycle complete, and the other material next time
		CYCLE_CNT := CYCLE_CNT + 1;
		METAL_TURN := NOT METAL_TURN;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			EM_M_LAST := EM_M_CNT;
			EM_P_LAST := EM_P_CNT;
			ST1_STEP := 10;
		END_IF;
	800:	// HOME: pusher back, belt and feeders off
		CV_RUN := FALSE;
		EM_M_EMIT := FALSE;
		EM_P_EMIT := FALSE;
		SOL_PUSH := FALSE;
		IF AS_PUSH_RET THEN
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds.
		CV_RUN := FALSE;
		EM_M_EMIT := FALSE;
		EM_P_EMIT := FALSE;
		SOL_PUSH := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving (a part taken, jammed, lost or sorted the wrong way), or
		// the selector was turned while running.
		CV_RUN := FALSE;
		EM_M_EMIT := FALSE;
		EM_P_EMIT := FALSE;
		SOL_PUSH := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off whatever never reached its bin. Only HERE.
			GONE_M := EM_M_CNT - RM_M_CNT;
			GONE_P := EM_P_CNT - RM_P_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

// The metal latch: the part passes the inductive sensor on its way to the stop beam, so the
// sensor is read while the belt carries it, not when it stands at the pusher.
IF ((ST1_STEP = 15) OR (ST1_STEP = 20)) AND PX_METAL THEN
	METAL := TRUE;
END_IF;

T_SETTLE(In := (ST1_STEP = 30), PT := T#300ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	CV_RUN := FALSE;
	EM_M_EMIT := FALSE;
	EM_P_EMIT := FALSE;
	SOL_PUSH := FALSE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	CV_RUN := FALSE;
	EM_M_EMIT := FALSE;
	EM_P_EMIT := FALSE;
	SOL_PUSH := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory per actuator, cleared when INDIVIDUAL
// ends. The feed buttons pulse one part each.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_CV AND NOT IND_CV_LAST THEN
		MEM_CV := NOT MEM_CV;
	END_IF;
	IF IND_PUSH AND NOT IND_PUSH_LAST THEN
		MEM_PUSH := NOT MEM_PUSH;
	END_IF;
	CV_RUN := MEM_CV;
	EM_M_EMIT := IND_FEED_M;
	EM_P_EMIT := IND_FEED_P;
	SOL_PUSH := MEM_PUSH;
ELSE
	MEM_CV := FALSE;
	MEM_PUSH := FALSE;
END_IF;
IND_CV_LAST := IND_CV;
IND_PUSH_LAST := IND_PUSH;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
