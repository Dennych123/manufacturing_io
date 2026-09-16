VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	N : UDINT;
	T_SETTLE : TON;
	T_GAP : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
	MEM_CV : BOOL;
	IND_CV_LAST : BOOL;
	MEM_FEED : BOOL;
	IND_FEED_LAST : BOOL;
	MEM_STOP : BOOL;
	IND_STOP_LAST : BOOL;
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
// stopper-pusher: the belt runs, the loader feeds a part every 2.5 s, and a stopper pin holds
// each part at the pusher. Odd parts are pushed off the belt into the reject chute; even parts
// are released to the outfeed. Repeat until STOP. The same sequence runs without a PLC in
// stopper-pusher.ctl.js.
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
	0:	// idle
		CV1_RUN := FALSE;
		EM1_EN := FALSE;
		SOL_STOP := FALSE;
		SOL_PUSH := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_PUSH_RET THEN
			STOP_REQ := FALSE;
			ST1_STEP := 10;
		END_IF;
	10:	// stopper down, belt and loader on
		SOL_STOP := TRUE;
		CV1_RUN := TRUE;
		EM1_EN := TRUE;
		IF AS_STOP_DN THEN
			ST1_STEP := 20;
		END_IF;
	20:	// a part settles against the pin
		IF T_SETTLE.Q THEN
			ST1_STEP := 30;
		END_IF;
	30:	// odd parts to the reject chute, even parts to the outfeed
		N := N + 1;
		IF (N MOD 2) = 1 THEN
			ST1_STEP := 40;
		ELSE
			ST1_STEP := 60;
		END_IF;
	40:	// push
		SOL_PUSH := TRUE;
		IF AS_PUSH_EXT THEN
			ST1_STEP := 50;
		END_IF;
	50:	// pusher back, and the part is gone from the eye
		SOL_PUSH := FALSE;
		IF AS_PUSH_RET AND NOT PE_STOP THEN
			ST1_STEP := 80;
		END_IF;
	60:	// release: stopper up until the part has left the eye
		SOL_STOP := FALSE;
		IF NOT PE_STOP THEN
			ST1_STEP := 70;
		END_IF;
	70:	// let it clear the pin, then block again
		IF T_GAP.Q THEN
			SOL_STOP := TRUE;
			IF AS_STOP_DN THEN
				ST1_STEP := 80;
			END_IF;
		END_IF;
	80:	// cycle complete
		CYCLE_CNT := CYCLE_CNT + 1;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			ST1_STEP := 20;
		END_IF;
	800:	// HOME: pusher back, stopper up, belt and loader off
		CV1_RUN := FALSE;
		EM1_EN := FALSE;
		SOL_STOP := FALSE;
		SOL_PUSH := FALSE;
		IF AS_PUSH_RET AND AS_STOP_UP THEN
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds.
		CV1_RUN := FALSE;
		EM1_EN := FALSE;
		SOL_STOP := FALSE;
		SOL_PUSH := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving, or the selector was turned while running.
		CV1_RUN := FALSE;
		EM1_EN := FALSE;
		SOL_STOP := FALSE;
		SOL_PUSH := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_SETTLE(In := (ST1_STEP = 20) AND PE_STOP, PT := T#300ms);
T_GAP(In := (ST1_STEP = 70), PT := T#400ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	CV1_RUN := FALSE;
	EM1_EN := FALSE;
	SOL_STOP := FALSE;
	SOL_PUSH := FALSE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	CV1_RUN := FALSE;
	EM1_EN := FALSE;
	SOL_STOP := FALSE;
	SOL_PUSH := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory per actuator, cleared when INDIVIDUAL ends.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_CV AND NOT IND_CV_LAST THEN
		MEM_CV := NOT MEM_CV;
	END_IF;
	IF IND_FEED AND NOT IND_FEED_LAST THEN
		MEM_FEED := NOT MEM_FEED;
	END_IF;
	IF IND_STOP AND NOT IND_STOP_LAST THEN
		MEM_STOP := NOT MEM_STOP;
	END_IF;
	IF IND_PUSH AND NOT IND_PUSH_LAST THEN
		MEM_PUSH := NOT MEM_PUSH;
	END_IF;
	CV1_RUN := MEM_CV;
	EM1_EN := MEM_FEED;
	SOL_STOP := MEM_STOP;
	SOL_PUSH := MEM_PUSH;
ELSE
	MEM_CV := FALSE;
	MEM_FEED := FALSE;
	MEM_STOP := FALSE;
	MEM_PUSH := FALSE;
END_IF;
IND_CV_LAST := IND_CV;
IND_FEED_LAST := IND_FEED;
IND_STOP_LAST := IND_STOP;
IND_PUSH_LAST := IND_PUSH;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
