VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	N : UDINT;
	T_SETTLE : TON;
	T_GAP : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
END_VAR
// stopper-pusher: the belt runs, the loader feeds a part every 2.5 s, and a stopper pin holds
// each part at the pusher. Odd parts are pushed off the belt into the reject chute; even parts
// are released to the outfeed. Repeat until STOP. The same sequence runs without a PLC in
// stopper-pusher.ctl.js.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

CASE ST1_STEP OF
	0:	// idle
		CV1_RUN := FALSE;
		EM1_EN := FALSE;
		SOL_STOP := FALSE;
		SOL_PUSH := FALSE;
		IF START_EDGE AND AS_PUSH_RET THEN
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
	900:	// FAULT: a step stopped moving. Stop feeding and the pusher, but LEAVE the stopper
		// down - it is holding the queue back.
		CV1_RUN := FALSE;
		EM1_EN := FALSE;
		SOL_PUSH := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_SETTLE(In := (ST1_STEP = 20) AND PE_STOP, PT := T#300ms);
T_GAP(In := (ST1_STEP = 70), PT := T#400ms);

// Watchdog: a step that stops moving is a jam, not patience. A part taken off the belt by hand
// leaves the settle step waiting for a pin it will never reach.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q THEN
	ST1_STEP := 900;
	CV1_RUN := FALSE;
	EM1_EN := FALSE;
	SOL_PUSH := FALSE;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 900);
PL_START := AUTO_RUN;
