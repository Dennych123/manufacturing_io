VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	T_WD : TON;
	EM_LAST : UDINT;
	STEP_LAST : UDINT;
	GONE : UDINT;
END_VAR
// pick-place: feed a part into the pick nest, pick it with the vacuum cup on the lift cylinder,
// traverse on the servo, lay it on the running outfeed belt, and come home while the belt takes
// it to the unloader. Repeat until STOP. The same sequence runs without a PLC in
// pick-place.ctl.js.
//
// Two handshakes matter. The servo's Execute is a LEVEL held until Done, and Done must be seen
// to drop before the next Execute. The nest clamp must open BEFORE the cup can take the part:
// one part is held by one holder.
//
// Waiting steps wait for an INVARIANT, not for a counter to move: a change cannot say which
// part moved (measured on a-to-b against the simulator: parts piled up, then pipelined).

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

CASE ST1_STEP OF
	0:	// idle
		CV_RUN := FALSE;
		EM_EMIT := FALSE;
		VAC_ON := FALSE;
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		SV_EXEC := FALSE;
		IF START_EDGE AND AS_Z_UP AND SV_INPOS AND NOT SV_DONE THEN
			STOP_REQ := FALSE;
			EM_LAST := EM_CNT;
			ST1_STEP := 10;
		END_IF;
	10:	// belt on, feed one part
		CV_RUN := TRUE;
		CLAMP_A := TRUE;
		EM_EMIT := TRUE;
		IF EM_CNT <> EM_LAST THEN
			EM_EMIT := FALSE;
			ST1_STEP := 20;
		END_IF;
	20:	// the nest reports the part
		IF NEST_A_P THEN
			ST1_STEP := 30;
		END_IF;
	30:	// lift down onto the part
		SOL_Z_UP := FALSE;
		SOL_Z_DN := TRUE;
		IF AS_Z_DN THEN
			ST1_STEP := 40;
		END_IF;
	40:	// nest lets go, vacuum takes over
		CLAMP_A := FALSE;
		VAC_ON := TRUE;
		IF VAC_SW THEN
			ST1_STEP := 50;
		END_IF;
	50:	// lift up with the part
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		IF AS_Z_UP THEN
			ST1_STEP := 60;
		END_IF;
	60:	// traverse to the belt
		SV_TGT := 600.0;
		SV_EXEC := TRUE;
		IF SV_DONE THEN
			SV_EXEC := FALSE;
			ST1_STEP := 70;
		END_IF;
	70:	// wait until the plant has seen Execute drop
		IF NOT SV_DONE THEN
			ST1_STEP := 80;
		END_IF;
	80:	// lift down over the belt
		SOL_Z_UP := FALSE;
		SOL_Z_DN := TRUE;
		IF AS_Z_DN THEN
			ST1_STEP := 90;
		END_IF;
	90:	// let the part go onto the belt
		VAC_ON := FALSE;
		IF NOT VAC_SW THEN
			ST1_STEP := 100;
		END_IF;
	100:	// lift up
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		IF AS_Z_UP THEN
			ST1_STEP := 110;
		END_IF;
	110:	// traverse home
		SV_TGT := 0.0;
		SV_EXEC := TRUE;
		IF SV_DONE THEN
			SV_EXEC := FALSE;
			ST1_STEP := 120;
		END_IF;
	120:	// wait until the plant has seen Execute drop
		IF NOT SV_DONE THEN
			ST1_STEP := 130;
		END_IF;
	130:	// the belt carries the part to the unloader: everything loaded must have left
		IF RM_CNT + GONE >= EM_CNT THEN
			CYCLE_CNT := CYCLE_CNT + 1;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSE
				EM_LAST := EM_CNT;
				ST1_STEP := 10;
			END_IF;
		END_IF;
	900:	// FAULT: a step stopped moving. Stop feeding and the servo, but KEEP vacuum: a real
		// machine does not drop the part it is holding.
		EM_EMIT := FALSE;
		CV_RUN := FALSE;
		SV_EXEC := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off whatever never reached the unloader. Only HERE: at every
			// START it would write off parts still legitimately in the machine, and then an older
			// part's removal ends this cycle - the pipelining bug that cost two live rounds.
			GONE := EM_CNT - RM_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

// Watchdog: a step that stops moving is a jam, not patience. Measured before this existed: taking
// the part out of the nest left ST1_STEP at 20 indefinitely with AUTO_RUN still on.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q THEN
	ST1_STEP := 900;
	EM_EMIT := FALSE;
	CV_RUN := FALSE;
	SV_EXEC := FALSE;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 900);
PL_START := AUTO_RUN;
