VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	TALL_TURN : BOOL;
	EM_T_LAST : UDINT;
	EM_S_LAST : UDINT;
	T_SETTLE : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
	GONE_T : UDINT;
	GONE_S : UDINT;
END_VAR
// sort-by-height: feed a tall part and a short part by turns, stop each one at the beams, and
// sort it by which beams are broken. The low beam sees any part; only a tall part reaches the
// high one. Tall parts are pushed off the belt into the reject chute, short ones ride on to the
// outfeed. The same sequence runs without a PLC in sort-by-height.ctl.js.
//
// Waiting steps wait for INVARIANTS - every tall part emitted has reached the bin, every short
// part the outfeed - and the conveying step first waits for the beam to clear, because a part
// still in it belongs to the last cycle.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

// A write-off goes STALE when the part turns up after all, and then RM + GONE runs PAST EM and
// the step waiting on that invariant stops waiting at all. GONE_T and GONE_S are UDINTs, so the
// subtraction underflows to about 4 billion when RM is ahead: guard before you subtract.
IF RM_T_CNT >= EM_T_CNT THEN
	GONE_T := 0;
ELSIF RM_T_CNT + GONE_T > EM_T_CNT THEN
	GONE_T := EM_T_CNT - RM_T_CNT;
END_IF;
IF RM_S_CNT >= EM_S_CNT THEN
	GONE_S := 0;
ELSIF RM_S_CNT + GONE_S > EM_S_CNT THEN
	GONE_S := EM_S_CNT - RM_S_CNT;
END_IF;

CASE ST1_STEP OF
	0:	// idle
		CV_RUN := FALSE;
		EM_T_EMIT := FALSE;
		EM_S_EMIT := FALSE;
		SOL_PUSH := FALSE;
		IF START_EDGE AND AS_PUSH_RET THEN
			STOP_REQ := FALSE;
			EM_T_LAST := EM_T_CNT;
			EM_S_LAST := EM_S_CNT;
			ST1_STEP := 10;
		END_IF;
	10:	// feed one part, tall and short by turns
		IF TALL_TURN THEN
			EM_T_EMIT := TRUE;
			IF EM_T_CNT <> EM_T_LAST THEN
				EM_T_EMIT := FALSE;
				ST1_STEP := 15;
			END_IF;
		ELSE
			EM_S_EMIT := TRUE;
			IF EM_S_CNT <> EM_S_LAST THEN
				EM_S_EMIT := FALSE;
				ST1_STEP := 15;
			END_IF;
		END_IF;
	15:	// the beam must be clear before this part runs to it
		CV_RUN := TRUE;
		IF NOT PE_LOW THEN
			ST1_STEP := 20;
		END_IF;
	20:	// carry it to the beams
		CV_RUN := TRUE;
		IF PE_LOW THEN
			CV_RUN := FALSE;
			ST1_STEP := 30;
		END_IF;
	30:	// let it settle in front of the pusher
		IF T_SETTLE.Q THEN
			ST1_STEP := 40;
		END_IF;
	40:	// the high beam tells tall from short
		IF PE_HIGH THEN
			ST1_STEP := 50;
		ELSE
			ST1_STEP := 60;
		END_IF;
	50:	// push the tall part off the belt
		SOL_PUSH := TRUE;
		IF AS_PUSH_EXT THEN
			ST1_STEP := 55;
		END_IF;
	55:	// pusher back, and the part has reached the bin
		SOL_PUSH := FALSE;
		IF AS_PUSH_RET AND (RM_T_CNT + GONE_T >= EM_T_CNT) THEN
			ST1_STEP := 70;
		END_IF;
	60:	// carry the short part to the outfeed
		CV_RUN := TRUE;
		IF RM_S_CNT + GONE_S >= EM_S_CNT THEN
			CV_RUN := FALSE;
			ST1_STEP := 70;
		END_IF;
	70:	// cycle complete, and the other kind next time
		CYCLE_CNT := CYCLE_CNT + 1;
		TALL_TURN := NOT TALL_TURN;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			EM_T_LAST := EM_T_CNT;
			EM_S_LAST := EM_S_CNT;
			ST1_STEP := 10;
		END_IF;
	900:	// FAULT: a step stopped moving, so one of the two invariants above can never hold
		// again. Acknowledging writes off what never arrived - only HERE, never at every
		// START, or a straggler's removal ends the next cycle.
		CV_RUN := FALSE;
		EM_T_EMIT := FALSE;
		EM_S_EMIT := FALSE;
		SOL_PUSH := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			GONE_T := EM_T_CNT - RM_T_CNT;
			GONE_S := EM_S_CNT - RM_S_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_SETTLE(In := (ST1_STEP = 30), PT := T#300ms);

// Watchdog: a step that stops moving is a jam, not patience. A part taken off the belt leaves
// this sequence waiting for a beam, or for an invariant that can no longer hold.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q THEN
	ST1_STEP := 900;
	CV_RUN := FALSE;
	EM_T_EMIT := FALSE;
	EM_S_EMIT := FALSE;
	SOL_PUSH := FALSE;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 900);
PL_START := AUTO_RUN;
