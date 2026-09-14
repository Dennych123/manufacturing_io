VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	T_DWELL : TON;
	EM_LAST : UDINT;
	RM_LAST : UDINT;
END_VAR
// a-to-b: load a part at the belt start, convey it to the end sensor, stop, dwell, then run
// the belt again until the unloader takes it. Repeat until STOP. The same sequence runs
// without a PLC in a-to-b.ctl.js.
//
// Commands are LEVELS held until the plant answers: EM1_EMIT stays on until the loader's
// counter EM1_CNT moves, so an OPC UA sample cannot miss it.
//
// A step that waits for a counter SNAPSHOTS it on entry. Measured on the simulator: a part
// removed while the sequence was still loading made step 40 see "the count moved" at once, so
// the cycle never waited for its own part and parts piled up (231 loaded, 217 unloaded).

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

CASE ST1_STEP OF
	0:	// idle, belt stopped
		CV1_RUN := FALSE;
		EM1_EMIT := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			EM_LAST := EM1_CNT;
			ST1_STEP := 10;
		END_IF;
	10:	// load one part
		EM1_EMIT := TRUE;
		IF EM1_CNT <> EM_LAST THEN
			EM1_EMIT := FALSE;
			ST1_STEP := 15;
		END_IF;
	15:	// the eye must be clear before this part runs to it
		CV1_RUN := TRUE;
		IF NOT PE_END THEN
			ST1_STEP := 20;
		END_IF;
	20:	// convey to the end sensor
		CV1_RUN := TRUE;
		IF PE_END THEN
			CV1_RUN := FALSE;
			ST1_STEP := 30;
		END_IF;
	30:	// dwell at the sensor
		IF T_DWELL.Q THEN
			RM_LAST := RM1_CNT;
			ST1_STEP := 40;
		END_IF;
	40:	// discharge to the unloader
		CV1_RUN := TRUE;
		IF RM1_CNT <> RM_LAST THEN
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
END_CASE;

T_DWELL(In := (ST1_STEP = 30), PT := T#500ms);
AUTO_RUN := ST1_STEP <> 0;
PL_START := AUTO_RUN;
