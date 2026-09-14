VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	EM_B_LAST : UDINT;
	EM_L_LAST : UDINT;
	NEXT_ST : INT;
	T_PRESS : TON;
	T_WD : TON;
	STEP_LAST : UDINT;
END_VAR
// assembler: a 4-station cam indexer. Every station works during the same dwell - load a base,
// drop a lid on it, press it - and then the table turns one step, carrying each assembly to the
// next station. The unload station simply takes whatever reaches it. The same sequence runs
// without a PLC in assembler.ctl.js.
//
// The index waits for an INVARIANT, TBL_STATION = NEXT_ST, not for the station to change: a
// change cannot say which index it belonged to, and OPC UA sampling can miss a short state.
// Each feeder command is held until that feeder's own counter answers it.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

CASE ST1_STEP OF
	0:	// idle, table stopped in a dwell, press up
		TBL_RUN := FALSE;
		EM_B_EMIT := FALSE;
		EM_L_EMIT := FALSE;
		SOL_PRESS_DN := FALSE;
		SOL_PRESS_UP := TRUE;
		IF START_EDGE AND TBL_INPOS AND AS_PRESS_UP THEN
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
	900:	// FAULT: a step stopped moving - a feeder could not drop, or the part under the press
		// was taken. Stop the table and the feeders and LIFT the press off the work.
		TBL_RUN := FALSE;
		EM_B_EMIT := FALSE;
		EM_L_EMIT := FALSE;
		SOL_PRESS_DN := FALSE;
		SOL_PRESS_UP := TRUE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_PRESS(In := (ST1_STEP = 40), PT := T#300ms);

// Watchdog: a step that stops moving is a jam, not patience. A feeder whose spot is blocked, or a
// part taken off the table, leaves this sequence holding a command that is never answered.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q THEN
	ST1_STEP := 900;
	TBL_RUN := FALSE;
	EM_B_EMIT := FALSE;
	EM_L_EMIT := FALSE;
	SOL_PRESS_DN := FALSE;
	SOL_PRESS_UP := TRUE;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 900);
PL_START := AUTO_RUN;
