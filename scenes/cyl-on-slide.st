VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	T_DWELL : TON;
END_VAR
// cyl-on-slide: slide to the press position, press down, dwell, press up, slide home, repeat
// until STOP. The same sequence runs without a PLC in cyl-on-slide.ctl.js.
//
// The servo handshake is MC_MoveAbsolute's: Execute is a LEVEL held until Done, and Done must
// be seen to drop before the next Execute. The plant samples PLC outputs every ~16 ms, so a
// one-scan pulse would be missed.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

CASE ST1_STEP OF
	0:	// idle at home, cylinder up
		SOL_ST1_PRSS_CYL_DN := FALSE;
		SOL_ST1_PRSS_CYL_UP := TRUE;
		SV1_EXEC := FALSE;
		IF START_EDGE AND AS_ST1_PRSS_CYL_UP AND SV1_INPOS AND NOT SV1_DONE THEN
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
END_CASE;

T_DWELL(In := (ST1_STEP = 40), PT := T#300ms);
AUTO_RUN := ST1_STEP <> 0;
PL_START := AUTO_RUN;
