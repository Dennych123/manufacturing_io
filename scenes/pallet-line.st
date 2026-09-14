VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	T_DWELL : TON;
	T_WD : TON;
	EM_P_LAST : UDINT;
	EM_W_LAST : UDINT;
	STEP_LAST : UDINT;
	GONE : UDINT;
	FED : UDINT;
END_VAR
// pallet-line: a pallet is fed onto the belt, stopped at the station by a POP-UP stop, lifted off
// the belt and located square on its pins, loaded with a part, then put back down and released to
// the unloader. Repeat until STOP. The same sequence runs without a PLC in pallet-line.ctl.js.
//
// The stop pops UP from under the belt on purpose. An overhead pin that the pallet has to drive
// under holds it from 7.9-15.1 mm away even after it has lifted clear and come to rest, and the
// clearance sweep is non-monotonic (5.1 held, 8.1 free, 11.1 free, 14.1 HELD, 17.1+ free), so
// there is no clearance to design to. Retracted, a pop-up stop is below the belt surface and
// nothing stands in the path at all.
//
// The pin must also be UP before a pallet arrives: one that rises under a pallet already over it
// tips the pallet off the belt (measured).
//
// The unloader counts the pallet AND the part riding on it, so the discharge step waits on both
// feeders: RM_CNT + GONE >= EM_P_CNT + EM_W_CNT.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

FED := EM_P_CNT + EM_W_CNT;
// A write-off goes STALE when what was written off turns up after all. GONE is a UDINT, so
// FED - RM_CNT with RM ahead underflows to about 4 billion: guard before you subtract.
IF RM_CNT >= FED THEN
	GONE := 0;
ELSIF RM_CNT + GONE > FED THEN
	GONE := FED - RM_CNT;
END_IF;

CASE ST1_STEP OF
	0:	// idle
		CV1_RUN := FALSE;
		EM_P_EMIT := FALSE;
		EM_W_EMIT := FALSE;
		SOL_LIFT := FALSE;
		SOL_STOP := FALSE;
		IF START_EDGE AND AS_LIFT_DN THEN
			STOP_REQ := FALSE;
			ST1_STEP := 10;
		END_IF;
	10:	// stop UP first, before any pallet is on its way
		SOL_STOP := TRUE;
		IF AS_STOP_UP THEN
			EM_P_LAST := EM_P_CNT;
			ST1_STEP := 20;
		END_IF;
	20:	// feed one pallet
		CV1_RUN := TRUE;
		EM_P_EMIT := TRUE;
		IF EM_P_CNT <> EM_P_LAST THEN
			EM_P_EMIT := FALSE;
			ST1_STEP := 30;
		END_IF;
	30:	// convey it to the station
		CV1_RUN := TRUE;
		IF PE_STN THEN
			CV1_RUN := FALSE;
			ST1_STEP := 40;
		END_IF;
	40:	// lift and locate
		SOL_LIFT := TRUE;
		IF AS_LIFT_UP AND PLT_PRESENT THEN
			EM_W_LAST := EM_W_CNT;
			ST1_STEP := 50;
		END_IF;
	50:	// load one part onto the pallet
		EM_W_EMIT := TRUE;
		IF EM_W_CNT <> EM_W_LAST THEN
			EM_W_EMIT := FALSE;
			ST1_STEP := 60;
		END_IF;
	60:	// the station does its work
		IF T_DWELL.Q THEN
			ST1_STEP := 70;
		END_IF;
	70:	// pallet back onto the belt
		SOL_LIFT := FALSE;
		IF AS_LIFT_DN THEN
			ST1_STEP := 80;
		END_IF;
	80:	// only now is the pin allowed down, clear of the path
		SOL_STOP := FALSE;
		IF AS_STOP_DN THEN
			ST1_STEP := 90;
		END_IF;
	90:	// discharge: every pallet and part fed must have reached the unloader
		CV1_RUN := TRUE;
		IF RM_CNT + GONE >= FED THEN
			CV1_RUN := FALSE;
			ST1_STEP := 100;
		END_IF;
	100:	// cycle complete
		CYCLE_CNT := CYCLE_CNT + 1;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			ST1_STEP := 10;
		END_IF;
	900:	// FAULT: a step stopped moving. Stop feeding and the belt; the lift keeps its pallet.
		CV1_RUN := FALSE;
		EM_P_EMIT := FALSE;
		EM_W_EMIT := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off what never reached the unloader. Only HERE: at every START
			// it would write off pallets still legitimately on the belt.
			GONE := FED - RM_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_DWELL(In := (ST1_STEP = 60), PT := T#500ms);

// Watchdog: a step that stops moving is a jam, not patience. Without it the sequence waits for
// ever and the machine only LOOKS alive.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q THEN
	ST1_STEP := 900;
	CV1_RUN := FALSE;
	EM_P_EMIT := FALSE;
	EM_W_EMIT := FALSE;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 900);
PL_START := AUTO_RUN;
