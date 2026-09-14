VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	T_SETTLE : TON;
	T_GRIP : TON;
	T_WD : TON;
	EM_LAST : UDINT;
	STEP_LAST : UDINT;
	GONE : UDINT;
	AT_POSE : BOOL;
	CLEARED : BOOL;
END_VAR
// blurobot: the rb4axis BLUEROBOT cell. A rail along X carries a three-joint arm that turns about
// X in the Y-Z plane, each angle relative to the one before it. It takes a part out of the infeed
// nest and lays it on the outfeed belt, which carries it to the unloader. The same sequence runs
// without a PLC in blurobot.ctl.js.
//
// Every pose was MEASURED with the scene's own kinematics, not guessed:
//   pose      rail     j1      j2       j3     gripper tip
//   pick       -10   52.53  -70.73   -71.80   [-10, 420, 670]  20 mm under the part's centre
//   pickUp     -10   54.93  -47.00   -97.93   40 mm above it
//   place      850   54.89  -63.04   -81.85   [850, 420, 720]  just over the outfeed belt
//   placeUp    850   55.35  -59.20   -86.15   40 mm above it
//   home         0   90.00  -90.00   -90.00   folded, the rb4axis home
//
// The nest SNAPS what it catches onto its pocket floor, and that pocket is sunk 40 mm to keep it
// out of the belt's path, so the part rests at z 660 - not on the belt surface at 700. The pick
// pose aims 20 mm under the part's centre, which puts it in the middle of the grip zone.
//
// The outfeed sits at x 850, not 450: at 450 its slab reached back into the infeed's path and held
// the incoming part from 16.6 mm away, 175 mm short of the nest. Two belts must not overlap in X.
//
// The lift is 40 mm and no more. The envelope closes as the wrist rises toward the shoulder, and
// the sweep is unforgiving: +40 keeps 22.1 deg of margin on every joint at both stations, +60
// leaves 16.7, +80 leaves 10.5, and by +120 the pick side is OUT OF REACH altogether.
//
// Execute is a LEVEL held until every axis reports Done, and the next move waits for Done to fall
// again: a one-scan pulse falls between two OPC UA samples.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

AT_POSE := A0_DONE AND A1_DONE AND A2_DONE AND A3_DONE;
CLEARED := NOT A0_DONE AND NOT A1_DONE AND NOT A2_DONE AND NOT A3_DONE;

// A write-off goes STALE when what was written off turns up after all. GONE is a UDINT, so
// EM_CNT - RM_CNT with RM ahead underflows to about 4 billion: guard before you subtract.
IF RM_CNT >= EM_CNT THEN
	GONE := 0;
ELSIF RM_CNT + GONE > EM_CNT THEN
	GONE := EM_CNT - RM_CNT;
END_IF;

CASE ST1_STEP OF
	0:	// idle
		CV_IN_RUN := FALSE;
		CV_OUT_RUN := FALSE;
		EM_EMIT := FALSE;
		GRIP_CLOSE := FALSE;
		NEST_CLAMP := FALSE;
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			ST1_STEP := 10;
		END_IF;
	10:	// fold to home before anything else moves
		A0_TGT := 0.0; A1_TGT := 90.0; A2_TGT := -90.0; A3_TGT := -90.0;
		A0_EXEC := TRUE; A1_EXEC := TRUE; A2_EXEC := TRUE; A3_EXEC := TRUE;
		IF AT_POSE THEN
			ST1_STEP := 15;
		END_IF;
	15:	// the axes must see Execute drop before the next move
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN
			EM_LAST := EM_CNT;
			ST1_STEP := 20;
		END_IF;
	20:	// feed one part onto the infeed belt
		CV_IN_RUN := TRUE;
		EM_EMIT := TRUE;
		IF EM_CNT <> EM_LAST THEN
			EM_EMIT := FALSE;
			ST1_STEP := 30;
		END_IF;
	30:	// run it into the nest
		CV_IN_RUN := TRUE;
		NEST_CLAMP := TRUE;
		IF PE_IN THEN
			ST1_STEP := 35;
		END_IF;
	35:	// let it settle square in the pocket
		CV_IN_RUN := FALSE;
		IF T_SETTLE.Q THEN
			ST1_STEP := 40;
		END_IF;
	40:	// above the nest, gripper open
		GRIP_CLOSE := FALSE;
		A0_TGT := -10.0; A1_TGT := 54.93; A2_TGT := -47.0; A3_TGT := -97.93;
		A0_EXEC := TRUE; A1_EXEC := TRUE; A2_EXEC := TRUE; A3_EXEC := TRUE;
		IF AT_POSE THEN
			ST1_STEP := 45;
		END_IF;
	45:
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN
			ST1_STEP := 50;
		END_IF;
	50:	// down onto the part
		A0_TGT := -10.0; A1_TGT := 52.53; A2_TGT := -70.73; A3_TGT := -71.8;
		A0_EXEC := TRUE; A1_EXEC := TRUE; A2_EXEC := TRUE; A3_EXEC := TRUE;
		IF AT_POSE THEN
			ST1_STEP := 55;
		END_IF;
	55:
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN
			ST1_STEP := 60;
		END_IF;
	60:	// the nest lets go, the gripper takes it. One part is held by ONE holder.
		// NOT GRIP_CLOSED: that switch sits at FULL close, so with a part between the fingers it
		// never comes on - which is how a MISSED grip stays visible to the PLC (rb4axis
		// SIM_GRIP_TUTUP). A grip is confirmed by the fingers leaving the open switch and STAYING
		// off it, so T_GRIP holds the confirm and any flicker restarts it.
		NEST_CLAMP := FALSE;
		GRIP_CLOSE := TRUE;
		IF T_GRIP.Q THEN
			ST1_STEP := 70;
		END_IF;
	70:	// lift clear
		A0_TGT := -10.0; A1_TGT := 54.93; A2_TGT := -47.0; A3_TGT := -97.93;
		A0_EXEC := TRUE; A1_EXEC := TRUE; A2_EXEC := TRUE; A3_EXEC := TRUE;
		IF AT_POSE THEN
			ST1_STEP := 75;
		END_IF;
	75:
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN
			ST1_STEP := 80;
		END_IF;
	80:	// traverse to the outfeed
		A0_TGT := 850.0; A1_TGT := 55.35; A2_TGT := -59.2; A3_TGT := -86.15;
		A0_EXEC := TRUE; A1_EXEC := TRUE; A2_EXEC := TRUE; A3_EXEC := TRUE;
		IF AT_POSE THEN
			ST1_STEP := 85;
		END_IF;
	85:
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN
			ST1_STEP := 90;
		END_IF;
	90:	// down onto the belt
		A0_TGT := 850.0; A1_TGT := 54.89; A2_TGT := -63.04; A3_TGT := -81.85;
		A0_EXEC := TRUE; A1_EXEC := TRUE; A2_EXEC := TRUE; A3_EXEC := TRUE;
		IF AT_POSE THEN
			ST1_STEP := 95;
		END_IF;
	95:
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN
			ST1_STEP := 100;
		END_IF;
	100:	// let it go. GRIP_CLOSED is false all through a successful grip, so waiting for NOT
		// GRIP_CLOSED would pass instantly and lift away with the part still in the fingers.
		GRIP_CLOSE := FALSE;
		IF GRIP_OPEN THEN
			ST1_STEP := 110;
		END_IF;
	110:	// lift away before the belt runs
		A0_TGT := 850.0; A1_TGT := 55.35; A2_TGT := -59.2; A3_TGT := -86.15;
		A0_EXEC := TRUE; A1_EXEC := TRUE; A2_EXEC := TRUE; A3_EXEC := TRUE;
		IF AT_POSE THEN
			ST1_STEP := 115;
		END_IF;
	115:
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN
			ST1_STEP := 120;
		END_IF;
	120:	// discharge: everything fed must have reached the unloader
		CV_OUT_RUN := TRUE;
		IF RM_CNT + GONE >= EM_CNT THEN
			CV_OUT_RUN := FALSE;
			ST1_STEP := 130;
		END_IF;
	130:	// cycle complete. Re-snapshot the feeder count for the NEXT cycle: taking it only at
		// step 15, which the cycle never visits again, left EM_LAST at its pre-first-part value,
		// so step 20 saw "the count already moved", dropped EM_EMIT without commanding a part, and
		// step 30 waited for a part that was never fed until the watchdog faulted (measured).
		CYCLE_CNT := CYCLE_CNT + 1;
		EM_LAST := EM_CNT;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			ST1_STEP := 20;
		END_IF;
	900:	// FAULT: a step stopped moving. Stop the belts, the feeder and the axes, but KEEP the
		// grip: a real arm does not open its fingers over the floor.
		CV_IN_RUN := FALSE;
		CV_OUT_RUN := FALSE;
		EM_EMIT := FALSE;
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off what never reached the unloader. Only HERE: at every START
			// it would write off parts still legitimately in the cell.
			GONE := EM_CNT - RM_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_SETTLE(In := (ST1_STEP = 35), PT := T#400ms);
T_GRIP(In := (ST1_STEP = 60) AND GRIP_CLOSE AND NOT GRIP_OPEN, PT := T#300ms);

// Watchdog: a step that stops moving is a jam, not patience. Without it the sequence waits for
// ever and the machine only LOOKS alive.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q THEN
	ST1_STEP := 900;
	CV_IN_RUN := FALSE;
	CV_OUT_RUN := FALSE;
	EM_EMIT := FALSE;
	A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 900);
PL_START := AUTO_RUN;
