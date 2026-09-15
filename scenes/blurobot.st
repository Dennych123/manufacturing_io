VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	T_WD : TON;
	T_GRIP : TON;
	STEP_LAST : UDINT;
	i : INT;
	j : INT;
	SRC : INT;
	DST : INT;
	IDX : INT;
	HOLDING : BOOL;
	AT_POSE : BOOL;
	CLEARED : BOOL;
	RAIL_X : LREAL;
	ARM_AT_1 : LREAL;
	ARM_AT_2 : LREAL;
	ARM_AT_3 : LREAL;
	ARM_UP_1 : LREAL;
	ARM_UP_2 : LREAL;
	ARM_UP_3 : LREAL;
	COVER_OK : BOOL;
	IN_ZONE : BOOL;
	SHUT : BOOL;
	// What the PLC BELIEVES about each station, and each machine's own clock. These are program
	// variables, not machine IO: nothing in the plant senses them. rb4axis keeps the same values in
	// SIM_ST_STATE / SIM_ST_TIMER and only publishes them so its viewer can draw the cell.
	//   0 empty, 1 processing, 2 finished and waiting to be collected
	ICC1_STATE : INT;
	ICC2_STATE : INT;
	DW1_STATE : INT;
	DW2_STATE : INT;
	ICC1_TIMER : LREAL;
	ICC2_TIMER : LREAL;
	DW1_TIMER : LREAL;
	DW2_TIMER : LREAL;
	// The cover target each station is latched on. Execute is LATCHED on its rising edge, like
	// every other axis here, so holding it true from the first scan means the target is never
	// re-read: measured, the covers held their startup 80 while the target sat at 0, no ICC clock
	// ever ran, and no board ever finished. A change re-pulses Execute.
	ICC1_CV_WANT : LREAL;
	ICC2_CV_WANT : LREAL;
	DW1_CV_WANT : LREAL;
	DW2_CV_WANT : LREAL;
END_VAR
// blurobot - a PORT of rb4axis PRG_SIM_ROBOT.st. The cell, the station table, the job priorities,
// the step numbers and the timings are that program's; only the tag shape differs, because this
// repo binds scalar tags while the original keeps SIM_ST_* arrays.
//
//   station   type  rail x   surface z   process
//   WIP IN     0     -1400      250        -      stock is not simulated: always FULL
//   ICC 1      1      -700      285      17.0 s   test; the cover presses the PCB onto the probes
//   ICC 2      1      -300      285      17.0 s
//   DW 1       2       300      250      15.0 s   data writer
//   DW 2       2       700      250      15.0 s
//   WIP OUT    3      1400      250        -      the product leaves: always EMPTY
//
// Job priority, and the ORDER is the behaviour:
//   1. fill an EMPTY ICC from WIP IN - both - before anything else. That is what buffer means: the
//      17 s tester must not idle while the robot runs errands for DW.
//   2. empty a FINISHED DW to WIP OUT, ahead of 3, so a free DW always exists. Without it two full
//      DWs plus two finished ICCs deadlock, and the deadlock looks like a robot that stopped.
//   3. move a FINISHED ICC to an EMPTY DW.
//
// Poses were measured against the scene's own kinematics with the carriage at z 200: all twelve
// (six surfaces + six 140 mm approaches) solve with 67.3 deg of worst-case joint margin. Only two
// station heights exist, so there are four arm poses and the rail carries the difference.
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

// ---------------------------------------------------------------- the machines
// Each keeps its OWN timer, running together: that is the whole point of a buffer, ICC 1 and ICC 2
// must be able to test at the same time while the robot is away. The clock runs ONLY with the
// cover shut - the cover presses the PCB onto the probes, and counting before it closes means
// claiming a test on a board nothing has touched.
//
// The cover closes only while processing and REOPENS if the arm is in its sweep: a light curtain,
// not a delay. Without it the leaf swings onto an arm that is still lifting out.
IN_ZONE := (ABS(A0_POS - (-700.0)) < 200.0) AND (A1_POS < 60.0);
SHUT := (ICC1_STATE = 1) AND NOT IN_ZONE;
IF SHUT THEN COVER_TARGET := 0.0; ELSE COVER_TARGET := 80.0; END_IF;
IF COVER_TARGET <> ICC1_CV_WANT THEN
	ICC1_CV_WANT := COVER_TARGET;
	ICC1_CV_EXEC := FALSE;
ELSE
	ICC1_CV_TGT := COVER_TARGET;
	ICC1_CV_EXEC := TRUE;
END_IF;
IF (ICC1_STATE = 1) AND (ICC1_CV_POS <= 0.5) THEN
	ICC1_TIMER := ICC1_TIMER - 0.002;
	IF ICC1_TIMER <= 0.0 THEN ICC1_TIMER := 0.0; ICC1_STATE := 2; END_IF;
END_IF;

IN_ZONE := (ABS(A0_POS - (-300.0)) < 200.0) AND (A1_POS < 60.0);
SHUT := (ICC2_STATE = 1) AND NOT IN_ZONE;
IF SHUT THEN COVER_TARGET := 0.0; ELSE COVER_TARGET := 80.0; END_IF;
IF COVER_TARGET <> ICC2_CV_WANT THEN
	ICC2_CV_WANT := COVER_TARGET;
	ICC2_CV_EXEC := FALSE;
ELSE
	ICC2_CV_TGT := COVER_TARGET;
	ICC2_CV_EXEC := TRUE;
END_IF;
IF (ICC2_STATE = 1) AND (ICC2_CV_POS <= 0.5) THEN
	ICC2_TIMER := ICC2_TIMER - 0.002;
	IF ICC2_TIMER <= 0.0 THEN ICC2_TIMER := 0.0; ICC2_STATE := 2; END_IF;
END_IF;

IN_ZONE := (ABS(A0_POS - 300.0) < 200.0) AND (A1_POS < 60.0);
SHUT := (DW1_STATE = 1) AND NOT IN_ZONE;
IF SHUT THEN COVER_TARGET := 0.0; ELSE COVER_TARGET := 80.0; END_IF;
IF COVER_TARGET <> DW1_CV_WANT THEN
	DW1_CV_WANT := COVER_TARGET;
	DW1_CV_EXEC := FALSE;
ELSE
	DW1_CV_TGT := COVER_TARGET;
	DW1_CV_EXEC := TRUE;
END_IF;
IF (DW1_STATE = 1) AND (DW1_CV_POS <= 0.5) THEN
	DW1_TIMER := DW1_TIMER - 0.002;
	IF DW1_TIMER <= 0.0 THEN DW1_TIMER := 0.0; DW1_STATE := 2; END_IF;
END_IF;

IN_ZONE := (ABS(A0_POS - 700.0) < 200.0) AND (A1_POS < 60.0);
SHUT := (DW2_STATE = 1) AND NOT IN_ZONE;
IF SHUT THEN COVER_TARGET := 0.0; ELSE COVER_TARGET := 80.0; END_IF;
IF COVER_TARGET <> DW2_CV_WANT THEN
	DW2_CV_WANT := COVER_TARGET;
	DW2_CV_EXEC := FALSE;
ELSE
	DW2_CV_TGT := COVER_TARGET;
	DW2_CV_EXEC := TRUE;
END_IF;
IF (DW2_STATE = 1) AND (DW2_CV_POS <= 0.5) THEN
	DW2_TIMER := DW2_TIMER - 0.002;
	IF DW2_TIMER <= 0.0 THEN DW2_TIMER := 0.0; DW2_STATE := 2; END_IF;
END_IF;

// WIP IN never runs out and WIP OUT never fills up: the stock either side of the cell is not
// simulated, and pretending otherwise would starve or block a cell that never does either. Those
// two need no state variable at all - the job selection below simply treats them that way.

// ---------------------------------------------------------- the pose for IDX
// Four arm poses only, because the six stations sit at just two heights; the rail carries the
// rest. Measured against the scene's own kinematics with the carriage at z 200.
CASE IDX OF
	0:	RAIL_X := -1400.0;
	1:	RAIL_X := -700.0;
	2:	RAIL_X := -300.0;
	3:	RAIL_X := 300.0;
	4:	RAIL_X := 700.0;
	5:	RAIL_X := 1400.0;
ELSE
	RAIL_X := 0.0;
END_CASE;

IF (IDX = 1) OR (IDX = 2) THEN
	// the 285 mm tables
	ARM_AT_1 := 15.72;   ARM_AT_2 := -71.33;   ARM_AT_3 := -34.39;
	ARM_UP_1 := 36.55;   ARM_UP_2 := -77.44;   ARM_UP_3 := -49.11;
ELSE
	// the 250 mm tables
	ARM_AT_1 := 9.76;   ARM_AT_2 := -67.25;   ARM_AT_3 := -32.5;
	ARM_UP_1 := 31.86;   ARM_UP_2 := -77.37;   ARM_UP_3 := -44.49;
END_IF;

// The WIP racks are open - no cover, so nothing to wait for.
CASE IDX OF
	1:	COVER_OK := ICC1_CV_POS >= 79.5;
	2:	COVER_OK := ICC2_CV_POS >= 79.5;
	3:	COVER_OK := DW1_CV_POS >= 79.5;
	4:	COVER_OK := DW2_CV_POS >= 79.5;
ELSE
	COVER_OK := TRUE;
END_CASE;

// ------------------------------------------------- every station holds what is on it
// Only the one being picked from lets go, and only while the gripper takes the board. Measured
// with no clamp set at all: the nest held nothing, WIPIN_P never came true, and the fingers closed
// to zero straight through the PCB - a part's width is only measured for a FREE part, so gripping
// a still-held board reads as gripping air and the closed switch asserts for the wrong reason.
// WIP IN is an ENDLESS stock: the PLC pins its state to "full" because the racks either side of
// the cell are not simulated, so the plant has to keep a board there or the cell runs dry and the
// next grip closes on air. A LEVEL held until the feeder's own count answers it, never a pulse,
// and only while that nest is empty and the arm is not taking from it.
EM_IN_EMIT := NOT WIPIN_P AND NOT (((ST1_STEP = 14) OR (ST1_STEP = 15)) AND (IDX = 0));

WIPIN_CLAMP := NOT (((ST1_STEP = 14) OR (ST1_STEP = 15)) AND (IDX = 0));
ICC1_CLAMP := NOT (((ST1_STEP = 14) OR (ST1_STEP = 15)) AND (IDX = 1));
ICC2_CLAMP := NOT (((ST1_STEP = 14) OR (ST1_STEP = 15)) AND (IDX = 2));
DW1_CLAMP := NOT (((ST1_STEP = 14) OR (ST1_STEP = 15)) AND (IDX = 3));
DW2_CLAMP := NOT (((ST1_STEP = 14) OR (ST1_STEP = 15)) AND (IDX = 4));
WIPOUT_CLAMP := NOT (((ST1_STEP = 14) OR (ST1_STEP = 15)) AND (IDX = 5));

CASE ST1_STEP OF
	0:	// pick a job - the order of these three searches IS the cell's behaviour
		SRC := -1;
		DST := -1;
		IF (ICC1_STATE = 0) THEN SRC := 0; DST := 1; END_IF;
		IF (DST < 0) AND (ICC2_STATE = 0) THEN SRC := 0; DST := 2; END_IF;
		IF (DST < 0) AND (DW1_STATE = 2) THEN SRC := 3; DST := 5; END_IF;
		IF (DST < 0) AND (DW2_STATE = 2) THEN SRC := 4; DST := 5; END_IF;
		IF (DST < 0) AND (ICC1_STATE = 2) AND (DW1_STATE = 0) THEN SRC := 1; DST := 3; END_IF;
		IF (DST < 0) AND (ICC1_STATE = 2) AND (DW2_STATE = 0) THEN SRC := 1; DST := 4; END_IF;
		IF (DST < 0) AND (ICC2_STATE = 2) AND (DW1_STATE = 0) THEN SRC := 2; DST := 3; END_IF;
		IF (DST < 0) AND (ICC2_STATE = 2) AND (DW2_STATE = 0) THEN SRC := 2; DST := 4; END_IF;
		IF DST >= 0 THEN
			IDX := SRC;
			ST1_STEP := 5;
		ELSIF STOP_REQ THEN
			STOP_REQ := FALSE;
		END_IF;

	5:	// fold to the travel pose - axes 1-3 only, the rail is not touched
		A1_TGT := 90.0;   A2_TGT := -90.0;   A3_TGT := -90.0;
		A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF A1_DONE AND A2_DONE AND A3_DONE THEN
			ST1_STEP := 7;
		END_IF;

	7:	// slide the rail. The arm MUST be folded: travelling while reaching down sweeps every
		// machine it passes, and on screen that looks perfectly smooth.
		A0_TGT := RAIL_X;
		A0_EXEC := TRUE;
		ST1_STEP := 8;

	8:	// TWO things, and the wait is HERE rather than after the approach is asked for: asking
		// while the cover is still shut gets the move refused, nothing moves, and Done stays TRUE -
		// so the next step would drive straight down from the travel pose, skipping the approach.
		IF A0_DONE AND COVER_OK THEN
			ST1_STEP := 10;
		END_IF;

	10:	// above the station
		A0_TGT := RAIL_X;   A1_TGT := ARM_UP_1;   A2_TGT := ARM_UP_2;   A3_TGT := ARM_UP_3;
		A0_EXEC := TRUE;   A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF AT_POSE THEN ST1_STEP := 11; END_IF;
	11:	A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN ST1_STEP := 12; END_IF;

	12:	// straight down onto the surface - the gripper pinches from ABOVE
		A0_TGT := RAIL_X;   A1_TGT := ARM_AT_1;   A2_TGT := ARM_AT_2;   A3_TGT := ARM_AT_3;
		A0_EXEC := TRUE;   A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF AT_POSE THEN ST1_STEP := 13; END_IF;
	13:	A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN ST1_STEP := 14; END_IF;

	14:	// the station let go above; close on the board
		GRIP_CLOSE := TRUE;
		ST1_STEP := 15;

	15:	// NOT GRIP_CLOSED: that switch sits at FULL close, and a good grip STOPS at the product
		// width - measured, the jaws ran 90 -> 60 mm half-opening on a 120 mm board and held
		// there. Waiting for GRIP_CLOSED waits for the signal that means the fingers found
		// NOTHING (rb4axis SIM_GRIP_TUTUP). Confirm on the open switch dropping and staying
		// dropped, so a board slipping out of the fingers restarts the confirm.
		IF T_GRIP.Q THEN
			HOLDING := TRUE;
			// the machine it came OUT of is empty now - SRC, never IDX; WIP IN is never emptied
			IF SRC = 1 THEN ICC1_STATE := 0; END_IF;
			IF SRC = 2 THEN ICC2_STATE := 0; END_IF;
			IF SRC = 3 THEN DW1_STATE := 0; END_IF;
			IF SRC = 4 THEN DW2_STATE := 0; END_IF;
			ST1_STEP := 16;
		END_IF;

	16:	A0_TGT := RAIL_X;   A1_TGT := ARM_UP_1;   A2_TGT := ARM_UP_2;   A3_TGT := ARM_UP_3;
		A0_EXEC := TRUE;   A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF AT_POSE THEN ST1_STEP := 17; END_IF;
	17:	A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN ST1_STEP := 18; END_IF;

	18:	A1_TGT := 90.0;   A2_TGT := -90.0;   A3_TGT := -90.0;
		A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF A1_DONE AND A2_DONE AND A3_DONE THEN
			IDX := DST;
			ST1_STEP := 20;
		END_IF;

	20:	A0_TGT := RAIL_X;
		A0_EXEC := TRUE;
		ST1_STEP := 21;

	21:	// same as step 8: the destination cover must be open BEFORE the approach is asked for
		IF A0_DONE AND COVER_OK THEN
			ST1_STEP := 22;
		END_IF;

	22:	A0_TGT := RAIL_X;   A1_TGT := ARM_UP_1;   A2_TGT := ARM_UP_2;   A3_TGT := ARM_UP_3;
		A0_EXEC := TRUE;   A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF AT_POSE THEN ST1_STEP := 23; END_IF;
	23:	A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN ST1_STEP := 24; END_IF;

	24:	A0_TGT := RAIL_X;   A1_TGT := ARM_AT_1;   A2_TGT := ARM_AT_2;   A3_TGT := ARM_AT_3;
		A0_EXEC := TRUE;   A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF AT_POSE THEN ST1_STEP := 25; END_IF;
	25:	A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN ST1_STEP := 26; END_IF;

	26:	GRIP_CLOSE := FALSE;
		ST1_STEP := 27;

	27:	IF GRIP_OPEN THEN
			HOLDING := FALSE;
			// WIP OUT: the product leaves the cell, and THAT is where cycle time is taken -
			// exit to exit, because that is what decides how many boards an hour leave.
			// Written against DST, never IDX: IDX is reused for source then destination, so on a
			// job whose SOURCE was this same station the step-15 clear would zero the state just
			// set here. Measured: ICC 1 was loaded, set to processing, then reported empty again,
			// so its cover never shut and its 17 s clock never ran.
			IF DST = 5 THEN
				CYCLE_CNT := CYCLE_CNT + 1;
			ELSE
				// the station starts its own clock; each runs independently
				IF DST = 1 THEN ICC1_STATE := 1; ICC1_TIMER := 17.0; END_IF;
				IF DST = 2 THEN ICC2_STATE := 1; ICC2_TIMER := 17.0; END_IF;
				IF DST = 3 THEN DW1_STATE := 1; DW1_TIMER := 15.0; END_IF;
				IF DST = 4 THEN DW2_STATE := 1; DW2_TIMER := 15.0; END_IF;
			END_IF;
			ST1_STEP := 28;
		END_IF;

	28:	A0_TGT := RAIL_X;   A1_TGT := ARM_UP_1;   A2_TGT := ARM_UP_2;   A3_TGT := ARM_UP_3;
		A0_EXEC := TRUE;   A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF AT_POSE THEN ST1_STEP := 29; END_IF;
	29:	A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF CLEARED THEN ST1_STEP := 30; END_IF;

	30:	A1_TGT := 90.0;   A2_TGT := -90.0;   A3_TGT := -90.0;
		A1_EXEC := TRUE;   A2_EXEC := TRUE;   A3_EXEC := TRUE;
		IF A1_DONE AND A2_DONE AND A3_DONE THEN
			ST1_STEP := 31;
		END_IF;

	31:	// Cycle stop is served HERE: the product is placed and the arm folded, so stopping is
		// controlled and START works again straight away.
		IF STOP_REQ THEN
			STOP_REQ := FALSE;
		END_IF;
		ST1_STEP := 0;

	900:	// FAULT: a step stopped moving. Hold everything, and KEEP the grip - a real arm does
		// not open its fingers over the floor.
		A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
END_CASE;

// The grip confirm: fingers off the open switch and staying off for 300 ms.
T_GRIP(In := (ST1_STEP = 15) AND GRIP_CLOSE AND NOT GRIP_OPEN, PT := T#300ms);

// Watchdog. 30 s, not 15: a station's own process is 17 s, and the arm legitimately waits at step
// 8 and 21 for a cover that only opens once that test has finished.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900), PT := T#30s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q THEN
	ST1_STEP := 900;
	A0_EXEC := FALSE; A1_EXEC := FALSE; A2_EXEC := FALSE; A3_EXEC := FALSE;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 900);
PL_START := AUTO_RUN;
