VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	T_WD : TON;
	T_PICK : TON;
	T_DROP : TON;
	T_PROC : TON;
	T_FILL : TON;
	STEP_LAST : UDINT;
	ROW : UINT;
	NEED_FILL : BOOL := TRUE;
	PAL_BASE : UDINT;
	FILL_MOVED : BOOL;
	PICKED : BOOL;
	ALL_DONE : BOOL;
	ALL_CLEAR : BOOL;
	ANY_VAC : BOOL;
	NO_VAC : BOOL;
	PAL_UP_1 : LREAL;
	PAL_UP_2 : LREAL;
	PAL_UP_3 : LREAL;
	PAL_UP_4 : LREAL;
	PAL_UP_5 : LREAL;
	PAL_UP_6 : LREAL;
	PAL_AT_1 : LREAL;
	PAL_AT_2 : LREAL;
	PAL_AT_3 : LREAL;
	PAL_AT_4 : LREAL;
	PAL_AT_5 : LREAL;
	PAL_AT_6 : LREAL;
	MEM_VAC : BOOL;
	IND_VAC_LAST : BOOL;
	IND_ACTIVE : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// robot-pitch: a FANUC LR Mate 200iD with a cam-driven pitch-change head. Five cups take a ROW
// of five plugs from the 5 x 5 pallet at 100 mm pitch, the cam closes them to 60 mm, the row is
// set into the five-pocket jig, the jig does its process, and the row is picked again at 60 and
// dropped into the bin. One cycle = one row. The same sequence runs without a PLC in
// robot-pitch.ctl.js.
//
// The poses are joint angles solved once against the scene's kinematics (0.05 mm, >= 30 deg of
// margin to every limit). Moves are joint-space: six targets and Executes together, wait for
// every Done, then drop Execute and wait for Done to clear before the next move.
//
// The panel has a selector AUTO / INDIVIDUAL (rb4axis): START only in AUTO, the individual
// buttons only in INDIVIDUAL, and a change while the sequence runs is a FAULT, not a pause.

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

ALL_DONE := J1_DONE AND J2_DONE AND J3_DONE AND J4_DONE AND J5_DONE AND J6_DONE;
ALL_CLEAR := NOT J1_DONE AND NOT J2_DONE AND NOT J3_DONE AND NOT J4_DONE AND NOT J5_DONE AND NOT J6_DONE;
ANY_VAC := VAC_C0 OR VAC_C1 OR VAC_C2 OR VAC_C3 OR VAC_C4;
NO_VAC := NOT ANY_VAC;

// The pallet row's poses, from the table (a lookup, not a step of the sequence)
CASE ROW OF
	0:
		PAL_UP_1 := -26.56;
		PAL_UP_2 := 11.48;
		PAL_UP_3 := -4.65;
		PAL_UP_4 := 0.01;
		PAL_UP_5 := -73.87;
		PAL_UP_6 := 26.55;
		PAL_AT_1 := -26.57;
		PAL_AT_2 := 16.99;
		PAL_AT_3 := -15.72;
		PAL_AT_4 := -0.01;
		PAL_AT_5 := -57.29;
		PAL_AT_6 := 26.58;
	1:
		PAL_UP_1 := -14.03;
		PAL_UP_2 := 5.51;
		PAL_UP_3 := -11.49;
		PAL_UP_4 := 0.01;
		PAL_UP_5 := -73.00;
		PAL_UP_6 := 14.02;
		PAL_AT_1 := -14.04;
		PAL_AT_2 := 11.47;
		PAL_AT_3 := -22.74;
		PAL_AT_4 := -0.01;
		PAL_AT_5 := -55.79;
		PAL_AT_6 := 14.05;
	2:
		PAL_UP_1 := 0.00;
		PAL_UP_2 := 3.41;
		PAL_UP_3 := -13.75;
		PAL_UP_4 := 0.00;
		PAL_UP_5 := -72.85;
		PAL_UP_6 := 0.00;
		PAL_AT_1 := 0.00;
		PAL_AT_2 := 9.49;
		PAL_AT_3 := -25.11;
		PAL_AT_4 := 0.00;
		PAL_AT_5 := -55.40;
		PAL_AT_6 := 0.00;
	3:
		PAL_UP_1 := 14.03;
		PAL_UP_2 := 5.51;
		PAL_UP_3 := -11.49;
		PAL_UP_4 := -0.01;
		PAL_UP_5 := -73.00;
		PAL_UP_6 := -14.02;
		PAL_AT_1 := 14.04;
		PAL_AT_2 := 11.47;
		PAL_AT_3 := -22.74;
		PAL_AT_4 := 0.01;
		PAL_AT_5 := -55.79;
		PAL_AT_6 := -14.05;
	4:
		PAL_UP_1 := 26.56;
		PAL_UP_2 := 11.48;
		PAL_UP_3 := -4.65;
		PAL_UP_4 := -0.01;
		PAL_UP_5 := -73.87;
		PAL_UP_6 := -26.55;
		PAL_AT_1 := 26.57;
		PAL_AT_2 := 16.99;
		PAL_AT_3 := -15.72;
		PAL_AT_4 := 0.01;
		PAL_AT_5 := -57.29;
		PAL_AT_6 := -26.58;
END_CASE;

// The pallet is FULL when the loader stops delivering: the timer restarts on every plug.
FILL_MOVED := EM_PAL_CNT <> PAL_BASE;
IF FILL_MOVED THEN
	PAL_BASE := EM_PAL_CNT;
END_IF;

CASE ST1_STEP OF
	0:	// idle
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		CAM_EXEC := FALSE;
		EM_PAL_EN := FALSE;
		VAC_ON := FALSE;
		PAL_CLAMP := TRUE;
		JIG_CLAMP := TRUE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED THEN
			STOP_REQ := FALSE;
			IF NEED_FILL THEN
				PAL_BASE := EM_PAL_CNT;
				ST1_STEP := 5;
			ELSE
				ST1_STEP := 10;
			END_IF;
		END_IF;
	5:	// a fresh pallet: full when the loader has delivered nothing for 600 ms
		PAL_CLAMP := TRUE;
		EM_PAL_EN := TRUE;
		IF T_FILL.Q THEN
			EM_PAL_EN := FALSE;
			NEED_FILL := FALSE;
			ROW := 0;
			ST1_STEP := 10;
		END_IF;
	10:	// cam wide, above the row
		J1_TGT := PAL_UP_1;
		J2_TGT := PAL_UP_2;
		J3_TGT := PAL_UP_3;
		J4_TGT := PAL_UP_4;
		J5_TGT := PAL_UP_5;
		J6_TGT := PAL_UP_6;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		CAM_TGT := 0.00;
		CAM_EXEC := TRUE;
		IF ALL_DONE AND CAM_DONE THEN
			ST1_STEP := 11;
		END_IF;
	11:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		CAM_EXEC := FALSE;
		IF ALL_CLEAR AND NOT CAM_DONE THEN
			ST1_STEP := 12;
		END_IF;
	12:	// down onto the row
		J1_TGT := PAL_AT_1;
		J2_TGT := PAL_AT_2;
		J3_TGT := PAL_AT_3;
		J4_TGT := PAL_AT_4;
		J5_TGT := PAL_AT_5;
		J6_TGT := PAL_AT_6;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 13;
		END_IF;
	13:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 14;
		END_IF;
	14:	// the pallet lets go, the cups take what is there
		PAL_CLAMP := FALSE;
		VAC_ON := TRUE;
		IF T_PICK.Q THEN
			PICKED := ANY_VAC;
			ST1_STEP := 15;
		END_IF;
	15:	// up with the row
		PAL_CLAMP := TRUE;
		J1_TGT := PAL_UP_1;
		J2_TGT := PAL_UP_2;
		J3_TGT := PAL_UP_3;
		J4_TGT := PAL_UP_4;
		J5_TGT := PAL_UP_5;
		J6_TGT := PAL_UP_6;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 16;
		END_IF;
	16:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			IF PICKED THEN
				ST1_STEP := 17;
			ELSE
				ST1_STEP := 36;
			END_IF;
		END_IF;
	17:	// cam narrow, above the jig
		J1_TGT := 77.47;
		J2_TGT := 13.83;
		J3_TGT := -1.79;
		J4_TGT := 0.00;
		J5_TGT := -74.38;
		J6_TGT := -77.47;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		CAM_TGT := 90.00;
		CAM_EXEC := TRUE;
		IF ALL_DONE AND CAM_DONE THEN
			ST1_STEP := 18;
		END_IF;
	18:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		CAM_EXEC := FALSE;
		IF ALL_CLEAR AND NOT CAM_DONE THEN
			ST1_STEP := 19;
		END_IF;
	19:	// down into the jig
		J1_TGT := 77.47;
		J2_TGT := 19.16;
		J3_TGT := -12.81;
		J4_TGT := 0.00;
		J5_TGT := -58.04;
		J6_TGT := -77.48;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 20;
		END_IF;
	20:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 21;
		END_IF;
	21:	// the jig takes the row
		JIG_CLAMP := TRUE;
		VAC_ON := FALSE;
		IF T_DROP.Q THEN
			ST1_STEP := 22;
		END_IF;
	22:
		J1_TGT := 77.47;
		J2_TGT := 13.83;
		J3_TGT := -1.79;
		J4_TGT := 0.00;
		J5_TGT := -74.38;
		J6_TGT := -77.47;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 23;
		END_IF;
	23:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 24;
		END_IF;
	24:	// the jig does its work
		IF T_PROC.Q THEN
			ST1_STEP := 25;
		END_IF;
	25:	// back down for the finished row
		J1_TGT := 77.47;
		J2_TGT := 19.16;
		J3_TGT := -12.81;
		J4_TGT := 0.00;
		J5_TGT := -58.04;
		J6_TGT := -77.48;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 26;
		END_IF;
	26:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 27;
		END_IF;
	27:	// the jig lets go, the cups take the row
		JIG_CLAMP := FALSE;
		VAC_ON := TRUE;
		IF T_PICK.Q THEN
			PICKED := ANY_VAC;
			ST1_STEP := 28;
		END_IF;
	28:
		JIG_CLAMP := TRUE;
		J1_TGT := 77.47;
		J2_TGT := 13.83;
		J3_TGT := -1.79;
		J4_TGT := 0.00;
		J5_TGT := -74.38;
		J6_TGT := -77.47;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 29;
		END_IF;
	29:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 30;
		END_IF;
	30:	// over the bin
		J1_TGT := 46.97;
		J2_TGT := 46.74;
		J3_TGT := 55.89;
		J4_TGT := 0.00;
		J5_TGT := -99.14;
		J6_TGT := -46.97;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 31;
		END_IF;
	31:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 32;
		END_IF;
	32:	// drop
		VAC_ON := FALSE;
		IF T_DROP.Q THEN
			ST1_STEP := 33;
		END_IF;
	33:	// cam wide again for the next row
		CAM_TGT := 0.00;
		CAM_EXEC := TRUE;
		IF CAM_DONE THEN
			ST1_STEP := 34;
		END_IF;
	34:
		CAM_EXEC := FALSE;
		IF NOT CAM_DONE THEN
			ST1_STEP := 35;
		END_IF;
	35:	// cycle complete: one row
		CYCLE_CNT := CYCLE_CNT + 1;
		ROW := ROW + 1;
		IF ROW >= 5 THEN
			ROW := 0;
			NEED_FILL := TRUE;
		END_IF;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSIF NEED_FILL THEN
			PAL_BASE := EM_PAL_CNT;
			ST1_STEP := 5;
		ELSE
			ST1_STEP := 10;
		END_IF;
	36:	// nothing came up from this row: not a cycle
		ROW := ROW + 1;
		IF ROW >= 5 THEN
			ROW := 0;
			NEED_FILL := TRUE;
		END_IF;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSIF NEED_FILL THEN
			PAL_BASE := EM_PAL_CNT;
			ST1_STEP := 5;
		ELSE
			ST1_STEP := 10;
		END_IF;
	800:	// HOME: the arm to its travel pose, the cam wide; the cups keep what they hold
		EM_PAL_EN := FALSE;
		J1_TGT := 0.00;
		J2_TGT := 0.00;
		J3_TGT := 0.00;
		J4_TGT := 0.00;
		J5_TGT := -90.00;
		J6_TGT := 0.00;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		CAM_TGT := 0.00;
		CAM_EXEC := TRUE;
		IF ALL_DONE AND CAM_DONE THEN
			J1_EXEC := FALSE;
			J2_EXEC := FALSE;
			J3_EXEC := FALSE;
			J4_EXEC := FALSE;
			J5_EXEC := FALSE;
			J6_EXEC := FALSE;
			CAM_EXEC := FALSE;
			EM_PAL_EN := FALSE;
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, every drive stops where it is. The cups KEEP
		// their plugs: five plugs let go over the pallet is not what a real head does.
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		CAM_EXEC := FALSE;
		EM_PAL_EN := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving, or the selector was turned while running.
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		CAM_EXEC := FALSE;
		EM_PAL_EN := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_PICK(In := (ST1_STEP = 14) OR (ST1_STEP = 27), PT := T#600ms);
T_DROP(In := ((ST1_STEP = 21) OR (ST1_STEP = 32)) AND NO_VAC, PT := T#300ms);
T_PROC(In := (ST1_STEP = 24), PT := T#1s);
T_FILL(In := (ST1_STEP = 5) AND NOT FILL_MOVED, PT := T#600ms);

// Watchdog: a running step that stops moving is a jam, not patience. A selector change while
// running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	J1_EXEC := FALSE;
	J2_EXEC := FALSE;
	J3_EXEC := FALSE;
	J4_EXEC := FALSE;
	J5_EXEC := FALSE;
	J6_EXEC := FALSE;
	CAM_EXEC := FALSE;
	EM_PAL_EN := FALSE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	J1_EXEC := FALSE;
	J2_EXEC := FALSE;
	J3_EXEC := FALSE;
	J4_EXEC := FALSE;
	J5_EXEC := FALSE;
	J6_EXEC := FALSE;
	CAM_EXEC := FALSE;
	EM_PAL_EN := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: jog buttons drive the joints straight (the axis holds its limits), the
// cam buttons command an end, the vacuum toggles. Nothing stays latched into AUTO.
IND_ACTIVE := NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800);
J1_JOG_P := IND_ACTIVE AND IND_J1_P;
J1_JOG_N := IND_ACTIVE AND IND_J1_N;
J2_JOG_P := IND_ACTIVE AND IND_J2_P;
J2_JOG_N := IND_ACTIVE AND IND_J2_N;
J3_JOG_P := IND_ACTIVE AND IND_J3_P;
J3_JOG_N := IND_ACTIVE AND IND_J3_N;
J4_JOG_P := IND_ACTIVE AND IND_J4_P;
J4_JOG_N := IND_ACTIVE AND IND_J4_N;
J5_JOG_P := IND_ACTIVE AND IND_J5_P;
J5_JOG_N := IND_ACTIVE AND IND_J5_N;
J6_JOG_P := IND_ACTIVE AND IND_J6_P;
J6_JOG_N := IND_ACTIVE AND IND_J6_N;
IF IND_ACTIVE THEN
	IF IND_VAC AND NOT IND_VAC_LAST THEN
		MEM_VAC := NOT MEM_VAC;
	END_IF;
	VAC_ON := MEM_VAC;
	IF IND_CAM_WIDE THEN
		CAM_TGT := 0.00;
		CAM_EXEC := TRUE;
	ELSIF IND_CAM_NARROW THEN
		CAM_TGT := 90.00;
		CAM_EXEC := TRUE;
	ELSE
		CAM_EXEC := FALSE;
	END_IF;
ELSE
	MEM_VAC := FALSE;
END_IF;
IND_VAC_LAST := IND_VAC;

// The speed override the operator dialled in, passed on to the axes.
OVR := OVR_SET;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
