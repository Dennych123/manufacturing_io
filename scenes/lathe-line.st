VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	T_WD : TON;
	T_GRIP : TON;
	T_SETTLE1 : TON;
	T_SETTLE2 : TON;
	T_DROP1 : TON;
	T_DROP2 : TON;
	T_CUT1 : TON;
	T_CUT2 : TON;
	T_WD4 : TON;
	T_WD5 : TON;
	STEP_LAST : UDINT;
	TGT : UINT;
	GO_POSE : UINT;
	GO_RAIL : UINT;
	HAS_RAW : BOOL;
	HAS_FIN : BOOL;
	M1_STATE : UINT;
	M2_STATE : UINT;
	S1_GO : BOOL;
	S2_GO : BOOL;
	ALL_DONE : BOOL;
	ALL_CLEAR : BOOL;
	COND : BOOL;
	MEM_A : BOOL;
	MEM_B : BOOL;
	MEM_D1 : BOOL;
	MEM_D2 : BOOL;
	MEM_L1 : BOOL;
	MEM_L2 : BOOL;
	IND_A_LAST : BOOL;
	IND_B_LAST : BOOL;
	IND_D1_LAST : BOOL;
	IND_D2_LAST : BOOL;
	IND_L1_LAST : BOOL;
	IND_L2_LAST : BOOL;
	IND_ACTIVE : BOOL;
	RUNNING : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// lathe-line: a DENSO VS-087 hanging from a traverse beam serves two TAKISAWA TCC-2000 lathes
// (OP10 then OP20) standing side by side, fed by ONE conveyor along their fronts. At each machine
// a stopper pops up out of the belt and a pin lift raises the part to the robot; the same pin
// hands over the raw part and takes the finished one back. The same sequence runs without a PLC
// in lathe-line.ctl.js - keep the two in step.
//
// The hand is a 90 degree DOUBLE hand: A along the flange axis for raw parts, B across it for
// finished ones, so one door opening serves the whole exchange.
//
// A grip is confirmed by the OPEN switch DROPPING, never by CLOSED: CLOSED sits at full close, so
// with a part between the fingers it never comes on.
//
// The joint targets are set by the CASE GO_POSE block at the bottom, which is GENERATED from the
// same IK run as the controller's table (tools/gen_lathe_line.js): 66 angles typed into two
// programs by hand drift, and the drift is silent.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_CSTOP THEN
	STOP_REQ := TRUE;
END_IF;
SEL_CHANGED := SEL_AUTO <> SEL_LAST;
SEL_LAST := SEL_AUTO;

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
GO_POSE := 0;

CASE ST1_STEP OF
	0:	// idle
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		RX_EXEC := FALSE;
		EM_IN_EMIT := FALSE;
		CV_RUN := FALSE;
		GRIP_A := FALSE;
		GRIP_B := FALSE;
		SOL_M1_DOOR := FALSE;
		SOL_M2_DOOR := FALSE;
		M1_CHUCK := TRUE;
		M2_CHUCK := TRUE;
		HAS_RAW := FALSE;
		HAS_FIN := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED THEN
			STOP_REQ := FALSE;
			ST2_STEP := 100;
			ST3_STEP := 100;
			ST4_STEP := 200;
			ST5_STEP := 200;
			ST1_STEP := 10;
		END_IF;
	10:	// which machine: one with a finished part first, it is holding one hostage
		HAS_RAW := FALSE;
		HAS_FIN := FALSE;
		// The pin must be UP with a raw part on it, every time: the robot gives the pin a finished
		// part back at the end of the visit, and what guarantees the pin is free then is that the
		// robot took what was on it. Going for a finished part with the pin empty cornered the
		// cell - the station cannot present an empty pin while the next part stands at the stop
		// over it. A cut part waits in the chuck instead, as the real cell does when the line
		// runs dry. A machine with a part finished is served first: it is holding one hostage.
		IF PX_S1 AND AS_S1_UP AND M1_STATE = 2 THEN
			TGT := 0;
			ST1_STEP := 11;
		ELSIF PX_S2 AND AS_S2_UP AND M2_STATE = 2 THEN
			TGT := 1;
			ST1_STEP := 11;
		ELSIF PX_S1 AND AS_S1_UP AND M1_STATE <> 1 THEN
			TGT := 0;
			ST1_STEP := 11;
		ELSIF PX_S2 AND AS_S2_UP AND M2_STATE <> 1 THEN
			TGT := 1;
			ST1_STEP := 11;
		END_IF;
	11:	// over the pin, hand A leading
		GO_POSE := 7;
		GO_RAIL := TGT + 1;
		RX_EXEC := TRUE;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE AND RX_DONE THEN
			ST1_STEP := 12;
		END_IF;
	12:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		RX_EXEC := FALSE;
		// The pin was up with a part on it when this trip was chosen, and the station stays at its
		// ready step until the robot says it is done, so nothing else can have appeared on it. A
		// part that has GONE (the viewer's hand took it) means no exchange: the machine keeps what
		// it has and the watchdog has the last word.
		IF TGT = 0 THEN
			COND := PX_S1;
		ELSE
			COND := PX_S2;
		END_IF;
		IF ALL_CLEAR AND NOT RX_DONE THEN
			IF COND THEN
				ST1_STEP := 13;
			ELSE
				ST1_STEP := 20;
			END_IF;
		END_IF;
	13:	// down onto the pin
		GO_POSE := 6;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 14;
		END_IF;
	14:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 15;
		END_IF;
	15:	// the pin lets go as the fingers close: a hand-over, as out of a chuck
		GRIP_A := TRUE;
		IF T_GRIP.Q AND NOT AS_A_OPEN THEN
			HAS_RAW := TRUE;
			IF TGT = 0 THEN
				S1_CLAMP := FALSE;
			ELSE
				S2_CLAMP := FALSE;
			END_IF;
			ST1_STEP := 16;
		END_IF;
	16:
		GO_POSE := 7;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 17;
		END_IF;
	17:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 20;
		END_IF;
	20:	// in front of the door
		GO_POSE := 3;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 21;
		END_IF;
	21:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 22;
		END_IF;
	22:	// the door must be OPEN before reaching in
		IF TGT = 0 THEN
			SOL_M1_DOOR := TRUE;
			IF AS_M1_DOOR_OP THEN
				ST1_STEP := 23;
			END_IF;
		ELSE
			SOL_M2_DOOR := TRUE;
			IF AS_M2_DOOR_OP THEN
				ST1_STEP := 23;
			END_IF;
		END_IF;
	23:
		IF (TGT = 0 AND M1_STATE = 2) OR (TGT = 1 AND M2_STATE = 2) THEN
			ST1_STEP := 30;
		ELSIF HAS_RAW THEN
			ST1_STEP := 40;
		ELSE
			ST1_STEP := 60;
		END_IF;
	30:	// hand B into the machine, clear of the chuck
		GO_POSE := 5;
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
	32:	// onto the finished part
		GO_POSE := 4;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 33;
		END_IF;
	33:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 34;
		END_IF;
	34:	// grip, then the chuck opens
		GRIP_B := TRUE;
		IF T_GRIP.Q AND NOT AS_B_OPEN THEN
			HAS_FIN := TRUE;
			IF TGT = 0 THEN
				M1_CHUCK := FALSE;
				M1_STATE := 0;
			ELSE
				M2_CHUCK := FALSE;
				M2_STATE := 0;
			END_IF;
			ST1_STEP := 35;
		END_IF;
	35:
		GO_POSE := 5;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 36;
		END_IF;
	36:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			IF HAS_RAW THEN
				ST1_STEP := 40;
			ELSE
				ST1_STEP := 60;
			END_IF;
		END_IF;
	40:	// hand A into the machine, clear of the chuck
		GO_POSE := 2;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 41;
		END_IF;
	41:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 42;
		END_IF;
	42:	// the raw part into the chuck
		GO_POSE := 1;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 43;
		END_IF;
	43:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 44;
		END_IF;
	44:	// the chuck closes, the hand opens, the cut starts
		GRIP_A := FALSE;
		IF TGT = 0 THEN
			M1_CHUCK := TRUE;
		ELSE
			M2_CHUCK := TRUE;
		END_IF;
		IF T_GRIP.Q AND AS_A_OPEN THEN
			HAS_RAW := FALSE;
			IF TGT = 0 THEN
				M1_STATE := 1;
			ELSE
				M2_STATE := 1;
			END_IF;
			ST1_STEP := 45;
		END_IF;
	45:
		GO_POSE := 2;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 46;
		END_IF;
	46:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 60;
		END_IF;
	60:	// out of the machine
		GO_POSE := 3;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 61;
		END_IF;
	61:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 62;
		END_IF;
	62:	// shut the door on the cut
		IF TGT = 0 THEN
			SOL_M1_DOOR := FALSE;
			COND := AS_M1_DOOR_CL;
		ELSE
			SOL_M2_DOOR := FALSE;
			COND := AS_M2_DOOR_CL;
		END_IF;
		IF COND THEN
			IF HAS_FIN THEN
				ST1_STEP := 70;
			ELSE
				ST1_STEP := 80;
			END_IF;
		END_IF;
	70:	// hand B over the pin the part came off
		GO_POSE := 9;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 71;
		END_IF;
	71:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 72;
		END_IF;
	72:
		GO_POSE := 8;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 73;
		END_IF;
	73:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 74;
		END_IF;
	74:	// the pin takes it, the hand lets go
		GRIP_B := FALSE;
		IF TGT = 0 THEN
			S1_CLAMP := TRUE;
			COND := PX_S1;
		ELSE
			S2_CLAMP := TRUE;
			COND := PX_S2;
		END_IF;
		IF T_GRIP.Q AND AS_B_OPEN AND COND THEN
			HAS_FIN := FALSE;
			ST1_STEP := 75;
		END_IF;
	75:
		GO_POSE := 9;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 76;
		END_IF;
	76:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 80;
		END_IF;
	80:	// done at this station: the pin may go down and the station is free again
		IF TGT = 0 THEN
			S1_GO := TRUE;
		ELSE
			S2_GO := TRUE;
		END_IF;
		CYCLE_CNT := CYCLE_CNT + 1;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			ST1_STEP := 10;
		END_IF;
	800:	// HOME: the arm to its travel pose over machine 1
		EM_IN_EMIT := FALSE;
		CV_RUN := FALSE;
		SOL_M1_DOOR := FALSE;
		SOL_M2_DOOR := FALSE;
		GO_POSE := 10;
		GO_RAIL := 1;
		RX_EXEC := TRUE;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE AND RX_DONE THEN
			J1_EXEC := FALSE;
			J2_EXEC := FALSE;
			J3_EXEC := FALSE;
			J4_EXEC := FALSE;
			J5_EXEC := FALSE;
			J6_EXEC := FALSE;
			RX_EXEC := FALSE;
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	900:	// stuck, or the selector was turned while running
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		RX_EXEC := FALSE;
		EM_IN_EMIT := FALSE;
		CV_RUN := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
	910:	// the master circuit is open: the jaws, the chucks and the pins KEEP what they hold
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		RX_EXEC := FALSE;
		EM_IN_EMIT := FALSE;
		CV_RUN := FALSE;
		IF READY THEN
			ST1_STEP := 0;
		END_IF;
END_CASE;

RUNNING := ST1_STEP <> 0 AND ST1_STEP <> 800 AND ST1_STEP <> 900 AND ST1_STEP <> 910;
IF RUNNING THEN
	CV_RUN := TRUE;
END_IF;

// ---------------------------------------------------------- ST2: machine 1, the cut
IF NOT RUNNING THEN
	ST2_STEP := 0;
ELSE
	CASE ST2_STEP OF
		100:
			IF M1_STATE = 1 THEN
				ST2_STEP := 110;
			END_IF;
		110:	// the door stays shut and the chuck stays closed for the whole of it
			IF T_CUT1.Q THEN
				M1_STATE := 2;
				ST2_STEP := 120;
			END_IF;
		120:	// the robot has taken it
			IF M1_STATE <> 2 THEN
				ST2_STEP := 100;
			END_IF;
	ELSE
		ST2_STEP := 100;
	END_CASE;
END_IF;

// ---------------------------------------------------------- ST3: machine 2, the cut
IF NOT RUNNING THEN
	ST3_STEP := 0;
ELSE
	CASE ST3_STEP OF
		100:
			IF M2_STATE = 1 THEN
				ST3_STEP := 110;
			END_IF;
		110:
			IF T_CUT2.Q THEN
				M2_STATE := 2;
				ST3_STEP := 120;
			END_IF;
		120:
			IF M2_STATE <> 2 THEN
				ST3_STEP := 100;
			END_IF;
	ELSE
		ST3_STEP := 100;
	END_CASE;
END_IF;

// ---------------------------------------------------------- ST4: station 1 and the feed
IF NOT RUNNING THEN
	ST4_STEP := 0;
	EM_IN_EMIT := FALSE;
ELSE
	CASE ST4_STEP OF
		200:	// down, stop up, nothing on the pin: call for one raw part
			SOL_S1_UP := FALSE;
			SOL_S1_DN := TRUE;
			SOL_S1_STOP := TRUE;
			S1_CLAMP := FALSE;
			S1_GO := FALSE;
			IF AS_S1_STOP_UP AND NOT PE_S1 THEN
				EM_IN_EMIT := TRUE;
				ST4_STEP := 210;
			END_IF;
		210:
			EM_IN_EMIT := FALSE;
			IF PE_S1 THEN
				ST4_STEP := 220;
			END_IF;
		220:	// the beam says the part is HERE; the STOP is what locates it, so keep driving
			IF T_SETTLE1.Q THEN
				SOL_S1_DN := FALSE;
				SOL_S1_UP := TRUE;
				S1_CLAMP := TRUE;
				ST4_STEP := 230;
			END_IF;
		230:
			IF AS_S1_UP THEN
				ST4_STEP := 240;
			END_IF;
		240:	// ready for the robot, and waiting for it to finish at this station
			IF S1_GO THEN
				S1_GO := FALSE;
				IF PX_S1 THEN
					ST4_STEP := 250;
				ELSE
					ST4_STEP := 270;
				END_IF;
			END_IF;
		250:	// let go FIRST, so the part rides the pin down and the pin drops away under it
			S1_CLAMP := FALSE;
			IF T_DROP1.Q THEN
				ST4_STEP := 260;
			END_IF;
		260:	// the STOP goes down BEFORE the pin: with it still up the part comes down onto the
			// 30 mm stop head instead of the belt, stands on its rim and topples. Station 2 must
			// be empty and clear too: two parts at one stop cannot be metered apart again.
			IF ST5_STEP = 200 AND NOT PE_S2 THEN
				SOL_S1_STOP := FALSE;
				ST4_STEP := 262;
			END_IF;
		262:
			IF AS_S1_STOP_DN THEN
				SOL_S1_UP := FALSE;
				SOL_S1_DN := TRUE;
				ST4_STEP := 265;
			END_IF;
		265:
			IF AS_S1_DN AND NOT PE_S1 THEN
				SOL_S1_STOP := TRUE;
				ST4_STEP := 200;
			END_IF;
		270:
			S1_CLAMP := FALSE;
			SOL_S1_UP := FALSE;
			SOL_S1_DN := TRUE;
			IF AS_S1_DN THEN
				ST4_STEP := 200;
			END_IF;
	ELSE
		ST4_STEP := 200;
	END_CASE;
END_IF;

// ---------------------------------------------------------- ST5: station 2 and the discharge
IF NOT RUNNING THEN
	ST5_STEP := 0;
ELSE
	CASE ST5_STEP OF
		200:
			SOL_S2_UP := FALSE;
			SOL_S2_DN := TRUE;
			SOL_S2_STOP := TRUE;
			S2_CLAMP := FALSE;
			S2_GO := FALSE;
			IF PE_S2 THEN
				ST5_STEP := 220;
			END_IF;
		220:
			IF T_SETTLE2.Q THEN
				SOL_S2_DN := FALSE;
				SOL_S2_UP := TRUE;
				S2_CLAMP := TRUE;
				ST5_STEP := 230;
			END_IF;
		230:
			IF AS_S2_UP THEN
				ST5_STEP := 240;
			END_IF;
		240:
			IF S2_GO THEN
				S2_GO := FALSE;
				IF PX_S2 THEN
					ST5_STEP := 250;
				ELSE
					ST5_STEP := 270;
				END_IF;
			END_IF;
		250:
			S2_CLAMP := FALSE;
			IF T_DROP2.Q THEN
				ST5_STEP := 260;
			END_IF;
		260:
			SOL_S2_STOP := FALSE;
			ST5_STEP := 262;
		262:
			IF AS_S2_STOP_DN THEN
				SOL_S2_UP := FALSE;
				SOL_S2_DN := TRUE;
				ST5_STEP := 265;
			END_IF;
		265:
			IF AS_S2_DN AND NOT PE_S2 THEN
				SOL_S2_STOP := TRUE;
				ST5_STEP := 200;
			END_IF;
		270:
			S2_CLAMP := FALSE;
			SOL_S2_UP := FALSE;
			SOL_S2_DN := TRUE;
			IF AS_S2_DN THEN
				ST5_STEP := 200;
			END_IF;
	ELSE
		ST5_STEP := 200;
	END_CASE;
END_IF;

// The joint targets for the pose the step asked for. GENERATED - see the header.
// BEGIN GENERATED - tools/gen_lathe_line.js
CASE GO_POSE OF
	1:	// chAtA
		J1_TGT := -93.18;
		J2_TGT := 7.66;
		J3_TGT := -37.91;
		J4_TGT := 91.60;
		J5_TGT := -87.24;
		J6_TGT := 0.00;
	2:	// chOutA
		J1_TGT := -74.48;
		J2_TGT := 10.03;
		J3_TGT := -40.73;
		J4_TGT := 81.92;
		J5_TGT := -103.29;
		J6_TGT := 0.00;
	3:	// doorA
		J1_TGT := -124.77;
		J2_TGT := -54.34;
		J3_TGT := -17.86;
		J4_TGT := 123.46;
		J5_TGT := -79.95;
		J6_TGT := 0.00;
	4:	// chAtB
		J1_TGT := -108.80;
		J2_TGT := -1.09;
		J3_TGT := -12.69;
		J4_TGT := -7.37;
		J5_TGT := -57.72;
		J6_TGT := 111.71;
	5:	// chOutB
		J1_TGT := -88.59;
		J2_TGT := -4.61;
		J3_TGT := -9.64;
		J4_TGT := 0.66;
		J5_TGT := -53.28;
		J6_TGT := 88.31;
	6:	// pinAtA
		J1_TGT := 66.86;
		J2_TGT := -42.35;
		J3_TGT := 8.50;
		J4_TGT := 0.01;
		J5_TGT := -56.15;
		J6_TGT := 0.00;
	7:	// pinUpA
		J1_TGT := 66.82;
		J2_TGT := -51.18;
		J3_TGT := 29.70;
		J4_TGT := 0.00;
		J5_TGT := -68.51;
		J6_TGT := 0.00;
	8:	// pinAtB
		J1_TGT := 71.46;
		J2_TGT := -41.47;
		J3_TGT := -14.24;
		J4_TGT := -3.70;
		J5_TGT := 55.75;
		J6_TGT := 2.09;
	9:	// pinUpB
		J1_TGT := 71.08;
		J2_TGT := -54.82;
		J3_TGT := 10.71;
		J4_TGT := -4.03;
		J5_TGT := 44.19;
		J6_TGT := 2.90;
	10:	// home
		J1_TGT := 4.43;
		J2_TGT := -47.26;
		J3_TGT := 42.85;
		J4_TGT := 0.01;
		J5_TGT := -85.59;
		J6_TGT := 0.00;
END_CASE;
CASE GO_RAIL OF
	1:	RX_TGT := -980.00;
	2:	RX_TGT := 620.00;
END_CASE;
// END GENERATED

// TONs after the CASE: their Q is read by the NEXT scan.
T_GRIP(In := ST1_STEP = 15 OR ST1_STEP = 34 OR ST1_STEP = 44 OR ST1_STEP = 74, PT := T#400ms);
T_SETTLE1(In := ST4_STEP = 220, PT := T#700ms);
T_SETTLE2(In := ST5_STEP = 220, PT := T#700ms);
T_DROP1(In := ST4_STEP = 250, PT := T#200ms);
T_DROP2(In := ST5_STEP = 250, PT := T#200ms);
T_CUT1(In := M1_STATE = 1, PT := T#20s);
T_CUT2(In := M2_STATE = 1, PT := T#20s);

// Watchdog: a running step that stops moving is a jam, not patience. A station that stops moving
// is one too - a part taken off the belt by hand leaves one waiting for a beam that never comes,
// and a machine that waits for ever looks alive.
// The TON is called BEFORE STEP_LAST is updated, so a step change gives it one scan of In = FALSE
// and resets it. Updating first would make the compare always true and the watchdog never reset.
T_WD(In := ST1_STEP <> 0 AND ST1_STEP <> 900 AND ST1_STEP <> 910 AND ST1_STEP = STEP_LAST, PT := T#45s);
STEP_LAST := ST1_STEP;
// Only the station steps that MOVE something are watched. The ones that wait for the line - for a
// part to be sent on (200, 210) and for the robot (240) - are not jams: the robot is 45 s away
// whenever it stands over the other machine through a 20 s cut.
T_WD4(In := RUNNING AND ST4_STEP >= 220 AND ST4_STEP <> 240, PT := T#45s);
T_WD5(In := RUNNING AND ST5_STEP >= 220 AND ST5_STEP <> 240, PT := T#45s);
IF T_WD.Q OR T_WD4.Q OR T_WD5.Q OR (SEL_CHANGED AND RUNNING) THEN
	ST1_STEP := 900;
	J1_EXEC := FALSE;
	J2_EXEC := FALSE;
	J3_EXEC := FALSE;
	J4_EXEC := FALSE;
	J5_EXEC := FALSE;
	J6_EXEC := FALSE;
	RX_EXEC := FALSE;
	EM_IN_EMIT := FALSE;
	CV_RUN := FALSE;
END_IF;

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
	RX_EXEC := FALSE;
	EM_IN_EMIT := FALSE;
	CV_RUN := FALSE;
ELSIF HOME_EDGE AND READY AND ST1_STEP = 0 THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := ST1_STEP <> 0 AND ST1_STEP <> 800 AND ST1_STEP <> 900 AND ST1_STEP <> 910;
IND_ACTIVE := NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND ST1_STEP <> 800;
IF IND_ACTIVE THEN
	IF IND_A AND NOT IND_A_LAST THEN
		MEM_A := NOT MEM_A;
	END_IF;
	IF IND_B AND NOT IND_B_LAST THEN
		MEM_B := NOT MEM_B;
	END_IF;
	IF IND_DOOR1 AND NOT IND_D1_LAST THEN
		MEM_D1 := NOT MEM_D1;
	END_IF;
	IF IND_DOOR2 AND NOT IND_D2_LAST THEN
		MEM_D2 := NOT MEM_D2;
	END_IF;
	IF IND_LIFT1 AND NOT IND_L1_LAST THEN
		MEM_L1 := NOT MEM_L1;
	END_IF;
	IF IND_LIFT2 AND NOT IND_L2_LAST THEN
		MEM_L2 := NOT MEM_L2;
	END_IF;
	GRIP_A := MEM_A;
	GRIP_B := MEM_B;
	SOL_M1_DOOR := MEM_D1;
	SOL_M2_DOOR := MEM_D2;
	SOL_S1_UP := MEM_L1;
	SOL_S1_DN := NOT MEM_L1;
	SOL_S2_UP := MEM_L2;
	SOL_S2_DN := NOT MEM_L2;
ELSE
	MEM_A := FALSE;
	MEM_B := FALSE;
	MEM_D1 := FALSE;
	MEM_D2 := FALSE;
	MEM_L1 := FALSE;
	MEM_L2 := FALSE;
END_IF;
IND_A_LAST := IND_A;
IND_B_LAST := IND_B;
IND_D1_LAST := IND_DOOR1;
IND_D2_LAST := IND_DOOR2;
IND_L1_LAST := IND_LIFT1;
IND_L2_LAST := IND_LIFT2;
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
RX_JOG_P := IND_ACTIVE AND IND_RX_P;
RX_JOG_N := IND_ACTIVE AND IND_RX_N;

OVR := OVR_SET;
PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
