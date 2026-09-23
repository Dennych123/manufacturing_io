VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	T_WD : TON;
	T_GRIP : TON;
	T_SETTLE : TON;
	T_DROP : TON;
	T_OSTOP : TON;
	T_OHOLD : TON;
	T_QUE : TON;
	T_CUT1 : TON;
	T_CUT2 : TON;
	T_WD4 : TON;
	T_WD5 : TON;
	T_WD6 : TON;
	EM_LAST : UDINT;
	EM_MAN_LAST : UDINT;
	IN_FLIGHT : BOOL;
	FEED_REQ : BOOL;
	FEED_EDGE : BOOL;
	IND_FEED_LAST : BOOL;
	MAN_EMIT : BOOL;
	STEP_LAST : UDINT;
	STEP4_LAST : UDINT;
	STEP5_LAST : UDINT;
	STEP6_LAST : UDINT;
	TGT : UINT;
	GO_POSE : UINT;
	GO_RAIL : UINT;
	HAS_RAW : BOOL;
	HAS_FIN : BOOL;
	M1_STATE : UINT;
	M2_STATE : UINT;
	OUT_GO : BOOL;
	ALL_DONE : BOOL;
	ALL_CLEAR : BOOL;
	COND : BOOL;
	RUNNING : BOOL;
	MEM_A : BOOL;
	MEM_B : BOOL;
	MEM_D1 : BOOL;
	MEM_D2 : BOOL;
	MEM_LI : BOOL;
	MEM_LO : BOOL;
	MEM_CVI : BOOL;
	MEM_CVO : BOOL;
	MEM_SI : BOOL;
	MEM_HI : BOOL;
	MEM_QI : BOOL;
	MEM_BL : BOOL;
	MEM_SO : BOOL;
	MEM_HO : BOOL;
	IND_A_LAST : BOOL;
	IND_B_LAST : BOOL;
	IND_D1_LAST : BOOL;
	IND_D2_LAST : BOOL;
	IND_LI_LAST : BOOL;
	IND_LO_LAST : BOOL;
	IND_CVI_LAST : BOOL;
	IND_CVO_LAST : BOOL;
	IND_SI_LAST : BOOL;
	IND_HI_LAST : BOOL;
	IND_QI_LAST : BOOL;
	IND_BL_LAST : BOOL;
	IND_SO_LAST : BOOL;
	IND_HO_LAST : BOOL;
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
// lathe-line: a NDESO NDESO-087 hanging from a traverse beam serves two TAKISAWA TCC-2000 lathes
// that run the SAME operation in PARALLEL. Raw castings arrive on the infeed conveyor and finished
// parts leave on a SEPARATE outfeed conveyor - neither lathe feeds the other. Both conveyors have
// a stop and a pin lift at the loading end of the cell, where the robot works them. The lathes'
// doors SLIDE sideways. The same sequence runs without a PLC in lathe-line.ctl.js - keep the two
// in step.
//
// The hand is a 90 degree DOUBLE hand: A along the flange axis for raw castings, B across it for
// finished parts, so one door opening serves the whole exchange.
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
		CVIN_RUN := FALSE;
		CVOUT_RUN := FALSE;
		GRIP_A := FALSE;
		GRIP_B := FALSE;
		SOL_M1_DOOR := FALSE;
		SOL_M2_DOOR := FALSE;
		M1_CHUCK := TRUE;
		M2_CHUCK := TRUE;
		HAS_RAW := FALSE;
		HAS_FIN := FALSE;
		OUT_GO := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED THEN
			STOP_REQ := FALSE;
			ST2_STEP := 100;
			ST3_STEP := 100;
			ST4_STEP := 200;
			ST5_STEP := 200;
			ST6_STEP := 200;
			ST1_STEP := 10;
		END_IF;
	10:	// over the infeed pin, hand A leading
		GO_POSE := 8;
		GO_RAIL := 3;
		RX_EXEC := TRUE;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE AND RX_DONE THEN
			ST1_STEP := 11;
		END_IF;
	11:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		RX_EXEC := FALSE;
		IF ALL_CLEAR AND NOT RX_DONE THEN
			ST1_STEP := 12;
		END_IF;
	12:	// the infeed station presents one casting at a time and holds it up until it is taken
		IF ST4_STEP = 240 AND PX_IN THEN
			ST1_STEP := 13;
		END_IF;
	13:	// down onto the pin
		GO_POSE := 7;
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
			IN_CLAMP := FALSE;
			ST1_STEP := 16;
		END_IF;
	16:
		GO_POSE := 8;
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
	20:	// which lathe: one with a part FINISHED first, else an empty one. Both run the same job.
		IF M1_STATE = 2 THEN
			TGT := 0;
			ST1_STEP := 21;
		ELSIF M2_STATE = 2 THEN
			TGT := 1;
			ST1_STEP := 21;
		ELSIF M1_STATE = 0 THEN
			TGT := 0;
			ST1_STEP := 21;
		ELSIF M2_STATE = 0 THEN
			TGT := 1;
			ST1_STEP := 21;
		END_IF;
	21:	// in front of the door
		GO_POSE := 3;
		GO_RAIL := TGT + 1;
		RX_EXEC := TRUE;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE AND RX_DONE THEN
			ST1_STEP := 22;
		END_IF;
	22:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		RX_EXEC := FALSE;
		IF ALL_CLEAR AND NOT RX_DONE THEN
			ST1_STEP := 23;
		END_IF;
	23:	// the door must be OPEN before reaching in
		IF TGT = 0 THEN
			SOL_M1_DOOR := TRUE;
			COND := AS_M1_DOOR_OP;
		ELSE
			SOL_M2_DOOR := TRUE;
			COND := AS_M2_DOOR_OP;
		END_IF;
		IF COND THEN
			ST1_STEP := 24;
		END_IF;
	24:
		IF (TGT = 0 AND M1_STATE = 2) OR (TGT = 1 AND M2_STATE = 2) THEN
			ST1_STEP := 30;
		ELSE
			ST1_STEP := 40;
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
			ST1_STEP := 40;
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
	42:	// the raw casting into the chuck
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
	62:	// slide the door shut on the cut
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
	70:	// over the outfeed pin, hand B leading - the outfeed is a separate belt, further left
		GO_POSE := 10;
		GO_RAIL := 4;
		RX_EXEC := TRUE;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE AND RX_DONE THEN
			ST1_STEP := 71;
		END_IF;
	71:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		RX_EXEC := FALSE;
		IF ALL_CLEAR AND NOT RX_DONE THEN
			ST1_STEP := 72;
		END_IF;
	72:	// the outfeed pin has to be up and EMPTY
		IF ST5_STEP = 240 AND NOT PX_OUT THEN
			ST1_STEP := 73;
		END_IF;
	73:
		GO_POSE := 9;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 74;
		END_IF;
	74:
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			ST1_STEP := 75;
		END_IF;
	75:	// the pin takes it, the hand lets go
		GRIP_B := FALSE;
		OUT_CLAMP := TRUE;
		IF T_GRIP.Q AND AS_B_OPEN AND PX_OUT THEN
			HAS_FIN := FALSE;
			ST1_STEP := 76;
		END_IF;
	76:
		GO_POSE := 10;
		J1_EXEC := TRUE;
		J2_EXEC := TRUE;
		J3_EXEC := TRUE;
		J4_EXEC := TRUE;
		J5_EXEC := TRUE;
		J6_EXEC := TRUE;
		IF ALL_DONE THEN
			ST1_STEP := 77;
		END_IF;
	77:	// the station is told only once the hand is CLEAR of the pin: told when the fingers opened
		// instead, the pin let go while the jaw was still retreating and the finger flicked the
		// part off the pin and across the cell (measured)
		J1_EXEC := FALSE;
		J2_EXEC := FALSE;
		J3_EXEC := FALSE;
		J4_EXEC := FALSE;
		J5_EXEC := FALSE;
		J6_EXEC := FALSE;
		IF ALL_CLEAR THEN
			OUT_GO := TRUE;
			ST1_STEP := 80;
		END_IF;
	80:	// one exchange is one cycle
		CYCLE_CNT := CYCLE_CNT + 1;
		IF STOP_REQ THEN
			ST1_STEP := 0;
		ELSE
			ST1_STEP := 10;
		END_IF;
	800:	// HOME: the arm to its travel pose over the conveyor stations
		EM_IN_EMIT := FALSE;
		CVIN_RUN := FALSE;
		CVOUT_RUN := FALSE;
		SOL_M1_DOOR := FALSE;
		SOL_M2_DOOR := FALSE;
		GO_POSE := 11;
		GO_RAIL := 3;
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
		CVIN_RUN := FALSE;
		CVOUT_RUN := FALSE;
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
		CVIN_RUN := FALSE;
		CVOUT_RUN := FALSE;
		IF READY THEN
			ST1_STEP := 0;
		END_IF;
END_CASE;

RUNNING := ST1_STEP <> 0 AND ST1_STEP <> 800 AND ST1_STEP <> 900 AND ST1_STEP <> 910;
// The outfeed belt runs whenever the cell does. The infeed belt and the feeder belong to ST4: the
// feeder is called one casting at a time, and a command held until it is answered is the only
// thing that says a casting was really made.
IF RUNNING THEN
	CVOUT_RUN := TRUE;
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

// ---------------------------------------------------------- ST4: the infeed station
// The lift and the hold stop. ST6 keeps a casting standing at the hold while this station works.
IF NOT RUNNING THEN
	ST4_STEP := 0;
	CVIN_RUN := FALSE;
ELSE
	CASE ST4_STEP OF
		200:	// armed: pin down, station stop up, hold up. The belt runs the whole time - what
			// keeps the castings apart is the escapement, not a belt that has to be switched off.
			CVIN_RUN := TRUE;
			SOL_IN_UP := FALSE;
			SOL_IN_DN := TRUE;
			IN_CLAMP := FALSE;
			SOL_IN_STOP := TRUE;
			SOL_IN_HOLD := TRUE;
			IF AS_IN_DN AND AS_IN_STOP_UP AND AS_IN_HOLD_UP THEN
				ST4_STEP := 205;
			END_IF;
		205:	// one standing at the hold AND this station empty: the pin down and the beam clear.
			// Without the second half the station pulls a casting up behind the one it already
			// holds - measured, the feeder then ran at one casting every 4.6 s against a 13 s
			// cycle and they piled up where they cannot be separated.
			IF PE_IN_HOLD AND NOT PE_IN AND AS_IN_DN THEN
				ST4_STEP := 220;
			END_IF;
		220:	// GO: the hold drops and that casting runs on to the station stop
			SOL_IN_HOLD := FALSE;
			IF AS_IN_HOLD_DN THEN
				ST4_STEP := 225;
			END_IF;
		225:	// the hold comes back up only once the casting has reached the STATION beam, 250 mm
			// away: its own beam clears with the casting still over it, and 120 ms of stroke at
			// 250 mm/s is 5 mm of margin
			IF PE_IN THEN
				SOL_IN_HOLD := TRUE;
				ST4_STEP := 230;
			END_IF;
		230:
			IF AS_IN_HOLD_UP THEN
				ST4_STEP := 235;
			END_IF;
		235:	// the beam says the casting is HERE; the STOP is what locates it, so keep driving
			IF T_SETTLE.Q THEN
				SOL_IN_DN := FALSE;
				SOL_IN_UP := TRUE;
				IN_CLAMP := TRUE;
				ST4_STEP := 238;
			END_IF;
		238:	// up WITH the casting: a lift that came up empty is a jam, not a cycle
			IF AS_IN_UP AND PX_IN THEN
				ST4_STEP := 240;
			END_IF;
		240:	// presented to the robot; it is gone when the pin reports empty
			IF NOT PX_IN THEN
				ST4_STEP := 270;
			END_IF;
		270:	// down onto free belt: the next casting is held 250 mm back at the hold
			IN_CLAMP := FALSE;
			SOL_IN_UP := FALSE;
			SOL_IN_DN := TRUE;
			IF AS_IN_DN THEN
				ST4_STEP := 200;
			END_IF;
	ELSE
		ST4_STEP := 200;
	END_CASE;
END_IF;

// ---------------------------------------------------------- ST6: the infeed escapement
// The queue stop, the pile blade and the feeder, running WHILE the lift works. One casting leaves
// the queue per pass, however many are piled up behind it:
//   blade IN between the front casting and the next -> the pile is held
//   stop DOWN -> the front casting runs on to the hold
//   stop UP   -> on free belt: the pile is 50 mm back, held by the blade
//   blade OUT -> the pile walks forward exactly one casting, and the seam is at the blade again
// Nothing ever comes up underneath a casting, which is the one thing this lane cannot survive: a
// stop rising under one throws it into the air and it sails over the next stop at twice belt speed.
IF NOT RUNNING THEN
	ST6_STEP := 0;
	EM_IN_EMIT := FALSE;
ELSE
	CASE ST6_STEP OF
		200:
			SOL_IN_QUE := TRUE;
			SOL_IN_BLADE := FALSE;
			IF AS_IN_QUE_UP AND AS_IN_BLADE_OUT THEN
				ST6_STEP := 205;
			END_IF;
		205:	// the hold is empty and armed: send the next casting down to it. Otherwise keep
			// this stop's own place filled, so one always stands ready behind the lift.
			IF NOT PE_IN_HOLD AND AS_IN_HOLD_UP THEN
				ST6_STEP := 210;
			ELSE
				ST6_STEP := 230;
			END_IF;
		210:
			IF PE_IN_QUE THEN
				ST6_STEP := 212;
			ELSIF IN_FLIGHT THEN
				ST6_STEP := 215;
			ELSE
				EM_LAST := EM_IN_CNT;
				EM_IN_EMIT := TRUE;
				ST6_STEP := 211;
			END_IF;
		211:	// the command is HELD until the feeder count moves
			EM_IN_EMIT := TRUE;
			IF EM_IN_CNT <> EM_LAST THEN
				EM_IN_EMIT := FALSE;
				IN_FLIGHT := TRUE;
				ST6_STEP := 215;
			END_IF;
		215:
			IF PE_IN_QUE THEN
				ST6_STEP := 212;
			END_IF;
		212:	// the blade goes in at the seam, between the casting at the stop and the next one.
			// With no pile it goes into free belt and holds nothing: same sequence either way.
			SOL_IN_BLADE := TRUE;
			IF AS_IN_BLADE_IN THEN
				ST6_STEP := 214;
			END_IF;
		214:	// never while the hold is down, or the casting runs straight through to the lift
			IF AS_IN_HOLD_UP THEN
				SOL_IN_QUE := FALSE;
				IF AS_IN_QUE_DN THEN
					ST6_STEP := 216;
				END_IF;
			END_IF;
		216:	// up again as soon as the casting stands at the hold: the pile cannot follow it,
			// the blade has it, so this stop always comes up on free belt
			IF PE_IN_HOLD THEN
				SOL_IN_QUE := TRUE;
				ST6_STEP := 218;
			END_IF;
		218:
			IF AS_IN_QUE_UP THEN
				ST6_STEP := 220;
			END_IF;
		220:	// blade out: the pile walks forward exactly one casting
			SOL_IN_BLADE := FALSE;
			IF AS_IN_BLADE_OUT THEN
				ST6_STEP := 222;
			END_IF;
		222:	// let it settle against the stop before the blade goes back in
			IF T_QUE.Q THEN
				ST6_STEP := 205;
			END_IF;
		230:	// top the queue stop up, so the next casting is already standing there when the
			// hold empties. One on its way counts as one: it is the lane's work in progress.
			IF PE_IN_QUE OR IN_FLIGHT THEN
				ST6_STEP := 205;
			ELSE
				EM_LAST := EM_IN_CNT;
				EM_IN_EMIT := TRUE;
				ST6_STEP := 235;
			END_IF;
		235:
			EM_IN_EMIT := TRUE;
			IF EM_IN_CNT <> EM_LAST THEN
				EM_IN_EMIT := FALSE;
				IN_FLIGHT := TRUE;
				ST6_STEP := 205;
			END_IF;
	ELSE
		ST6_STEP := 200;
	END_CASE;
END_IF;

// A casting that has reached the queue stop is no longer on its way.
IF PE_IN_QUE THEN
	IN_FLIGHT := FALSE;
END_IF;
// The FEED button, in ANY mode: in INDIVIDUAL it drops a casting itself (below), and while the
// cell runs it ASKS here and the station feeds when there is room. The ask is held until it can be
// served - a lane with a casting at the queue stop, or one already on its way, has nowhere to put
// another, and two castings touching at one stop never come apart again.
FEED_EDGE := IND_FEED AND NOT IND_FEED_LAST;
IND_FEED_LAST := IND_FEED;
IF RUNNING THEN
	IF FEED_EDGE THEN
		FEED_REQ := TRUE;
	END_IF;
	IF FEED_REQ AND NOT MAN_EMIT AND NOT PE_IN_QUE AND NOT IN_FLIGHT THEN
		MAN_EMIT := TRUE;
		EM_MAN_LAST := EM_IN_CNT;
	END_IF;
	IF MAN_EMIT THEN
		EM_IN_EMIT := TRUE;
		IF EM_IN_CNT <> EM_MAN_LAST THEN
			EM_IN_EMIT := FALSE;
			MAN_EMIT := FALSE;
			IN_FLIGHT := TRUE;
			FEED_REQ := FALSE;
		END_IF;
	END_IF;
ELSE
	FEED_REQ := FALSE;
	MAN_EMIT := FALSE;
END_IF;

// ---------------------------------------------------------- ST5: the outfeed station
IF NOT RUNNING THEN
	ST5_STEP := 0;
ELSE
	CASE ST5_STEP OF
		200:	// the outfeed pin waits UP and EMPTY: here it is the robot that loads it. Its own
			// hold stop stands 250 mm DOWNSTREAM, armed, so finished parts leave one at a time
			// and nothing can back up under this pin.
			SOL_OUT_STOP := TRUE;
			SOL_OUT_DN := FALSE;
			SOL_OUT_UP := TRUE;
			SOL_OUT_HOLD := TRUE;
			OUT_CLAMP := FALSE;
			IF AS_OUT_UP AND AS_OUT_STOP_UP AND AS_OUT_HOLD_UP THEN
				ST5_STEP := 240;
			END_IF;
		240:
			IF OUT_GO AND PX_OUT THEN
				OUT_GO := FALSE;
				ST5_STEP := 250;
			END_IF;
		250:	// let go FIRST, so the part rides the pin down and the pin drops away under it
			OUT_CLAMP := FALSE;
			IF T_DROP.Q THEN
				ST5_STEP := 260;
			END_IF;
		260:	// the STOP goes down BEFORE the pin: with it still up the part comes down onto the
			// 30 mm stop head instead of the belt, stands on its rim and topples
			SOL_OUT_STOP := FALSE;
			IF AS_OUT_STOP_DN THEN
				ST5_STEP := 262;
			END_IF;
		262:
			SOL_OUT_UP := FALSE;
			SOL_OUT_DN := TRUE;
			IF AS_OUT_DN THEN
				ST5_STEP := 265;
			END_IF;
		265:	// it has left the station BEAM, but not the stop: the beam clears with the part
			// tail still over the stop head, and a head coming up under that tail tips the
			// part onto its side (measured: 12 -> 90 degrees, and it rode out lying down).
			// Wait T_OSTOP - 200 mm of belt - and only then arm the stop for the next one.
			IF NOT PE_OUT AND T_OSTOP.Q THEN
				SOL_OUT_STOP := TRUE;
				ST5_STEP := 266;
			END_IF;
		266:	// it runs on to the outfeed HOLD and stands there
			IF PE_OUT_HOLD THEN
				ST5_STEP := 268;
			END_IF;
		268:	// GO: away to the next process, one part at a time
			SOL_OUT_HOLD := FALSE;
			IF AS_OUT_HOLD_DN THEN
				ST5_STEP := 270;
			END_IF;
		270:	// the hold re-arms once the part is past its beam AND clear of the head itself -
			// the same trap one place further down the belt: fixing only the station stop
			// moved the tip-over from x 1675 to x 1455, where the hold came up under it
			IF NOT PE_OUT_HOLD AND T_OHOLD.Q THEN
				SOL_OUT_HOLD := TRUE;
				ST5_STEP := 275;
			END_IF;
		275:
			IF AS_OUT_HOLD_UP THEN
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
		J2_TGT := 23.49;
		J3_TGT := -31.11;
		J4_TGT := -89.58;
		J5_TGT := 86.84;
		J6_TGT := 0.00;
	2:	// chOutA
		J1_TGT := -79.65;
		J2_TGT := 24.94;
		J3_TGT := -33.19;
		J4_TGT := -91.50;
		J5_TGT := 100.23;
		J6_TGT := 0.00;
	3:	// doorA
		J1_TGT := -76.94;
		J2_TGT := -22.03;
		J3_TGT := 15.00;
		J4_TGT := -91.62;
		J5_TGT := 102.96;
		J6_TGT := 0.00;
	4:	// chAtB
		J1_TGT := -105.39;
		J2_TGT := 13.70;
		J3_TGT := -7.75;
		J4_TGT := -14.80;
		J5_TGT := -50.19;
		J6_TGT := 110.08;
	5:	// chOutB
		J1_TGT := -90.00;
		J2_TGT := 10.24;
		J3_TGT := -4.49;
		J4_TGT := -0.02;
		J5_TGT := -43.43;
		J6_TGT := 90.02;
	6:	// doorB
		J1_TGT := -103.29;
		J2_TGT := -33.05;
		J3_TGT := 33.10;
		J4_TGT := -9.66;
		J5_TGT := -54.59;
		J6_TGT := 106.37;
	7:	// inAt
		J1_TGT := 90.00;
		J2_TGT := -59.86;
		J3_TGT := 33.09;
		J4_TGT := 0.00;
		J5_TGT := -63.24;
		J6_TGT := 0.00;
	8:	// inUp
		J1_TGT := 90.00;
		J2_TGT := -69.59;
		J3_TGT := 53.30;
		J4_TGT := 0.00;
		J5_TGT := -73.72;
		J6_TGT := 0.00;
	9:	// outAt
		J1_TGT := 11.25;
		J2_TGT := -62.72;
		J3_TGT := 11.88;
		J4_TGT := 12.56;
		J5_TGT := 51.51;
		J6_TGT := -7.89;
	10:	// outUp
		J1_TGT := 14.57;
		J2_TGT := -77.36;
		J3_TGT := 33.45;
		J4_TGT := 13.79;
		J5_TGT := 44.76;
		J6_TGT := -9.89;
	11:	// home
		J1_TGT := -50.20;
		J2_TGT := 112.90;
		J3_TGT := 131.23;
		J4_TGT := 0.02;
		J5_TGT := 25.88;
		J6_TGT := 0.00;
END_CASE;
CASE GO_RAIL OF
	1:	RX_TGT := -880.00;
	2:	RX_TGT := 720.00;
	3:	RX_TGT := 2100.00;
	4:	RX_TGT := 1750.00;
END_CASE;
// END GENERATED

// TONs after the CASE: their Q is read by the NEXT scan.
T_GRIP(In := ST1_STEP = 15 OR ST1_STEP = 34 OR ST1_STEP = 44 OR ST1_STEP = 75, PT := T#400ms);
T_SETTLE(In := ST4_STEP = 235, PT := T#700ms);
T_DROP(In := ST5_STEP = 250, PT := T#200ms);
T_OSTOP(In := ST5_STEP = 265 AND NOT PE_OUT, PT := T#800ms);
T_OHOLD(In := ST5_STEP = 270 AND NOT PE_OUT_HOLD, PT := T#800ms);
T_QUE(In := ST6_STEP = 222, PT := T#400ms);
T_CUT1(In := M1_STATE = 1, PT := T#8s);
T_CUT2(In := M2_STATE = 1, PT := T#8s);

// Watchdog: a running step that stops moving is a jam, not patience. A station step that MOVES
// something is watched the same way; the ones that wait for the line (200, 210) and for the robot
// (240) are not jams, because the robot is 45 s away whenever it stands at the other machine
// through an 8 s cut. Each TON is called BEFORE its LAST copy is updated, so a step change gives
// it one scan of In = FALSE and resets it.
T_WD(In := ST1_STEP <> 0 AND ST1_STEP <> 900 AND ST1_STEP <> 910 AND ST1_STEP = STEP_LAST, PT := T#45s);
T_WD4(In := RUNNING AND ST4_STEP >= 210 AND ST4_STEP <> 240 AND ST4_STEP = STEP4_LAST, PT := T#45s);
T_WD5(In := RUNNING AND ST5_STEP >= 250 AND ST5_STEP = STEP5_LAST, PT := T#45s);
T_WD6(In := RUNNING AND ST6_STEP >= 212 AND ST6_STEP <> 215 AND ST6_STEP = STEP6_LAST, PT := T#45s);
STEP_LAST := ST1_STEP;
STEP4_LAST := ST4_STEP;
STEP5_LAST := ST5_STEP;
STEP6_LAST := ST6_STEP;
IF T_WD.Q OR T_WD4.Q OR T_WD5.Q OR T_WD6.Q OR (SEL_CHANGED AND RUNNING) THEN
	ST1_STEP := 900;
	J1_EXEC := FALSE;
	J2_EXEC := FALSE;
	J3_EXEC := FALSE;
	J4_EXEC := FALSE;
	J5_EXEC := FALSE;
	J6_EXEC := FALSE;
	RX_EXEC := FALSE;
	EM_IN_EMIT := FALSE;
	CVIN_RUN := FALSE;
	CVOUT_RUN := FALSE;
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
	CVIN_RUN := FALSE;
	CVOUT_RUN := FALSE;
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
	IF IND_LIFT_IN AND NOT IND_LI_LAST THEN
		MEM_LI := NOT MEM_LI;
	END_IF;
	IF IND_LIFT_OUT AND NOT IND_LO_LAST THEN
		MEM_LO := NOT MEM_LO;
	END_IF;
	IF IND_CV_IN AND NOT IND_CVI_LAST THEN
		MEM_CVI := NOT MEM_CVI;
	END_IF;
	IF IND_CV_OUT AND NOT IND_CVO_LAST THEN
		MEM_CVO := NOT MEM_CVO;
	END_IF;
	IF IND_STOP_IN AND NOT IND_SI_LAST THEN
		MEM_SI := NOT MEM_SI;
	END_IF;
	IF IND_HOLD_IN AND NOT IND_HI_LAST THEN
		MEM_HI := NOT MEM_HI;
	END_IF;
	IF IND_QUE_IN AND NOT IND_QI_LAST THEN
		MEM_QI := NOT MEM_QI;
	END_IF;
	IF IND_BLADE_IN AND NOT IND_BL_LAST THEN
		MEM_BL := NOT MEM_BL;
	END_IF;
	IF IND_STOP_OUT AND NOT IND_SO_LAST THEN
		MEM_SO := NOT MEM_SO;
	END_IF;
	IF IND_HOLD_OUT AND NOT IND_HO_LAST THEN
		MEM_HO := NOT MEM_HO;
	END_IF;
	GRIP_A := MEM_A;
	GRIP_B := MEM_B;
	SOL_M1_DOOR := MEM_D1;
	SOL_M2_DOOR := MEM_D2;
	SOL_IN_UP := MEM_LI;
	SOL_IN_DN := NOT MEM_LI;
	SOL_OUT_UP := MEM_LO;
	SOL_OUT_DN := NOT MEM_LO;
	// One press, one casting: the button is momentary and the feeder answers its RISING edge,
	// so holding it down does not pour castings onto the belt. Only here - in AUTO, ST4 owns it.
	EM_IN_EMIT := IND_FEED;
	CVIN_RUN := MEM_CVI;
	CVOUT_RUN := MEM_CVO;
	// Every stop has its own button, and every stop is ARMED unless its button says otherwise: a
	// casting let loose down a belt with the stops dropped runs off the end and out of the world,
	// and there is no discharge at the infeed end to catch it. So each button DROPS its stop. The
	// blade is the other way round - out is where it lets the line run.
	SOL_IN_STOP := NOT MEM_SI;
	SOL_IN_HOLD := NOT MEM_HI;
	SOL_IN_QUE := NOT MEM_QI;
	SOL_IN_BLADE := MEM_BL;
	SOL_OUT_STOP := NOT MEM_SO;
	SOL_OUT_HOLD := NOT MEM_HO;
	// Castings fed by hand stack up at the hold, touching. Going back to AUTO with more than one
	// of them there faults the cell at step 225 after the watchdog: the hold cannot rise through
	// a queue. That is the escapement reporting a jam - clear the belt before running AUTO.
ELSE
	MEM_A := FALSE;
	MEM_B := FALSE;
	MEM_D1 := FALSE;
	MEM_D2 := FALSE;
	MEM_LI := FALSE;
	MEM_LO := FALSE;
	MEM_CVI := FALSE;
	MEM_CVO := FALSE;
	MEM_SI := FALSE;
	MEM_HI := FALSE;
	MEM_QI := FALSE;
	MEM_BL := FALSE;
	MEM_SO := FALSE;
	MEM_HO := FALSE;
END_IF;
IND_A_LAST := IND_A;
IND_B_LAST := IND_B;
IND_D1_LAST := IND_DOOR1;
IND_D2_LAST := IND_DOOR2;
IND_LI_LAST := IND_LIFT_IN;
IND_LO_LAST := IND_LIFT_OUT;
IND_CVI_LAST := IND_CV_IN;
IND_CVO_LAST := IND_CV_OUT;
IND_SI_LAST := IND_STOP_IN;
IND_HI_LAST := IND_HOLD_IN;
IND_QI_LAST := IND_QUE_IN;
IND_BL_LAST := IND_BLADE_IN;
IND_SO_LAST := IND_STOP_OUT;
IND_HO_LAST := IND_HOLD_OUT;
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
