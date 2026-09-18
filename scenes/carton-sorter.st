VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	T_DIV_A : TON;
	T_DIV_B : TON;
	T_WD2 : TON;
	T_WD3 : TON;
	STEP2_LAST : UDINT;
	STEP3_LAST : UDINT;
	RUNNING : BOOL;
	SCAN_EDGE : BOOL;
	PE_IN_LAST : BOOL;
	UP_A_EDGE : BOOL;
	PE_UP_A_LAST : BOOL;
	UP_B_EDGE : BOOL;
	PE_UP_B_LAST : BOOL;
	NEXT_DEST : UINT;
	DEST : UINT;
	QA : ARRAY[0..15] OF UINT;
	QA_H : UINT;
	QA_T : UINT;
	QA_N : UINT;
	QB : ARRAY[0..15] OF UINT;
	QB_H : UINT;
	QB_T : UINT;
	QB_N : UINT;
	MEM_CV : BOOL;
	MEM_FEED : BOOL;
	MEM_SPA : BOOL;
	MEM_SPB : BOOL;
	IND_CV_LAST : BOOL;
	IND_FEED_LAST : BOOL;
	IND_SPA_LAST : BOOL;
	IND_SPB_LAST : BOOL;
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
// carton-sorter: a sortation line on the Open Industry Project's own numbers - a 1.524 m belt at
// 2 m/s, cartons 600 x 400 x 400 at 10 kg fed 45 a minute, and two swing-blade diverters whose
// 2.127 m blade sweeps 1504 mm across the belt at 45 degrees, the belt itself driving the carton
// along the blade and onto a take-away at 1 m/s. The same sequence runs without a PLC in
// carton-sorter.ctl.js - keep the two in step.
//
// TRACKING is the whole job here, and it is a SHIFT REGISTER, not a timer: QA holds the
// destinations of the cartons between the scanner and blade A, QB those between A and B. A
// carton arriving at a blade pops the queue; one that is not for that blade is pushed onto the
// next one's. Timing the belt instead would mean a jam, a hand taking a carton off the line, or a
// change of speed override silently sorting the wrong box - a queue moves only when a beam says
// a carton actually went past.
//
// Destinations: 0 = take-away A, 1 = take-away B, 2 = straight through (a no-read).

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

SCAN_EDGE := PE_IN AND NOT PE_IN_LAST;
UP_A_EDGE := PE_UP_A AND NOT PE_UP_A_LAST;
UP_B_EDGE := PE_UP_B AND NOT PE_UP_B_LAST;

CASE ST1_STEP OF
	0:	// idle
		DVA_EXEC := FALSE;
		DVB_EXEC := FALSE;
		CV_RUN := FALSE;
		SPA_RUN := FALSE;
		SPB_RUN := FALSE;
		EM_EN := FALSE;
		QA_H := 0;
		QA_T := 0;
		QA_N := 0;
		QB_H := 0;
		QB_T := 0;
		QB_N := 0;
		NEXT_DEST := 0;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED THEN
			STOP_REQ := FALSE;
			ST2_STEP := 100;
			ST3_STEP := 100;
			ST1_STEP := 10;
		END_IF;
	10:	// the line runs and the scanner reads every carton that goes by
		CV_RUN := TRUE;
		SPA_RUN := TRUE;
		SPB_RUN := TRUE;
		EM_EN := TRUE;
		IF SCAN_EDGE THEN
			IF QA_N >= 16 THEN
				ST1_STEP := 900;
			ELSE
				QA[QA_T] := NEXT_DEST;
				QA_T := (QA_T + 1) MOD 16;
				QA_N := QA_N + 1;
				NEXT_DEST := (NEXT_DEST + 1) MOD 3;
				CYCLE_CNT := CYCLE_CNT + 1;
			END_IF;
		END_IF;
		IF STOP_REQ AND QA_N = 0 AND QB_N = 0 THEN
			EM_EN := FALSE;
			ST1_STEP := 0;
		END_IF;
	800:	// HOME: the blades parked along the line, out of the path
		CV_RUN := FALSE;
		SPA_RUN := FALSE;
		SPB_RUN := FALSE;
		EM_EN := FALSE;
		DVA_TGT := 0.0;
		DVB_TGT := 0.0;
		DVA_EXEC := TRUE;
		DVB_EXEC := TRUE;
		IF DVA_DONE AND DVB_DONE THEN
			DVA_EXEC := FALSE;
			DVB_EXEC := FALSE;
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	900:	// stuck, a runaway queue, or the selector was turned while running
		DVA_EXEC := FALSE;
		DVB_EXEC := FALSE;
		CV_RUN := FALSE;
		SPA_RUN := FALSE;
		SPB_RUN := FALSE;
		EM_EN := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
	910:	// the master circuit is open
		DVA_EXEC := FALSE;
		DVB_EXEC := FALSE;
		CV_RUN := FALSE;
		SPA_RUN := FALSE;
		SPB_RUN := FALSE;
		EM_EN := FALSE;
		IF READY THEN
			ST1_STEP := 0;
		END_IF;
END_CASE;

PE_IN_LAST := PE_IN;
RUNNING := ST1_STEP = 10;

// ---------------------------------------------------------- ST2: diverter A
IF NOT RUNNING THEN
	ST2_STEP := 0;
ELSE
	CASE ST2_STEP OF
		100:	// waiting for a carton; the queue says what this one is for
			IF UP_A_EDGE THEN
				IF QA_N > 0 THEN
					DEST := QA[QA_H];
					QA_H := (QA_H + 1) MOD 16;
					QA_N := QA_N - 1;
				ELSE
					DEST := 2;
				END_IF;
				IF DEST = 0 THEN
					ST2_STEP := 110;
				ELSE
					IF QB_N < 16 THEN
						QB[QB_T] := DEST;
						QB_T := (QB_T + 1) MOD 16;
						QB_N := QB_N + 1;
					END_IF;
					ST2_STEP := 130;
				END_IF;
			END_IF;
		110:	// the blade out, across the belt
			DVA_TGT := -45.0;
			DVA_EXEC := TRUE;
			IF DVA_DONE THEN
				ST2_STEP := 111;
			END_IF;
		111:
			DVA_EXEC := FALSE;
			IF NOT DVA_DONE THEN
				ST2_STEP := 112;
			END_IF;
		112:	// the carton rides the blade; the take-away beam says it arrived
			IF PE_A THEN
				ST2_STEP := 113;
			ELSIF T_DIV_A.Q THEN
				ST2_STEP := 900;
			END_IF;
		113:	// and it is CLEAR when that beam lets go again
			IF NOT PE_A THEN
				ST2_STEP := 120;
			END_IF;
		120:
			DVA_TGT := 0.0;
			DVA_EXEC := TRUE;
			IF DVA_DONE THEN
				ST2_STEP := 121;
			END_IF;
		121:
			DVA_EXEC := FALSE;
			IF NOT DVA_DONE THEN
				ST2_STEP := 100;
			END_IF;
		130:	// not for A: let it run on to B
			IF NOT PE_UP_A THEN
				ST2_STEP := 100;
			END_IF;
	ELSE
		ST2_STEP := 100;
	END_CASE;
END_IF;

// ---------------------------------------------------------- ST3: diverter B
IF NOT RUNNING THEN
	ST3_STEP := 0;
ELSE
	CASE ST3_STEP OF
		100:
			IF UP_B_EDGE THEN
				IF QB_N > 0 THEN
					DEST := QB[QB_H];
					QB_H := (QB_H + 1) MOD 16;
					QB_N := QB_N - 1;
				ELSE
					DEST := 2;
				END_IF;
				IF DEST = 1 THEN
					ST3_STEP := 110;
				ELSE
					ST3_STEP := 130;
				END_IF;
			END_IF;
		110:
			DVB_TGT := 45.0;
			DVB_EXEC := TRUE;
			IF DVB_DONE THEN
				ST3_STEP := 111;
			END_IF;
		111:
			DVB_EXEC := FALSE;
			IF NOT DVB_DONE THEN
				ST3_STEP := 112;
			END_IF;
		112:
			IF PE_B THEN
				ST3_STEP := 113;
			ELSIF T_DIV_B.Q THEN
				ST3_STEP := 900;
			END_IF;
		113:
			IF NOT PE_B THEN
				ST3_STEP := 120;
			END_IF;
		120:
			DVB_TGT := 0.0;
			DVB_EXEC := TRUE;
			IF DVB_DONE THEN
				ST3_STEP := 121;
			END_IF;
		121:
			DVB_EXEC := FALSE;
			IF NOT DVB_DONE THEN
				ST3_STEP := 100;
			END_IF;
		130:
			IF NOT PE_UP_B THEN
				ST3_STEP := 100;
			END_IF;
	ELSE
		ST3_STEP := 100;
	END_CASE;
END_IF;

PE_UP_A_LAST := PE_UP_A;
PE_UP_B_LAST := PE_UP_B;

// TONs after the CASE: their Q is read by the NEXT scan. A blade that is out and never sees its
// carton reach the take-away has lost it: that is a fault, not patience.
T_DIV_A(In := ST2_STEP = 112, PT := T#6s);
T_DIV_B(In := ST3_STEP = 112, PT := T#6s);

// A diverter step that stops moving is a jam. Its waiting step (100) is not: on a quiet line the
// next carton is as far away as the feed rate says. The TONs are called before the LAST copies
// are updated, so a step change gives them one scan of In = FALSE and resets them.
T_WD2(In := RUNNING AND ST2_STEP <> 100 AND ST2_STEP = STEP2_LAST, PT := T#20s);
T_WD3(In := RUNNING AND ST3_STEP <> 100 AND ST3_STEP = STEP3_LAST, PT := T#20s);
STEP2_LAST := ST2_STEP;
STEP3_LAST := ST3_STEP;
IF T_WD2.Q OR T_WD3.Q OR ST2_STEP = 900 OR ST3_STEP = 900 OR (SEL_CHANGED AND RUNNING) THEN
	ST1_STEP := 900;
	DVA_EXEC := FALSE;
	DVB_EXEC := FALSE;
	CV_RUN := FALSE;
	SPA_RUN := FALSE;
	SPB_RUN := FALSE;
	EM_EN := FALSE;
END_IF;

IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	DVA_EXEC := FALSE;
	DVB_EXEC := FALSE;
	CV_RUN := FALSE;
	SPA_RUN := FALSE;
	SPB_RUN := FALSE;
	EM_EN := FALSE;
ELSIF HOME_EDGE AND READY AND ST1_STEP = 0 THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := ST1_STEP <> 0 AND ST1_STEP <> 800 AND ST1_STEP <> 900 AND ST1_STEP <> 910;
IND_ACTIVE := NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND ST1_STEP <> 800;
IF IND_ACTIVE THEN
	IF IND_CV AND NOT IND_CV_LAST THEN
		MEM_CV := NOT MEM_CV;
	END_IF;
	IF IND_FEED AND NOT IND_FEED_LAST THEN
		MEM_FEED := NOT MEM_FEED;
	END_IF;
	IF IND_SPA AND NOT IND_SPA_LAST THEN
		MEM_SPA := NOT MEM_SPA;
	END_IF;
	IF IND_SPB AND NOT IND_SPB_LAST THEN
		MEM_SPB := NOT MEM_SPB;
	END_IF;
	CV_RUN := MEM_CV;
	EM_EN := MEM_FEED;
	SPA_RUN := MEM_SPA;
	SPB_RUN := MEM_SPB;
ELSE
	MEM_CV := FALSE;
	MEM_FEED := FALSE;
	MEM_SPA := FALSE;
	MEM_SPB := FALSE;
END_IF;
IND_CV_LAST := IND_CV;
IND_FEED_LAST := IND_FEED;
IND_SPA_LAST := IND_SPA;
IND_SPB_LAST := IND_SPB;
DVA_JOG_P := IND_ACTIVE AND IND_DVA_P;
DVA_JOG_N := IND_ACTIVE AND IND_DVA_N;
DVB_JOG_P := IND_ACTIVE AND IND_DVB_P;
DVB_JOG_N := IND_ACTIVE AND IND_DVB_N;

OVR := OVR_SET;
PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
