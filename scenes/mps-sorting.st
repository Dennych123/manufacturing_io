VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	T_WD : TON;
	T_READ : TON;
	T_FEED : TON;
	STEP_LAST : UDINT;
	KIND : UINT;
	NEXT_FEED : UINT;
	CLEAR : BOOL;
	RUNNING : BOOL;
	MEM_BELT : BOOL;
	MEM_STOP : BOOL;
	MEM_W1 : BOOL;
	MEM_W2 : BOOL;
	IND_BELT_LAST : BOOL;
	IND_STOP_LAST : BOOL;
	IND_W1_LAST : BOOL;
	IND_W2_LAST : BOOL;
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
// mps-sorting: Festo Didactic's MPS Sorting station 8046325 (manual 8046391, 11/2015). The
// sequence and the IO are the manual's: a 40 mm workpiece is laid on the belt, a fork light
// barrier sees it, the belt runs it up to a stop, and three sensors at the stop say what it is -
// a through-beam sees every workpiece, a retro-reflective one sees every one that is NOT matt
// black, and an inductive one sees the metal. So black = seen but not by the retro one, red =
// retro but not inductive, metallic = both. Red goes to the chute at the belt START (Weiche 1),
// metallic to the one in the MIDDLE (Weiche 2), black runs off the belt END.
// The same sequence runs without a PLC in mps-sorting.ctl.js - keep the two in step.
//
// ONE difference from the real machine, deliberate: Festo's Q3 is "Stopper einfahren" - the stop
// is out by default and the output retracts it. Here the stop pops UP out of the belt
// (STOP_HOLD, energised to hold), because in this simulator a stop a part passes UNDER holds it
// from a positive distance (CLAUDE.md). It is the same signal inverted.
//
// KIND: 0 = black, 1 = red, 2 = metallic.

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

// Everything laid on the belt has reached a bin: the invariant a discharge step waits for. A
// count that "has moved" cannot say WHICH workpiece moved (CLAUDE.md).
CLEAR := (EM_BLACK_CNT + EM_RED_CNT + EM_METAL_CNT) = (RM_1_CNT + RM_2_CNT + RM_3_CNT);

CASE ST1_STEP OF
	0:	// Ausgangsstellung: belt off, stop out, both gates in
		BELT_FWD := FALSE;
		W1_EXT := FALSE;
		W2_EXT := FALSE;
		EM_BLACK := FALSE;
		EM_RED := FALSE;
		EM_METAL := FALSE;
		STOP_HOLD := TRUE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED THEN
			STOP_REQ := FALSE;
			ST1_STEP := 10;
		END_IF;
	10:	// the operator lays the next workpiece on the belt, one colour at a time
		BELT_FWD := FALSE;
		STOP_HOLD := TRUE;
		EM_BLACK := NEXT_FEED = 0;
		EM_RED := NEXT_FEED = 1;
		EM_METAL := NEXT_FEED = 2;
		IF T_FEED.Q THEN
			NEXT_FEED := (NEXT_FEED + 1) MOD 3;
			ST1_STEP := 15;
		END_IF;
	15:	// Startvoraussetzung: Werkstück am Bandanfang
		EM_BLACK := FALSE;
		EM_RED := FALSE;
		EM_METAL := FALSE;
		BELT_FWD := FALSE;
		STOP_HOLD := TRUE;
		IF WP_AT_START AND NOT CHUTE_FULL THEN
			ST1_STEP := 20;
		END_IF;
	20:	// Bandmotor ein
		BELT_FWD := TRUE;
		STOP_HOLD := TRUE;
		IF WP_DETECTED THEN
			ST1_STEP := 30;
		END_IF;
	30:	// Farb-/Materialidentifikation, with the workpiece held against the stop
		BELT_FWD := TRUE;
		IF T_READ.Q THEN
			IF WP_METALLIC THEN
				KIND := 2;
				ST1_STEP := 50;
			ELSIF WP_NOT_BLACK THEN
				KIND := 1;
				ST1_STEP := 40;
			ELSE
				KIND := 0;
				ST1_STEP := 60;
			END_IF;
		END_IF;
	40:	// red: Weiche 1 ausfahren
		BELT_FWD := TRUE;
		W1_EXT := TRUE;
		IF W1_OUT THEN
			ST1_STEP := 41;
		END_IF;
	41:	// Sperre einfahren
		BELT_FWD := TRUE;
		STOP_HOLD := FALSE;
		IF AS_STOP_FREE THEN
			ST1_STEP := 42;
		END_IF;
	42:	// Werkstück ausgeschleust
		BELT_FWD := TRUE;
		IF CLEAR THEN
			ST1_STEP := 43;
		END_IF;
	43:	// Weiche 1 einfahren
		BELT_FWD := TRUE;
		W1_EXT := FALSE;
		IF AS_W1_IN THEN
			ST1_STEP := 70;
		END_IF;
	50:	// metallic: Weiche 2 ausfahren
		BELT_FWD := TRUE;
		W2_EXT := TRUE;
		IF W2_OUT THEN
			ST1_STEP := 51;
		END_IF;
	51:
		BELT_FWD := TRUE;
		STOP_HOLD := FALSE;
		IF AS_STOP_FREE THEN
			ST1_STEP := 52;
		END_IF;
	52:
		BELT_FWD := TRUE;
		IF CLEAR THEN
			ST1_STEP := 53;
		END_IF;
	53:
		BELT_FWD := TRUE;
		W2_EXT := FALSE;
		IF AS_W2_IN THEN
			ST1_STEP := 70;
		END_IF;
	60:	// black: let it go, it runs off the belt end onto the third chute
		BELT_FWD := TRUE;
		STOP_HOLD := FALSE;
		IF AS_STOP_FREE THEN
			ST1_STEP := 61;
		END_IF;
	61:
		BELT_FWD := TRUE;
		IF CLEAR THEN
			ST1_STEP := 70;
		END_IF;
	70:	// Bandmotor aus, Sperre ausfahren
		BELT_FWD := FALSE;
		STOP_HOLD := TRUE;
		IF AS_STOP_HOLD THEN
			CYCLE_CNT := CYCLE_CNT + 1;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSE
				ST1_STEP := 10;
			END_IF;
		END_IF;
	800:	// HOME is the station's Ausgangsstellung
		BELT_FWD := FALSE;
		W1_EXT := FALSE;
		W2_EXT := FALSE;
		EM_BLACK := FALSE;
		EM_RED := FALSE;
		EM_METAL := FALSE;
		STOP_HOLD := TRUE;
		IF AS_STOP_HOLD AND AS_W1_IN AND AS_W2_IN THEN
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	900:	// stuck, or the selector was turned while running
		BELT_FWD := FALSE;
		W1_EXT := FALSE;
		W2_EXT := FALSE;
		EM_BLACK := FALSE;
		EM_RED := FALSE;
		EM_METAL := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
	910:	// the master circuit is open: the stop KEEPS the workpiece it is holding
		BELT_FWD := FALSE;
		W1_EXT := FALSE;
		W2_EXT := FALSE;
		EM_BLACK := FALSE;
		EM_RED := FALSE;
		EM_METAL := FALSE;
		IF READY THEN
			ST1_STEP := 0;
		END_IF;
END_CASE;

RUNNING := ST1_STEP <> 0 AND ST1_STEP <> 800 AND ST1_STEP <> 900 AND ST1_STEP <> 910;

// TONs after the CASE: their Q is read by the NEXT scan.
T_READ(In := ST1_STEP = 30, PT := T#400ms);
T_FEED(In := ST1_STEP = 10, PT := T#600ms);

// Watchdog: a running step that stops moving is a jam, not patience - a workpiece taken off the
// belt by hand leaves step 20 waiting for a stop it will never reach. The TON is called BEFORE
// STEP_LAST is updated, so a step change gives it one scan of In = FALSE and resets it.
T_WD(In := ST1_STEP <> 0 AND ST1_STEP <> 900 AND ST1_STEP <> 910 AND ST1_STEP = STEP_LAST, PT := T#20s);
STEP_LAST := ST1_STEP;
IF T_WD.Q OR (SEL_CHANGED AND RUNNING) THEN
	ST1_STEP := 900;
	BELT_FWD := FALSE;
	W1_EXT := FALSE;
	W2_EXT := FALSE;
	EM_BLACK := FALSE;
	EM_RED := FALSE;
	EM_METAL := FALSE;
END_IF;

IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	BELT_FWD := FALSE;
	W1_EXT := FALSE;
	W2_EXT := FALSE;
	EM_BLACK := FALSE;
	EM_RED := FALSE;
	EM_METAL := FALSE;
ELSIF HOME_EDGE AND READY AND ST1_STEP = 0 THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := ST1_STEP <> 0 AND ST1_STEP <> 800 AND ST1_STEP <> 900 AND ST1_STEP <> 910;
IND_ACTIVE := NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND ST1_STEP <> 800;
IF IND_ACTIVE THEN
	IF IND_BELT AND NOT IND_BELT_LAST THEN
		MEM_BELT := NOT MEM_BELT;
	END_IF;
	IF IND_STOP AND NOT IND_STOP_LAST THEN
		MEM_STOP := NOT MEM_STOP;
	END_IF;
	IF IND_W1 AND NOT IND_W1_LAST THEN
		MEM_W1 := NOT MEM_W1;
	END_IF;
	IF IND_W2 AND NOT IND_W2_LAST THEN
		MEM_W2 := NOT MEM_W2;
	END_IF;
	BELT_FWD := MEM_BELT;
	STOP_HOLD := MEM_STOP;
	W1_EXT := MEM_W1;
	W2_EXT := MEM_W2;
	EM_BLACK := IND_FEED_BLACK;
	EM_RED := IND_FEED_RED;
	EM_METAL := IND_FEED_METAL;
ELSE
	MEM_BELT := FALSE;
	MEM_STOP := FALSE;
	MEM_W1 := FALSE;
	MEM_W2 := FALSE;
END_IF;
IND_BELT_LAST := IND_BELT;
IND_STOP_LAST := IND_STOP;
IND_W1_LAST := IND_W1;
IND_W2_LAST := IND_W2;

OVR := OVR_SET;
PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
