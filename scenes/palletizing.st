VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
	RUNNING : BOOL;
	LOAD_BUSY : BOOL;
	UNLOAD_BUSY : BOOL;
	LOAD_IDX : INT;
	UNLOAD_IDX : INT;
	FULL : ARRAY[0..3] OF BOOL;
	HALF : BOOL;
	ROW_Y : LREAL := 330.0;
	PICKED : BOOL;
	ANY_VAC : BOOL;
	NO_VAC : BOOL;
	NO_UVAC : BOOL;
	NEED_FILL : BOOL := TRUE;
	PAL_BASE : UDINT;
	NEXT_ST : INT;
	T_FILL : TON;
	T_PICK : TON;
	T_UPICK : TON;
	T_WD1 : TON;
	T_WD2 : TON;
	T_WD3 : TON;
	STEP1_LAST : UDINT;
	STEP2_LAST : UDINT;
	STEP3_LAST : UDINT;
	JAM : BOOL;
	MEM_VAC : BOOL;
	IND_VAC_LAST : BOOL;
	MEM_LIFT : BOOL;
	IND_LIFT_LAST : BOOL;
	MEM_PAL : BOOL;
	IND_PAL_LAST : BOOL;
	MEM_JIG : BOOL;
	IND_JIG_LAST : BOOL;
	MEM_UVAC : BOOL;
	IND_UVAC_LAST : BOOL;
	MEM_ULIFT : BOOL;
	IND_ULIFT_LAST : BOOL;
	MEM_USHIFT : BOOL;
	IND_USHIFT_LAST : BOOL;
	INDIVIDUAL : BOOL;
END_VAR
// palletizing: a 10 x 10 pallet of spark plugs, a five-up vacuum gantry on two servo axes, a
// rotary carrier of five-slot jigs, and an unload head that hands the plugs to the next process.
// The same sequence runs without a PLC in palletizing.ctl.js.
//
// THREE stations run at the same time, which is what a cell does and what one long sequence could
// not: the gantry goes back for the next five as soon as it has put its five down, while the
// carrier indexes and the unloader empties a jig on the far side. They meet at the carrier: FULL[i]
// says which jig has parts in it, and neither head lets the table turn under it.
//
// A head takes what is there. A hole may be empty - a plug was lifted out, or the pallet is
// finished - and the gantry carries on with what it got rather than waiting for a plug that is not
// coming. Nothing at all means the tray is empty, and a fresh pallet is called for.
//
// Gantry positions are millimetres on each axis, measured from the scene: cup 3 sits at
// X = GX - 300 and Y = GY - 600, so the pallet halves are at GX 150 and 450, the jig at GX 1268,
// and pallet row r is at GY 330 + 60 r.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_CSTOP THEN
	STOP_REQ := TRUE;
END_IF;
SEL_CHANGED := SEL_AUTO <> SEL_LAST;
SEL_LAST := SEL_AUTO;

// The master circuit: E-STOP is a latching mushroom, MASTER ON energises the machine, and an
// E-STOP also loses the home position.
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

ANY_VAC := VAC_SW1 OR VAC_SW2 OR VAC_SW3 OR VAC_SW4 OR VAC_SW5;
NO_VAC := NOT ANY_VAC;
NO_UVAC := NOT (UVAC_SW1 OR UVAC_SW2 OR UVAC_SW3 OR UVAC_SW4 OR UVAC_SW5);
// The jig standing at the load station, and the one at the unload station.
LOAD_IDX := (2 - TBL_STATION + 4) MOD 4;
UNLOAD_IDX := (4 - TBL_STATION) MOD 4;
// Neither head lets the carrier turn while it is over a jig.
LOAD_BUSY := (ST1_STEP >= 57) AND (ST1_STEP <= 80);
UNLOAD_BUSY := (ST3_STEP >= 110) AND (ST3_STEP <= 130);

// ---------------------------------------------------------------- ST1: the depalletiser
CASE ST1_STEP OF
	0:	// idle: head up, vacuum off
		VAC_ON := FALSE;
		GX_EXEC := FALSE;
		GY_EXEC := FALSE;
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		EM_PAL_EN := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_Z_UP AND AS_U_UP THEN
			STOP_REQ := FALSE;
			ST2_STEP := 10;
			ST3_STEP := 100;
			IF NEED_FILL THEN
				PAL_BASE := EM_PAL_CNT;
				ST1_STEP := 5;
			ELSE
				ST1_STEP := 10;
			END_IF;
		END_IF;
	5:	// A fresh pallet. The tray is FULL when the loader stops delivering, not after a count of
		// 100: a hole that still had a plug in it would make a count wait for ever.
		PAL_CLAMP := TRUE;
		EM_PAL_EN := TRUE;
		IF EM_PAL_CNT <> PAL_BASE THEN
			PAL_BASE := EM_PAL_CNT;
		ELSIF T_FILL.Q THEN
			EM_PAL_EN := FALSE;
			NEED_FILL := FALSE;
			ST1_STEP := 10;
		END_IF;
	10:	// both gantry axes move at once; Execute is a LEVEL held until Done
		PAL_CLAMP := TRUE;
		JIG_CLAMP := TRUE;
		IF HALF THEN
			GX_TGT := 450.0;
		ELSE
			GX_TGT := 150.0;
		END_IF;
		GY_TGT := ROW_Y;
		GX_EXEC := TRUE;
		GY_EXEC := TRUE;
		IF GX_DONE AND GY_DONE THEN
			GX_EXEC := FALSE;
			GY_EXEC := FALSE;
			ST1_STEP := 15;
		END_IF;
	15:	// wait until the plant has seen Execute drop
		IF NOT GX_DONE AND NOT GY_DONE THEN
			ST1_STEP := 20;
		END_IF;
	20:	// head down onto the row
		SOL_Z_UP := FALSE;
		SOL_Z_DN := TRUE;
		IF AS_Z_DN THEN
			ST1_STEP := 30;
		END_IF;
	30:	// the tray lets go while the cups take whatever is there
		PAL_CLAMP := FALSE;
		VAC_ON := TRUE;
		IF T_PICK.Q THEN
			PICKED := ANY_VAC;
			ST1_STEP := 40;
		END_IF;
	40:	// the tray is held again, and the head lifts with what it got
		PAL_CLAMP := TRUE;
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		IF AS_Z_UP THEN
			IF PICKED THEN
				ST1_STEP := 50;
			ELSE
				// nothing at all came up: the tray is empty
				NEED_FILL := TRUE;
				PAL_BASE := EM_PAL_CNT;
				ST1_STEP := 5;
			END_IF;
		END_IF;
	50:	// across to the jig at the load station
		GX_TGT := 1268.0;
		GY_TGT := 600.0;
		GX_EXEC := TRUE;
		GY_EXEC := TRUE;
		IF GX_DONE AND GY_DONE THEN
			GX_EXEC := FALSE;
			GY_EXEC := FALSE;
			ST1_STEP := 55;
		END_IF;
	55:	// wait until the plant has seen Execute drop
		IF NOT GX_DONE AND NOT GY_DONE THEN
			ST1_STEP := 57;
		END_IF;
	57:	// the load station must be standing still with an empty jig under it
		IF TBL_INPOS AND NOT FULL[LOAD_IDX] THEN
			ST1_STEP := 60;
		END_IF;
	60:	// head down over the jig
		SOL_Z_UP := FALSE;
		SOL_Z_DN := TRUE;
		IF AS_Z_DN THEN
			ST1_STEP := 70;
		END_IF;
	70:	// the jig nests take the plugs as the cups let go: one part, one holder
		VAC_ON := FALSE;
		IF NO_VAC THEN
			ST1_STEP := 80;
		END_IF;
	80:	// head up; the carrier may take this jig away now
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		IF AS_Z_UP THEN
			FULL[LOAD_IDX] := TRUE;
			CYCLE_CNT := CYCLE_CNT + 1;
			IF HALF THEN
				HALF := FALSE;
				IF ROW_Y >= 870.0 THEN
					ROW_Y := 330.0;
					NEED_FILL := TRUE;
				ELSE
					ROW_Y := ROW_Y + 60.0;
				END_IF;
			ELSE
				HALF := TRUE;
			END_IF;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSIF NEED_FILL THEN
				PAL_BASE := EM_PAL_CNT;
				ST1_STEP := 5;
			ELSE
				ST1_STEP := 10;
			END_IF;
		END_IF;
	800:	// HOME: heads up, unload shift over the jig, both gantry axes to zero, carrier to its
		// origin. The vacuum is left alone: a head keeps the plugs it is holding.
		TBL_RUN := NOT TBL_ORIGIN;
		SOL_Z_DN := FALSE;
		SOL_Z_UP := TRUE;
		SOL_U_DN := FALSE;
		SOL_U_UP := TRUE;
		SOL_U_BIN := FALSE;
		SOL_U_JIG := TRUE;
		GX_TGT := 0.0;
		GY_TGT := 0.0;
		GX_EXEC := TRUE;
		GY_EXEC := TRUE;
		IF AS_Z_UP AND AS_U_UP AND AS_U_JIG AND GX_DONE AND GY_DONE AND TBL_ORIGIN THEN
			GX_EXEC := FALSE;
			GY_EXEC := FALSE;
			TBL_RUN := FALSE;
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: stuck, or the selector was turned while running. Motion and feeding off; the
		// trays and the cups keep whatever they hold.
		TBL_RUN := FALSE;
		GX_EXEC := FALSE;
		GY_EXEC := FALSE;
		EM_PAL_EN := FALSE;
		PAL_CLAMP := TRUE;
		JIG_CLAMP := TRUE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - which is what
		// really happens: a 5/2 single-solenoid valve springs back, a double one holds. The trays
		// and the cups KEEP their parts: a hundred plugs let loose in their pockets is not what a
		// power cut does, and it costs the plant four times the work.
		TBL_RUN := FALSE;
		GX_EXEC := FALSE;
		GY_EXEC := FALSE;
		EM_PAL_EN := FALSE;
		SOL_Z_DN := FALSE;
		SOL_Z_UP := FALSE;
		SOL_U_DN := FALSE;
		SOL_U_UP := FALSE;
		SOL_U_BIN := FALSE;
		SOL_U_JIG := FALSE;
		PAL_CLAMP := TRUE;
		JIG_CLAMP := TRUE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
END_CASE;

RUNNING := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// ---------------------------------------------------------------- ST2: the jig carrier
IF NOT RUNNING THEN
	ST2_STEP := 0;
	TBL_RUN := FALSE;
ELSE
	CASE ST2_STEP OF
		10:	// Turn when there is a full jig to move on AND an empty one to bring to the unloader,
			// and never while a head is over the table.
			TBL_RUN := FALSE;
			IF FULL[LOAD_IDX] AND NOT FULL[UNLOAD_IDX] AND NOT LOAD_BUSY AND NOT UNLOAD_BUSY AND TBL_INPOS THEN
				NEXT_ST := (TBL_STATION + 1) MOD 4;
				ST2_STEP := 20;
			END_IF;
		20:	// the invariant: this index has landed on the station it was sent to
			TBL_RUN := TRUE;
			IF TBL_INPOS AND (TBL_STATION = NEXT_ST) THEN
				TBL_RUN := FALSE;
				ST2_STEP := 10;
			END_IF;
		ELSE
			ST2_STEP := 10;
	END_CASE;
END_IF;

// ---------------------------------------------------------------- ST3: the unloader
IF NOT RUNNING THEN
	ST3_STEP := 0;
ELSE
	CASE ST3_STEP OF
		100:	// wait for a full jig to arrive and stand still
			IF FULL[UNLOAD_IDX] AND TBL_INPOS THEN
				ST3_STEP := 110;
			END_IF;
		110:	// over the jig
			SOL_U_BIN := FALSE;
			SOL_U_JIG := TRUE;
			IF AS_U_JIG THEN
				ST3_STEP := 115;
			END_IF;
		115:	// down onto it
			SOL_U_UP := FALSE;
			SOL_U_DN := TRUE;
			IF AS_U_DN THEN
				ST3_STEP := 120;
			END_IF;
		120:	// whatever is in the jig, however many: a gap is not a reason to stop
			JIG_CLAMP := FALSE;
			UVAC_ON := TRUE;
			IF T_UPICK.Q THEN
				ST3_STEP := 130;
			END_IF;
		130:	// up, and the carrier is free to turn again
			JIG_CLAMP := TRUE;
			SOL_U_DN := FALSE;
			SOL_U_UP := TRUE;
			IF AS_U_UP THEN
				FULL[UNLOAD_IDX] := FALSE;
				ST3_STEP := 140;
			END_IF;
		140:	// across to the next process
			SOL_U_JIG := FALSE;
			SOL_U_BIN := TRUE;
			IF AS_U_BIN THEN
				ST3_STEP := 150;
			END_IF;
		150:	// down over the bin
			SOL_U_UP := FALSE;
			SOL_U_DN := TRUE;
			IF AS_U_DN THEN
				ST3_STEP := 160;
			END_IF;
		160:	// let them go: the next process takes them from here
			UVAC_ON := FALSE;
			IF NO_UVAC THEN
				ST3_STEP := 170;
			END_IF;
		170:	// up
			SOL_U_DN := FALSE;
			SOL_U_UP := TRUE;
			IF AS_U_UP THEN
				ST3_STEP := 180;
			END_IF;
		180:	// back over the jig, ready for the next one
			SOL_U_BIN := FALSE;
			SOL_U_JIG := TRUE;
			IF AS_U_JIG THEN
				ST3_STEP := 100;
			END_IF;
		ELSE
			ST3_STEP := 100;
	END_CASE;
END_IF;

// The loader drops one every 40 ms while it can, so 600 ms of nothing means the tray is full.
T_FILL(In := (ST1_STEP = 5) AND (EM_PAL_CNT = PAL_BASE), PT := T#600ms);
// A head is given this long to get hold of what is under it, then carries on with what it has.
T_PICK(In := (ST1_STEP = 30), PT := T#600ms);
T_UPICK(In := (ST3_STEP = 120), PT := T#600ms);

// Watchdog, one per station: a running step that stops moving is a jam, not patience. The
// carrier's wait for work and the unloader's wait for a jig are not stuck, so they are excluded.
T_WD1(In := (ST1_STEP = STEP1_LAST) AND RUNNING, PT := T#15s);
T_WD2(In := (ST2_STEP = STEP2_LAST) AND RUNNING AND (ST2_STEP <> 10), PT := T#15s);
T_WD3(In := (ST3_STEP = STEP3_LAST) AND RUNNING AND (ST3_STEP <> 100), PT := T#15s);
IF ST1_STEP <> STEP1_LAST THEN
	STEP1_LAST := ST1_STEP;
END_IF;
IF ST2_STEP <> STEP2_LAST THEN
	STEP2_LAST := ST2_STEP;
END_IF;
IF ST3_STEP <> STEP3_LAST THEN
	STEP3_LAST := ST3_STEP;
END_IF;
JAM := T_WD1.Q OR T_WD2.Q OR T_WD3.Q;
IF RUNNING AND (JAM OR SEL_CHANGED) THEN
	ST1_STEP := 900;
	ST2_STEP := 0;
	ST3_STEP := 0;
	TBL_RUN := FALSE;
	GX_EXEC := FALSE;
	GY_EXEC := FALSE;
	EM_PAL_EN := FALSE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	ST2_STEP := 0;
	ST3_STEP := 0;
	TBL_RUN := FALSE;
	GX_EXEC := FALSE;
	GY_EXEC := FALSE;
	EM_PAL_EN := FALSE;
	SOL_Z_DN := FALSE;
	SOL_Z_UP := FALSE;
	SOL_U_DN := FALSE;
	SOL_U_UP := FALSE;
	SOL_U_BIN := FALSE;
	SOL_U_JIG := FALSE;
	PAL_CLAMP := TRUE;
	JIG_CLAMP := TRUE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory per actuator, cleared when
// INDIVIDUAL ends. The index button is held and the carrier turns while it is down.
INDIVIDUAL := NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800);
IF INDIVIDUAL THEN
	IF IND_VAC AND NOT IND_VAC_LAST THEN
		MEM_VAC := NOT MEM_VAC;
	END_IF;
	IF IND_LIFT AND NOT IND_LIFT_LAST THEN
		MEM_LIFT := NOT MEM_LIFT;
	END_IF;
	IF IND_PAL AND NOT IND_PAL_LAST THEN
		MEM_PAL := NOT MEM_PAL;
	END_IF;
	IF IND_JIG AND NOT IND_JIG_LAST THEN
		MEM_JIG := NOT MEM_JIG;
	END_IF;
	IF IND_UVAC AND NOT IND_UVAC_LAST THEN
		MEM_UVAC := NOT MEM_UVAC;
	END_IF;
	IF IND_ULIFT AND NOT IND_ULIFT_LAST THEN
		MEM_ULIFT := NOT MEM_ULIFT;
	END_IF;
	IF IND_USHIFT AND NOT IND_USHIFT_LAST THEN
		MEM_USHIFT := NOT MEM_USHIFT;
	END_IF;
	VAC_ON := MEM_VAC;
	UVAC_ON := MEM_UVAC;
	PAL_CLAMP := MEM_PAL;
	JIG_CLAMP := MEM_JIG;
	SOL_Z_DN := MEM_LIFT;
	SOL_Z_UP := NOT MEM_LIFT;
	SOL_U_DN := MEM_ULIFT;
	SOL_U_UP := NOT MEM_ULIFT;
	SOL_U_BIN := MEM_USHIFT;
	SOL_U_JIG := NOT MEM_USHIFT;
	TBL_RUN := IND_INDEX;
ELSE
	MEM_VAC := FALSE;
	MEM_LIFT := FALSE;
	MEM_PAL := FALSE;
	MEM_JIG := FALSE;
	MEM_UVAC := FALSE;
	MEM_ULIFT := FALSE;
	MEM_USHIFT := FALSE;
END_IF;
IND_VAC_LAST := IND_VAC;
IND_LIFT_LAST := IND_LIFT;
IND_PAL_LAST := IND_PAL;
IND_JIG_LAST := IND_JIG;
IND_UVAC_LAST := IND_UVAC;
IND_ULIFT_LAST := IND_ULIFT;
IND_USHIFT_LAST := IND_USHIFT;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

// Jog: the axis creeps while the button is held, and only on INDIVIDUAL.
GX_JOG_P := INDIVIDUAL AND IND_GX_P;
GX_JOG_N := INDIVIDUAL AND IND_GX_N;
GY_JOG_P := INDIVIDUAL AND IND_GY_P;
GY_JOG_N := INDIVIDUAL AND IND_GY_N;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
