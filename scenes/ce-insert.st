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
	TYPE1 : BOOL;
	A : INT;
	B : INT;
	I : INT;
	N : INT;
	SV_CENTER : LREAL;
	SV_LEFT : LREAL;
	SV_RIGHT : LREAL;
	SV_TGT : LREAL;
	HALF : INT;
	CHK : ARRAY[0..3] OF BOOL := [TRUE, TRUE, TRUE, TRUE];
	DIV_F : ARRAY[0..1] OF BOOL := [TRUE, TRUE];
	STP_DN : BOOL := TRUE;
	SEP_F : BOOL;
	PSH_F : BOOL;
	FDR : ARRAY[0..1] OF BOOL;
	MEM : ARRAY[0..3] OF BOOL;
	DELIV : BOOL;
	FLW : ARRAY[0..3] OF BOOL;
	SEEN1 : BOOL;
	SEEN2 : BOOL;
	FULL_IN : ARRAY[0..3] OF BOOL;
	FULL : ARRAY[0..3] OF BOOL;
	AS_CHK : ARRAY[0..3] OF BOOL;
	AS_UCHK : ARRAY[0..3] OF BOOL;
	FLW_IN : ARRAY[0..3] OF BOOL;
	T_FULL : ARRAY[0..3] OF TON;
	T_ST1OFF : TON;
	T_ST2OFF : TON;
	T_ST3FULL : TON;
	T_EXS : TON;
	T_DROP : TON;
	ST1_OFF : BOOL;
	ST2_OFF : BOOL;
	ST3_FULL : BOOL;
	EXS : BOOL;
	DROP_OFF : BOOL;
	ST2_READY : BOOL;
	T_WD1 : TON;
	T_WD2 : TON;
	T_WD3 : TON;
	STEP1_LAST : UDINT;
	STEP2_LAST : UDINT;
	STEP3_LAST : UDINT;
	JAM : BOOL;
	OUT_ON : BOOL;
	INDIVIDUAL : BOOL;
	IND_MOVE : BOOL;
	IND_TGT : LREAL;
	IND_C_LAST : BOOL;
	IND_L_LAST : BOOL;
	IND_R_LAST : BOOL;
	MEM_STP2 : BOOL;
	IND_STP2_LAST : BOOL;
	MEM_STP3 : BOOL;
	IND_STP3_LAST : BOOL;
	MEM_STP4 : BOOL;
	IND_STP4_LAST : BOOL;
	MEM_STP5 : BOOL;
	IND_STP5_LAST : BOOL;
	MEM_DIVL : BOOL;
	IND_DIVL_LAST : BOOL;
	MEM_DIVR : BOOL;
	IND_DIVR_LAST : BOOL;
	MEM_STP1 : BOOL;
	IND_STP1_LAST : BOOL;
	MEM_SEP : BOOL;
	IND_SEP_LAST : BOOL;
	MEM_PSH : BOOL;
	IND_PSH_LAST : BOOL;
	MEM_FDR1 : BOOL;
	IND_FDR1_LAST : BOOL;
	MEM_FDR2 : BOOL;
	IND_FDR2_LAST : BOOL;
END_VAR
// ce-insert: Denny's CE INSERT machine for spark plugs (tools/gen_ce_insert.js has the machine),
// ported from his NX1P2 program "Ce Insert Track". The same sequence runs without a PLC in
// ce-insert.ctl.js - keep the two in step.
//
// Three stations run at once and meet on shared state, as his Prg010/011/012 meet on GB010/011/012:
//   ST1  the running type's two gripper-stoppers open and pins run down to the divider until each
//        lane's FULL beam holds (then that gripper chucks); when the shuttle waits at a receiving
//        position the divider opens, the batch runs off, and it shuts once INSERT FLOW OUT has been
//        dark for 500 ms - after it has SEEN the batch go past.
//   ST2  the shuttle takes two batches on its four lanes: type 1 (left feeder, lanes 1-2) fills
//        shuttle lanes 1-2 at CENTER and 3-4 at LEFT; type 2 fills 3-4 at CENTER and 1-2 at RIGHT.
//        Full, it goes to CENTER and lifts STOPPER-1 onto the chute, while the chute is not full.
//   ST3  four pins at the chute end (SHUTTER EXIST 1-4): the separator takes them out of the queue,
//        goes back, the pusher pushes them over the receivers and they drop down the pipes.
//
// Not copied from his program, both found while porting: its ST3 drives the EJECTOR valve from the
// PUSHER symbols and the other way round (Denny confirmed the swap), and its ST2 Memory rung sets
// "shutter 3, 4 full" at POS4 with no batch delivered.

// BEGIN GENERATED - tools/gen_ce_insert.js
SV_CENTER := 110.0;
SV_LEFT := 30.0;
SV_RIGHT := 190.0;
// END GENERATED

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
TYPE1 := NOT SEL_TYPE2;
IF TYPE1 THEN
	A := 0;
ELSE
	A := 2;
END_IF;
B := A + 1;

// The beams, debounced the way his LT timers do it.
FULL_IN[0] := PH_ST1_SHT1_FULL;
FULL_IN[1] := PH_ST1_SHT2_FULL;
FULL_IN[2] := PH_ST1_SHT3_FULL;
FULL_IN[3] := PH_ST1_SHT4_FULL;
FOR I := 0 TO 3 DO
	T_FULL[I](In := FULL_IN[I], PT := T#300ms);
	FULL[I] := T_FULL[I].Q;
END_FOR;
AS_CHK[0] := AS_ST1_STP2_CHK;
AS_CHK[1] := AS_ST1_STP3_CHK;
AS_CHK[2] := AS_ST1_STP4_CHK;
AS_CHK[3] := AS_ST1_STP5_CHK;
AS_UCHK[0] := AS_ST1_STP2_UCHK;
AS_UCHK[1] := AS_ST1_STP3_UCHK;
AS_UCHK[2] := AS_ST1_STP4_UCHK;
AS_UCHK[3] := AS_ST1_STP5_UCHK;
FLW_IN[0] := PH_ST3_SHT1_FLW_OUT;
FLW_IN[1] := PH_ST3_SHT2_FLW_OUT;
FLW_IN[2] := PH_ST3_SHT3_FLW_OUT;
FLW_IN[3] := PH_ST3_SHT4_FLW_OUT;
T_ST1OFF(In := NOT PH_ST1_INS_FLW_OUT, PT := T#500ms);
ST1_OFF := T_ST1OFF.Q;
T_ST2OFF(In := NOT PH_ST2_INS_FLW_OUT, PT := T#500ms);
ST2_OFF := T_ST2OFF.Q;
T_ST3FULL(In := PH_ST3_SHT_FULL, PT := T#300ms);
ST3_FULL := T_ST3FULL.Q;
T_EXS(In := PH_ST3_SHT1_EXS AND PH_ST3_SHT2_EXS AND PH_ST3_SHT3_EXS AND PH_ST3_SHT4_EXS, PT := T#200ms);
EXS := T_EXS.Q;
T_DROP(In := NOT (PH_ST3_SHT1_FLW_OUT OR PH_ST3_SHT2_FLW_OUT OR PH_ST3_SHT3_FLW_OUT OR PH_ST3_SHT4_FLW_OUT), PT := T#100ms);
DROP_OFF := T_DROP.Q;
// ST2 stands at a receiving position waiting for a batch (his GB011_11 / 12).
ST2_READY := ST2_STEP = 120;

// ---------------------------------------------------------------- ST1: supply feeder
CASE ST1_STEP OF
	0:	// idle
		FDR[0] := FALSE;
		FDR[1] := FALSE;
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_ST2_STP1_DN AND AS_ST3_EJC_BWD AND AS_ST3_PSH_BWD THEN
			STOP_REQ := FALSE;
			ST2_STEP := 100;
			ST3_STEP := 200;
			ST1_STEP := 10;
		END_IF;
	10:	// the running type's bowl feeds all the time; buffer, or discharge into a waiting shuttle
		FDR[0] := TYPE1;
		FDR[1] := NOT TYPE1;
		IF NOT (FULL[A] AND FULL[B]) THEN
			ST1_STEP := 20;
		ELSIF ST2_READY THEN
			ST1_STEP := 40;
		END_IF;
	20:	// buffering: both grippers of the pair open
		CHK[A] := FALSE;
		CHK[B] := FALSE;
		IF AS_UCHK[A] AND AS_UCHK[B] THEN
			ST1_STEP := 30;
		END_IF;
	30:	// each gripper chucks as soon as its own lane is full
		IF FULL[A] THEN
			CHK[A] := TRUE;
		END_IF;
		IF FULL[B] THEN
			CHK[B] := TRUE;
		END_IF;
		IF CHK[A] AND CHK[B] AND AS_CHK[A] AND AS_CHK[B] THEN
			ST1_STEP := 10;
		END_IF;
	40:	// discharging: the pair's divider opens
		IF TYPE1 THEN
			DIV_F[0] := FALSE;
		ELSE
			DIV_F[1] := FALSE;
		END_IF;
		SEEN1 := FALSE;
		IF (TYPE1 AND AS_ST1_LFT_DIV_BWD) OR (NOT TYPE1 AND AS_ST1_RGT_DIV_BWD) THEN
			ST1_STEP := 45;
		END_IF;
	45:	// the batch must be SEEN going past before "dark" means it has gone
		IF PH_ST1_INS_FLW_OUT THEN
			SEEN1 := TRUE;
		END_IF;
		IF SEEN1 AND ST1_OFF THEN
			ST1_STEP := 50;
		END_IF;
	50:
		DIV_F[0] := TRUE;
		DIV_F[1] := TRUE;
		IF AS_ST1_LFT_DIV_FWD AND AS_ST1_RGT_DIV_FWD THEN
			DELIV := TRUE;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSE
				ST1_STEP := 10;
			END_IF;
		END_IF;
	800:	// HOME: grippers chucked, dividers and STOPPER-1 closed, separator and pusher back, shuttle CENTER
		FOR I := 0 TO 3 DO
			CHK[I] := TRUE;
		END_FOR;
		DIV_F[0] := TRUE;
		DIV_F[1] := TRUE;
		STP_DN := TRUE;
		SEP_F := FALSE;
		PSH_F := FALSE;
		FDR[0] := FALSE;
		FDR[1] := FALSE;
		SV_ST2_TGT := SV_CENTER;
		SV_ST2_EXEC := TRUE;
		IF SV_ST2_DONE AND AS_ST1_STP2_CHK AND AS_ST1_STP3_CHK AND AS_ST1_STP4_CHK AND AS_ST1_STP5_CHK
				AND AS_ST1_LFT_DIV_FWD AND AS_ST1_RGT_DIV_FWD AND AS_ST2_STP1_DN AND AS_ST3_EJC_BWD AND AS_ST3_PSH_BWD THEN
			SV_ST2_EXEC := FALSE;
			FOR I := 0 TO 3 DO
				MEM[I] := FALSE;
			END_FOR;
			DELIV := FALSE;
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP
		FDR[0] := FALSE;
		FDR[1] := FALSE;
		SV_ST2_EXEC := FALSE;
		IF READY THEN
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: START acknowledges
		FDR[0] := FALSE;
		FDR[1] := FALSE;
		SV_ST2_EXEC := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			ST1_STEP := 0;
		END_IF;
END_CASE;
RUNNING := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// ---------------------------------------------------------------- ST2: the shuttle
IF NOT RUNNING THEN
	ST2_STEP := 0;
ELSE
	CASE ST2_STEP OF
		100:	// decide
			N := 0;
			FOR I := 0 TO 3 DO
				IF MEM[I] THEN
					N := N + 1;
				END_IF;
			END_FOR;
			IF N = 4 THEN
				IF NOT ST3_FULL AND ST2_OFF THEN
					SV_TGT := SV_CENTER;
					ST2_STEP := 160;
				END_IF;
			ELSIF N = 0 THEN
				SV_TGT := SV_CENTER;
				IF TYPE1 THEN
					HALF := 0;
				ELSE
					HALF := 2;
				END_IF;
				ST2_STEP := 110;
			ELSIF TYPE1 AND MEM[0] AND MEM[1] AND NOT MEM[2] AND NOT MEM[3] THEN
				SV_TGT := SV_LEFT;
				HALF := 2;
				ST2_STEP := 110;
			ELSIF NOT TYPE1 AND MEM[2] AND MEM[3] AND NOT MEM[0] AND NOT MEM[1] THEN
				SV_TGT := SV_RIGHT;
				HALF := 0;
				ST2_STEP := 110;
			ELSIF NOT ST3_FULL AND ST2_OFF THEN
				// half a load of the other type (the type was changed): run it off
				SV_TGT := SV_CENTER;
				ST2_STEP := 160;
			END_IF;
		110:	// Execute is a LEVEL held until Done
			SV_ST2_TGT := SV_TGT;
			SV_ST2_EXEC := TRUE;
			IF SV_ST2_DONE THEN
				SV_ST2_EXEC := FALSE;
				ST2_STEP := 115;
			END_IF;
		115:
			IF NOT SV_ST2_DONE THEN
				ST2_STEP := 120;
			END_IF;
		120:	// at a receiving position: ST1 discharges into the lanes in front of it
			IF DELIV THEN
				DELIV := FALSE;
				MEM[HALF] := TRUE;
				MEM[HALF + 1] := TRUE;
				ST2_STEP := 100;
			END_IF;
		160:
			SV_ST2_TGT := SV_CENTER;
			SV_ST2_EXEC := TRUE;
			IF SV_ST2_DONE THEN
				SV_ST2_EXEC := FALSE;
				ST2_STEP := 165;
			END_IF;
		165:
			IF NOT SV_ST2_DONE THEN
				ST2_STEP := 170;
			END_IF;
		170:	// STOPPER-1 up: all four lanes run onto the chute
			STP_DN := FALSE;
			SEEN2 := FALSE;
			IF AS_ST2_STP1_UP THEN
				ST2_STEP := 175;
			END_IF;
		175:
			IF PH_ST2_INS_FLW_OUT THEN
				SEEN2 := TRUE;
			END_IF;
			IF SEEN2 AND ST2_OFF THEN
				ST2_STEP := 180;
			END_IF;
		180:
			STP_DN := TRUE;
			IF AS_ST2_STP1_DN THEN
				FOR I := 0 TO 3 DO
					MEM[I] := FALSE;
				END_FOR;
				ST2_STEP := 100;
			END_IF;
		ELSE
			ST2_STEP := 100;
	END_CASE;
END_IF;

// ---------------------------------------------------------------- ST3: CE eject
IF NOT RUNNING THEN
	ST3_STEP := 0;
ELSE
	CASE ST3_STEP OF
		200:	// four pins at the chute end
			SEP_F := FALSE;
			PSH_F := FALSE;
			IF EXS THEN
				ST3_STEP := 210;
			END_IF;
		210:	// the separator takes them out of the queue
			SEP_F := TRUE;
			IF AS_ST3_EJC_FWD THEN
				ST3_STEP := 215;
			END_IF;
		215:	// still in the beams, in the slots
			IF PH_ST3_SHT1_EXS AND PH_ST3_SHT2_EXS AND PH_ST3_SHT3_EXS AND PH_ST3_SHT4_EXS THEN
				ST3_STEP := 220;
			END_IF;
		220:
			SEP_F := FALSE;
			IF AS_ST3_EJC_BWD THEN
				FOR I := 0 TO 3 DO
					FLW[I] := FALSE;
				END_FOR;
				ST3_STEP := 230;
			END_IF;
		230:	// the pusher takes them over the receivers: they drop
			PSH_F := TRUE;
			IF AS_ST3_PSH_FWD THEN
				ST3_STEP := 240;
			END_IF;
		240:
			PSH_F := FALSE;
			IF AS_ST3_PSH_BWD THEN
				ST3_STEP := 250;
			END_IF;
		250:	// every pin went down its pipe, and none is still in a beam
			IF FLW[0] AND FLW[1] AND FLW[2] AND FLW[3] AND DROP_OFF THEN
				CYCLE_CNT := CYCLE_CNT + 1;
				ST3_STEP := 200;
			END_IF;
		ELSE
			ST3_STEP := 200;
	END_CASE;
END_IF;
IF (ST3_STEP >= 230) AND (ST3_STEP <= 250) THEN
	FOR I := 0 TO 3 DO
		IF FLW_IN[I] THEN
			FLW[I] := TRUE;
		END_IF;
	END_FOR;
END_IF;

// Watchdog, one per station, on the steps that MOVE something (waits for material excluded).
T_WD1(In := (ST1_STEP = STEP1_LAST) AND RUNNING AND (ST1_STEP <> 10), PT := T#15s);
T_WD2(In := (ST2_STEP = STEP2_LAST) AND RUNNING AND (ST2_STEP <> 100) AND (ST2_STEP <> 120), PT := T#15s);
T_WD3(In := (ST3_STEP = STEP3_LAST) AND RUNNING AND (ST3_STEP <> 200), PT := T#15s);
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
	SV_ST2_EXEC := FALSE;
	FDR[0] := FALSE;
	FDR[1] := FALSE;
END_IF;
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	ST2_STEP := 0;
	ST3_STEP := 0;
	SV_ST2_EXEC := FALSE;
	FDR[0] := FALSE;
	FDR[1] := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: momentary buttons, a PLC toggle memory per actuator, cleared when
// INDIVIDUAL ends. The shuttle position buttons start one move each; Execute drops at Done.
INDIVIDUAL := NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800);
IF INDIVIDUAL THEN
	IF IND_STP2 AND NOT IND_STP2_LAST THEN
		MEM_STP2 := NOT MEM_STP2;
	END_IF;
	IF IND_STP3 AND NOT IND_STP3_LAST THEN
		MEM_STP3 := NOT MEM_STP3;
	END_IF;
	IF IND_STP4 AND NOT IND_STP4_LAST THEN
		MEM_STP4 := NOT MEM_STP4;
	END_IF;
	IF IND_STP5 AND NOT IND_STP5_LAST THEN
		MEM_STP5 := NOT MEM_STP5;
	END_IF;
	IF IND_DIVL AND NOT IND_DIVL_LAST THEN
		MEM_DIVL := NOT MEM_DIVL;
	END_IF;
	IF IND_DIVR AND NOT IND_DIVR_LAST THEN
		MEM_DIVR := NOT MEM_DIVR;
	END_IF;
	IF IND_STP1 AND NOT IND_STP1_LAST THEN
		MEM_STP1 := NOT MEM_STP1;
	END_IF;
	IF IND_SEP AND NOT IND_SEP_LAST THEN
		MEM_SEP := NOT MEM_SEP;
	END_IF;
	IF IND_PSH AND NOT IND_PSH_LAST THEN
		MEM_PSH := NOT MEM_PSH;
	END_IF;
	IF IND_FDR1 AND NOT IND_FDR1_LAST THEN
		MEM_FDR1 := NOT MEM_FDR1;
	END_IF;
	IF IND_FDR2 AND NOT IND_FDR2_LAST THEN
		MEM_FDR2 := NOT MEM_FDR2;
	END_IF;
	CHK[0] := NOT MEM_STP2;
	CHK[1] := NOT MEM_STP3;
	CHK[2] := NOT MEM_STP4;
	CHK[3] := NOT MEM_STP5;
	DIV_F[0] := NOT MEM_DIVL;
	DIV_F[1] := NOT MEM_DIVR;
	STP_DN := NOT MEM_STP1;
	SEP_F := MEM_SEP;
	PSH_F := MEM_PSH;
	FDR[0] := MEM_FDR1;
	FDR[1] := MEM_FDR2;
	IF NOT IND_MOVE AND ((IND_SV_CTR AND NOT IND_C_LAST) OR (IND_SV_LFT AND NOT IND_L_LAST) OR (IND_SV_RGT AND NOT IND_R_LAST)) THEN
		IND_MOVE := TRUE;
		IF IND_SV_CTR THEN
			IND_TGT := SV_CENTER;
		ELSIF IND_SV_LFT THEN
			IND_TGT := SV_LEFT;
		ELSE
			IND_TGT := SV_RIGHT;
		END_IF;
	END_IF;
	IND_C_LAST := IND_SV_CTR;
	IND_L_LAST := IND_SV_LFT;
	IND_R_LAST := IND_SV_RGT;
	SV_ST2_TGT := IND_TGT;
	SV_ST2_EXEC := IND_MOVE;
	IF IND_MOVE AND SV_ST2_DONE THEN
		IND_MOVE := FALSE;
	END_IF;
ELSE
	MEM_STP2 := FALSE;
	MEM_STP3 := FALSE;
	MEM_STP4 := FALSE;
	MEM_STP5 := FALSE;
	MEM_DIVL := FALSE;
	MEM_DIVR := FALSE;
	MEM_STP1 := FALSE;
	MEM_SEP := FALSE;
	MEM_PSH := FALSE;
	MEM_FDR1 := FALSE;
	MEM_FDR2 := FALSE;
	IND_MOVE := FALSE;
	IND_C_LAST := FALSE;
	IND_L_LAST := FALSE;
	IND_R_LAST := FALSE;
END_IF;
IND_STP2_LAST := IND_STP2;
IND_STP3_LAST := IND_STP3;
IND_STP4_LAST := IND_STP4;
IND_STP5_LAST := IND_STP5;
IND_DIVL_LAST := IND_DIVL;
IND_DIVR_LAST := IND_DIVR;
IND_STP1_LAST := IND_STP1;
IND_SEP_LAST := IND_SEP;
IND_PSH_LAST := IND_PSH;
IND_FDR1_LAST := IND_FDR1;
IND_FDR2_LAST := IND_FDR2;

// Solenoids from where each actuator is told to be. Without MASTER ON (and on E-STOP) the master
// circuit is open and every coil drops: a double-solenoid valve then holds where it is.
OUT_ON := READY;
SOL_ST1_STP2_CHK := OUT_ON AND CHK[0];
SOL_ST1_STP2_UCHK := OUT_ON AND NOT CHK[0];
SOL_ST1_STP3_CHK := OUT_ON AND CHK[1];
SOL_ST1_STP3_UCHK := OUT_ON AND NOT CHK[1];
SOL_ST1_STP4_CHK := OUT_ON AND CHK[2];
SOL_ST1_STP4_UCHK := OUT_ON AND NOT CHK[2];
SOL_ST1_STP5_CHK := OUT_ON AND CHK[3];
SOL_ST1_STP5_UCHK := OUT_ON AND NOT CHK[3];
SOL_ST1_LFT_DIV_FWD := OUT_ON AND DIV_F[0];
SOL_ST1_LFT_DIV_BWD := OUT_ON AND NOT DIV_F[0];
SOL_ST1_RGT_DIV_FWD := OUT_ON AND DIV_F[1];
SOL_ST1_RGT_DIV_BWD := OUT_ON AND NOT DIV_F[1];
SOL_ST2_STP1_DN := OUT_ON AND STP_DN;
SOL_ST2_STP1_UP := OUT_ON AND NOT STP_DN;
SOL_ST3_EJC_FWD := OUT_ON AND SEP_F;
SOL_ST3_EJC_BWD := OUT_ON AND NOT SEP_F;
SOL_ST3_PSH_FWD := OUT_ON AND PSH_F;
SOL_ST3_PSH_BWD := OUT_ON AND NOT PSH_F;
CR_ST1_PART_FDR1_STR := OUT_ON AND FDR[0];
CR_ST1_PART_FDR2_STR := OUT_ON AND FDR[1];

OVR := OVR_SET;
SV_ST2_JOG_P := INDIVIDUAL AND IND_SV_P;
SV_ST2_JOG_N := INDIVIDUAL AND IND_SV_N;
PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
PL_SV_CTR := ABS(SV_ST2_POS - SV_CENTER) < 0.5;
PL_SV_LFT := ABS(SV_ST2_POS - SV_LEFT) < 0.5;
PL_SV_RGT := ABS(SV_ST2_POS - SV_RIGHT) < 0.5;
