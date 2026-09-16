VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	SEL_LAST : BOOL := TRUE;
	SEL_CHANGED : BOOL;
	T_DWELL : TON;
	T_SETTLE : TON;
	T_CONF : TON;
	T_WD : TON;
	EM_P_LAST : UDINT;
	EM_W_LAST : UDINT;
	STEP_LAST : UDINT;
	GONE : UDINT;
	FED : UDINT;
	MEM_CV : BOOL;
	MEM_STOP : BOOL;
	MEM_LIFT : BOOL;
	IND_CV_LAST : BOOL;
	IND_STOP_LAST : BOOL;
	IND_LIFT_LAST : BOOL;
	ESTOP : BOOL;
	READY : BOOL;
	MASTER_ON : BOOL;
	HOMED : BOOL;
	MASTER_EDGE : BOOL;
	PB_MASTER_LAST : BOOL;
	HOME_EDGE : BOOL;
	PB_HOME_LAST : BOOL;
END_VAR
// pallet-line: a pallet is fed onto the belt, stopped at the station by a POP-UP stop, lifted off
// the belt and located square on its pins, loaded with a part, then put back down and released to
// the unloader. Repeat until CYCLE STOP. The same sequence runs without a PLC in pallet-line.ctl.js.
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
//
// The panel has a selector AUTO / INDIVIDUAL (rb4axis): START only in AUTO, the individual buttons
// only in INDIVIDUAL, and a change while the sequence runs is a FAULT, not a pause.

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
		IF START_EDGE AND SEL_AUTO AND READY AND HOMED AND AS_LIFT_DN THEN
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
	30:	// convey it to the station. The beam only says a pallet is HERE; the PIN locates it.
		// Stopping the belt on the beam edge left the pallet 106 mm short of the pin, so the
		// part feeder dropped its load onto the belt behind the deck every cycle (measured).
		CV1_RUN := TRUE;
		IF PE_STN THEN
			ST1_STEP := 35;
		END_IF;
	35:	// keep driving until it is pressed against the pin
		CV1_RUN := TRUE;
		IF T_SETTLE.Q THEN
			CV1_RUN := FALSE;
			ST1_STEP := 40;
		END_IF;
	40:	// lift and locate. The pallet must stay reported present for a CONFIRM time, not just for
		// the instant the switch first makes: a part is dropped on the strength of this signal,
		// and T_CONF restarts if it flickers.
		SOL_LIFT := TRUE;
		IF T_CONF.Q THEN
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
	800:	// HOME: the lift down and the pin down, so the belt is clear along its whole length
		CV1_RUN := FALSE;
		EM_P_EMIT := FALSE;
		EM_W_EMIT := FALSE;
		SOL_LIFT := FALSE;
		SOL_STOP := FALSE;
		IF AS_LIFT_DN AND AS_STOP_DN THEN
			HOMED := TRUE;
			ST1_STEP := 0;
		END_IF;
	910:	// E-STOP: the master circuit is open, so every solenoid de-energises - a 5/2 single-
		// solenoid valve springs back. The LIFT is the exception: it is a holder, and a real
		// machine does not drop what it carries when the power goes.
		CV1_RUN := FALSE;
		EM_P_EMIT := FALSE;
		EM_W_EMIT := FALSE;
		SOL_STOP := FALSE;
		IF READY THEN	// MASTER ON after the mushroom is released
			ST1_STEP := 0;
		END_IF;
	900:	// FAULT: a step stopped moving. Stop feeding and the belt; the pin keeps holding the
		// queue back and the lift keeps its pallet.
		CV1_RUN := FALSE;
		EM_P_EMIT := FALSE;
		EM_W_EMIT := FALSE;
		IF START_EDGE AND SEL_AUTO THEN
			STOP_REQ := FALSE;
			// Acknowledging writes off what never reached the unloader. Only HERE: at every START
			// it would write off pallets still legitimately on the belt.
			GONE := FED - RM_CNT;
			ST1_STEP := 0;
		END_IF;
END_CASE;

T_DWELL(In := (ST1_STEP = 60), PT := T#500ms);
// 106 mm from the beam edge to the pin at 250 mm/s is 424 ms; 700 leaves margin to press up.
T_SETTLE(In := (ST1_STEP = 35), PT := T#700ms);
T_CONF(In := (ST1_STEP = 40) AND AS_LIFT_UP AND PLT_PRESENT, PT := T#300ms);

// Watchdog: a running step that stops moving is a jam, not patience. Without it the sequence waits
// for ever and the machine only LOOKS alive. A selector change while running trips the same way.
T_WD(In := (ST1_STEP = STEP_LAST) AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910), PT := T#15s);
IF ST1_STEP <> STEP_LAST THEN
	STEP_LAST := ST1_STEP;
END_IF;
IF T_WD.Q OR (SEL_CHANGED AND (ST1_STEP <> 0) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910)) THEN
	ST1_STEP := 900;
	CV1_RUN := FALSE;
	EM_P_EMIT := FALSE;
	EM_W_EMIT := FALSE;
END_IF;

// E-STOP at any moment, and the HOME button when the machine is energised and idle.
IF ESTOP THEN
	IF ST1_STEP <> 910 THEN
		ST1_STEP := 910;
	END_IF;
	CV1_RUN := FALSE;
	EM_P_EMIT := FALSE;
	EM_W_EMIT := FALSE;
	SOL_STOP := FALSE;
ELSIF HOME_EDGE AND READY AND (ST1_STEP = 0) THEN
	ST1_STEP := 800;
END_IF;

AUTO_RUN := (ST1_STEP <> 0) AND (ST1_STEP <> 800) AND (ST1_STEP <> 900) AND (ST1_STEP <> 910);

// Individual operation: the buttons are momentary; the PLC keeps a toggle memory per actuator and
// clears it when INDIVIDUAL ends, so nothing stays latched into AUTO.
IF NOT SEL_AUTO AND NOT AUTO_RUN AND READY AND (ST1_STEP <> 800) THEN
	IF IND_CV AND NOT IND_CV_LAST THEN
		MEM_CV := NOT MEM_CV;
	END_IF;
	IF IND_STOP AND NOT IND_STOP_LAST THEN
		MEM_STOP := NOT MEM_STOP;
	END_IF;
	IF IND_LIFT AND NOT IND_LIFT_LAST THEN
		MEM_LIFT := NOT MEM_LIFT;
	END_IF;
	CV1_RUN := MEM_CV;
	SOL_STOP := MEM_STOP;
	SOL_LIFT := MEM_LIFT;
	EM_P_EMIT := IND_FEED_P;
	EM_W_EMIT := IND_FEED_W;
ELSE
	MEM_CV := FALSE;
	MEM_STOP := FALSE;
	MEM_LIFT := FALSE;
END_IF;
IND_CV_LAST := IND_CV;
IND_STOP_LAST := IND_STOP;
IND_LIFT_LAST := IND_LIFT;

// The speed override the operator dialled in, passed on to the axes and the belts.
OVR := OVR_SET;

PL_START := AUTO_RUN;
PL_MASTER := MASTER_ON;
PL_HOME := HOMED;
