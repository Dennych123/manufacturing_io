VAR
	START_EDGE : BOOL;
	PB_START_LAST : BOOL;
	STOP_REQ : BOOL;
	T_DEMAND : TON;
END_VAR
// buffer-queue: parts queue nose to tail on the accumulation belt while it stands still, and one
// part is metered out on each demand by running that belt until the exit beam has seen the part
// pass. The feeder is throttled by the queue-full beam, so the buffer fills and then holds. The
// same sequence runs without a PLC in buffer-queue.ctl.js.
//
// The belt itself is the stop. No stopper pin is used here on purpose: with parts touching there
// is no gap for a pin to come down into, and it would land on a part.

START_EDGE := PB_START AND NOT PB_START_LAST;
PB_START_LAST := PB_START;
IF PB_STOP THEN
	STOP_REQ := TRUE;
END_IF;

CASE ST1_STEP OF
	0:	// idle
		CV_ACC_RUN := FALSE;
		IF START_EDGE THEN
			STOP_REQ := FALSE;
			ST1_STEP := 10;
		END_IF;
	10:	// the buffer fills while the downstream station works
		CV_ACC_RUN := FALSE;
		IF T_DEMAND.Q THEN
			ST1_STEP := 15;
		END_IF;
	15:	// the exit beam must be clear before metering, or the last part answers for this one
		CV_ACC_RUN := TRUE;
		IF NOT PE_EXIT THEN
			ST1_STEP := 20;
		END_IF;
	20:	// run until the front part is in the exit beam
		CV_ACC_RUN := TRUE;
		IF PE_EXIT THEN
			ST1_STEP := 30;
		END_IF;
	30:	// and until it has passed: exactly one part has left the queue
		CV_ACC_RUN := TRUE;
		IF NOT PE_EXIT THEN
			CV_ACC_RUN := FALSE;
			CYCLE_CNT := CYCLE_CNT + 1;
			IF STOP_REQ THEN
				ST1_STEP := 0;
			ELSE
				ST1_STEP := 10;
			END_IF;
		END_IF;
END_CASE;

// The metered part leaves down the discharge chute, and the feeder stops while the queue is full.
T_DEMAND(In := (ST1_STEP = 10), PT := T#2s);
AUTO_RUN := ST1_STEP <> 0;
EM_EN := AUTO_RUN AND NOT PE_FULL;
PL_START := AUTO_RUN;
