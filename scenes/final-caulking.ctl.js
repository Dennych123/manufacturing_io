// The final-caulking scene's controller: the machine's OWN CX-Programmer program (plc/
// final-caulking-mdf.cxp, CJ2M, the new design) run by the ladder soft-PLC in the plant's step -
// see server/ladderctl.js - plus the wiring around it that is neither the PLC nor a scene component:
//
//   - the hard-wired panel circuits the PLC only reads back (E-STOP contact, master relay, air,
//     fuse, door switches) and the light curtain's safety relay;
//   - the magnet controllers (KE-4E): the program holds MG1 ON / MG2 ON itself (5.05, 5.07);
//   - the operator's hands: a diaphragm into the feed-in through the light curtain, an M&B onto the
//     turntable, and the "nagara" start switch pressed after each;
//   - the Final Caulking machine next door, a MODEL of what its own PLC does: it answers over the
//     EtherNet/IP data link the program reads at E0_3300/E0_3301 (and writes at E0_3200/3201).
//     The hard-wired interlocks (CIO 2.xx) are switched off in the program (GSB001 = P_Off).
import { createLadderController } from '../server/ladderctl.js';

/** @param {any} scene @param {{root: string, log?: (s: string) => void}} opt */
export function create(scene, { root, log }) {
  const w = wiring();
  return createLadderController(scene, { root, log, wiring: w });
}

function wiring() {
  let masterCR = false;
  // operator hands: an edge each time, the automatic feed's timers, the hand in the curtain
  let emD = false, emM = false, emptyD = 0, emptyM = 0, lastT = 0, handD = 0, nagD = 0, nagM = 0, waitD = 0, waitM = 0;
  // Final Caulking model
  const fc = { k: 0, state: 'idle', t0: 0, res: /** @type {Array<'ok'|'ng'|null>} */ ([null, null]), req: false, moved: false, ngNext: false, ngSince: -1 };

  return {
    reset() {
      masterCR = false; emD = false; emM = false; emptyD = 0; emptyM = 0; lastT = 0; handD = 0; nagD = 0; nagM = 0; waitD = 0; waitM = 0;
      Object.assign(fc, { k: 0, state: 'idle', t0: 0, res: [null, null], req: false, ngSince: -1 });
    },
    /** Inputs the PLC reads that no component owns. @param {Record<string, any>} io @param {number} t @param {any} plc */
    before(io, t, plc) {
      const estop = !!io.PB_ESTOP;
      plc.set('0.00', estop ? 0 : 1);            // PB.EMG STOP is an NC contact: ON = not pressed
      plc.set('0.01', 1);                        // air pressure switch
      plc.set('0.07', 0);                        // teaching key off
      plc.set('0.08', 1);                        // machine door closed
      plc.set('0.09', 1);                        // dummy door closed
      plc.set('0.13', 1);                        // fuses
      // The light curtain where the outer cover was. Its safety relay reads back as SHUTTER DOOR OUT
      // (the program's INPUT section takes that from 2.00 now) and is muted while the program holds
      // 1.10 'Interupt Safety area' - AUTO with the inner cover down, or INDIVIDUAL at home. A hand
      // in it unmuted, with the inner cover up as well, is AL12 SHUTTER DOOR INTERRUPTED.
      const lcClear = !io.OP_LC_BREAK && handD <= 0;
      const lcOk = lcClear || !!plc.bit('1.10');
      plc.set('2.00', lcOk ? 1 : 0);
      plc.set('0.10', lcOk ? 1 : 0);
      plc.set('0.11', io.AS_SHUT2_DN ? 1 : 0);   // the inner cover's door switch
      // Master relay: MASTER ON pulls it in and it holds itself; the E-STOP drops it, and so does the
      // light curtain's safety relay when a hand breaks it UNMUTED (1.10 off: the inner cover is up).
      if (estop || !lcOk) masterCR = false; else if (io.PB_MASTER) masterCR = true;
      plc.set('0.12', masterCR ? 1 : 0);
      plc.set('0.14', plc.bit('5.00'));          // diaphragm door enable relay follows its coil
      plc.set('3304.15', 1); plc.set('4.15', 1); // safety area clear
      // sim_mch (E0_3.10), the program's own bench mode, ON (Denny: always). It bypasses the Final
      // Caulking interlocks and reads the ST1 photo-eye from 3303.13, so that input is the ST1 eye.
      plc.set('E0_3.10', io.SIM_MCH_OFF ? 0 : 1);
      plc.set('3303.13', io.FC_ST1_PH ? 1 : 0);
      // The nagara switches: the operator says "part is in" (the program takes the rising edge).
      plc.set('3301.15', io.OP_NAGARA_D || nagD > 0 ? 1 : 0);
      plc.set('3304.07', io.OP_NAGARA_M || nagM > 0 ? 1 : 0);
      finalCaulking(io, t, plc);
    },
    /** Outputs the PLC drives that no component owns. @param {Record<string, any>} io @param {number} t @param {any} plc */
    after(io, t, plc) {
      // KE-4E controllers: the program latches MG ON itself and drops it with MG OFF.
      io.MAG1_ON = !!plc.bit('5.05') && plc.running;
      io.MAG2_ON = !!plc.bit('5.07') && plc.running;
      // The cycle the viewer times (scene.cycle.countTag): one per horn caulked, OK or NG.
      io.CYCLE_CNT = io.FC_CAULK_CNT || 0;
      // The locating pin stands in the M&B path once it is up: the turntable cannot turn it past.
      io.ROT_PIN_LOCK = !!io.AS_PIN_UP;
      // The finish tray only locates the horn while it is up: at DOWN the tray tilts and the horn
      // must slide, and a part a nest holds is kinematic - it ignores the tilt.
      io.TRAY_CLAMP = !io.AS_TRAY_DN;
      // The operator: a press puts one part down where it belongs, if the spot is free, and then
      // presses the nagara switch. Keeping the cell fed, they put a diaphragm in only while the light
      // curtain is muted (the inner cover is down), and press nagara again while nothing has started.
      const dt = t - lastT; lastT = t;
      handD -= dt; nagD -= dt; nagM -= dt;
      emptyD = io.PH_DIAPH_LOADIN ? 0 : emptyD + dt;
      emptyM = io.PH_WORK_IN_ROT ? 0 : emptyM + dt;
      const auto = !!io.OP_AUTO_FEED;
      const wantD = (io.OP_PUT_DIAPH || (auto && emptyD > 1500 && !!io.AS_SHUT2_DN && !!plc.bit('1.10'))) && !io.PH_DIAPH_LOADIN;
      const wantM = (io.OP_PUT_MB || (auto && emptyM > 1500 && !!io.AS_MBIN_UP && !!io.AS_MBS_RET)) && !io.PH_WORK_IN_ROT;
      io.EM_DIAPH = wantD && !emD; emD = !!wantD;
      io.EM_MB = wantM && !emM; emM = !!wantM;
      if (io.EM_DIAPH) { emptyD = 0; handD = 150; waitD = 900; }   // the hand is out before the program reacts to the part
      if (io.EM_MB) { emptyM = 0; waitM = 900; }
      if (auto) {
        waitD -= dt; waitM -= dt;
        if (io.PH_DIAPH_LOADIN && waitD <= 0 && plc.word('H32') === 0) { nagD = 300; waitD = 2500; }
        if (io.PH_WORK_IN_ROT && waitM <= 0 && plc.word('H13') === 0 && !io.MTD1_FWD && !io.MTD1_REV) { nagM = 300; waitM = 2500; }
      }
    },
  };

  /**
   * The Final Caulking machine, as its data link shows it to this PLC. A two-station table: the
   * add-on loads and unloads at ST1, the press caulks at ST2. The handshake is the program's own
   * (P11 Device_Output R8): this machine REQUESTS a turn (E0_3300.15 'table rotate req') and the
   * add-on answers 'index rotate allowed' (E0_3200.14) unless its lift wants to come down. It asks
   * whenever ST1 holds a unit that is not caulked yet; a unit arriving at ST2 is caulked; the
   * result (OK/NG) is reported for the jig standing at ST1.
   * @param {Record<string, any>} io @param {number} t @param {any} plc
   */
  function finalCaulking(io, t, plc) {
    const allow = !!plc.bit('E0_3200.14'), liftUp = !!plc.bit('E0_3200.06');
    const st1 = !!io.FC_ST1_PH, atSt1 = fc.k % 2;
    const inPos = io.FC_TBL_INPOS !== false, pressUp = !!io.FC_PRESS_UP_END;
    io.FC_CV_RUN = true;
    switch (fc.state) {
      case 'idle':
        io.FC_INDEX = false; io.FC_PRESS_DN = false; io.FC_PRESS_UP = true; io.FC_CAULK = false;
        // The table turns on the add-on's START: P16 Station_Output R4 raises GB016_068 'Auto start
        // trigger' when its lift is back up off a loaded ST1, and Device_Output R33 sends it on as
        // E0_3200.07 (blinking at 0.1 s). E0_3300.15 (request) is still raised for the program's own interlocks.
        fc.req = st1 && fc.res[atSt1] == null && liftUp && inPos && pressUp;
        if (fc.req && (plc.bit('E0_3200.07') || plc.bit('E0_47.05'))) { fc.req = false; fc.state = 'index'; fc.t0 = t; fc.moved = false; }
        break;
      case 'index':
        // The cam may still be in its dwell: an index is done when in-position has DROPPED and come back.
        io.FC_INDEX = true;
        if (!inPos) fc.moved = true;
        if (fc.moved && inPos) { io.FC_INDEX = false; fc.k++; fc.state = io.FC_ST2_PH ? 'press' : 'idle'; fc.t0 = t; }
        break;
      case 'press':
        io.FC_PRESS_UP = false; io.FC_PRESS_DN = true;
        if (io.FC_PRESS_DN_END && t - fc.t0 > 300) {
          // In the program's bench mode (sim_mch) every horn is judged NG: INPUT R25 makes 'WORK NG'
          // from sim_mch AND 3303.13, which is the ST1 eye. So the model says NG too, and no OK.
          fc.ngNext = !!io.FC_FORCE_NG || !io.SIM_MCH_OFF;
          io.FC_NG = fc.ngNext; io.FC_CAULK = true;
          fc.res[(fc.k + 1) % 2] = fc.ngNext ? 'ng' : 'ok';      // the jig at ST2
          fc.state = 'return'; fc.t0 = t;
        }
        break;
      case 'return':
        io.FC_CAULK = false; io.FC_PRESS_DN = false; io.FC_PRESS_UP = true;
        if (pressUp) fc.state = 'idle';
        break;
    }
    const here = fc.k % 2;
    if (!st1 && fc.state === 'idle') fc.res[here] = null;
    const res = st1 ? fc.res[here] : null;
    // NG is reported as a PULSE when the jig comes back to ST1, OK as a level while it stands there.
    // The program latches NG (P08 R4: SET H88.01, after a 1 s filter) and clears it when a gripper
    // takes the horn - with the unit still in the jig. A held NG was set again at once, and the next
    // OK horn then read OK and NG together (H78 = 3) and no discharge case matched: measured, the
    // cell stopped on the first OK after an NG. An assumption about the machine next door: ask.
    if (res === 'ng' && fc.ngSince < 0) fc.ngSince = t;
    if (res !== 'ng') fc.ngSince = -1;
    const ngPulse = res === 'ng' && t - fc.ngSince < 1500;
    const busy = fc.state !== 'idle';
    const E = (/** @type {string} */ a, /** @type {any} */ v) => plc.set(a, v ? 1 : 0);
    E('E0_3300.00', 1); E('E0_3300.01', 0);   // .01 is 'fuse blown' (AL14 when ON)
    E('E0_3300.02', 1); E('E0_3300.03', inPos && pressUp);
    E('E0_3300.04', 1); E('E0_3300.05', 1);   // .05 ON = no abnormality (START CONDITION 1-11)
    E('E0_3300.06', 0); E('E0_3300.07', io.PH_MB_VAC);
    E('E0_3300.08', ngPulse); E('E0_3300.09', 0); E('E0_3300.10', res === 'ok' && !!io.SIM_MCH_OFF); E('E0_3300.11', busy);
    E('E0_3300.12', st1); E('E0_3300.13', io.FC_ST2_PH); E('E0_3300.14', io.FC_CV_PH); E('E0_3300.15', fc.req);
    E('E0_3301.00', 1); E('E0_3301.01', inPos); E('E0_3301.02', inPos); E('E0_3301.03', 1); E('E0_3301.05', 1); E('E0_3301.04', fc.state === 'index');
    E('E0_3301.06', 1); E('E0_3301.07', io.PH_MB_VAC); E('E0_3301.08', 1); E('E0_3301.09', io.PH_ST_DIAPH_VAC); E('E0_3301.10', 1);
    E('E0_3301.11', 0); E('E0_3301.13', 0); E('E0_3301.14', 1); E('E0_3301.15', 1);
  }
}
