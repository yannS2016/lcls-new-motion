from epics import PV
import time
import sys
import random
import argparse

def get_args():
    parser = argparse.ArgumentParser(
        description="EPICS Axis Batch Motion/Homing/Compensation Test Script"
    )
    parser.add_argument(
        "--motor", type=str, required=True,
        help="EPICS PV prefix for the axis, e.g. 'TST:MOTION:M1:PLC:'"
    )
    parser.add_argument(
        "--num-moves", type=int, default=5,
        help="Number of random absolute moves to batch test (default: 5)"
    )
    parser.add_argument(
        "-v", "--verbose", action='store_true',
        help="Print detailed debug info"
    )
    return parser.parse_args()

args = get_args()
MOTOR = args.motor
num_moves = args.num_moves
VERBOSE = args.verbose

def vprint(*a, **kw):
    if VERBOSE:
        print(*a, **kw)

PVNAMES = {
    "bHomeCmd":         MOTOR + "bHomeCmd",
    "bMoveCmd":         MOTOR + "bMoveCmd",
    "bReset":           MOTOR + "bReset",
    "bHalt":            MOTOR + "bHalt",
    "fPosition":        MOTOR + "fPosition",
    "fHomePosition":    MOTOR + "fHomePosition",
    "fVelocity":        MOTOR + "fVelocity",
    "fAcceleration":    MOTOR + "fAcceleration",
    "fDeceleration":    MOTOR + "fDeceleration",
    "nCmdData":         MOTOR + "nCmdData",
    "nCommand":         MOTOR + "nCommand",
    "bBacklashEnable":  MOTOR + "bBacklashEnable",
    "fBacklashComp":    MOTOR + "fBacklashComp",
    "fActPosition":     MOTOR + "AxisStatus:PLC:fActPosition_RBV",
    "bDone":            MOTOR + "AxisStatus:PLC:bDone_RBV",
    "bBusy":            MOTOR + "AxisStatus:PLC:bBusy_RBV",
    "bError":           MOTOR + "AxisStatus:PLC:bError_RBV",
    "nErrorId":         MOTOR + "AxisStatus:PLC:nErrorId_RBV",
    "bHomed":           MOTOR + "AxisStatus:PLC:bHomed_RBV",
    "sErrorMessage":    MOTOR + "AxisStatus:PLC:sErrorMessage_RBV",
    "bBacklashStatus":  MOTOR + "bBacklashStatus_RBV",
    "fMeasuredBacklashComp": MOTOR + "fMeasuredBacklashComp_RBV",
}
PVs = {key: PV(pvname) for key, pvname in PVNAMES.items()}

def assert_true(cond, msg):
    if not cond:
        print('[FAIL]', msg)
        raise AssertionError(msg)
    print('[PASS]', msg)

def assert_false(cond, msg):
    assert_true(not cond, msg)

def check_connected():
    for key, pv in PVs.items():
        if not pv.wait_for_connection(timeout=2.0):
            raise Exception(f"PV {key} ({pv.pvname}) not connected!")

def wait_pv_flag(key, value=1, timeout=14.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        v = PVs[key].get(timeout=1)
        if v == value:
            return True
        time.sleep(0.2)
    raise AssertionError(f"Timeout waiting for {key} == {value}")

def wait_move_and_check(target, pos_tol=0.03, timeout=10.0, expect_comp=None, comp_tol=0.01):
    done_flag = [False]
    def cb_done(pvname=None, value=None, **_):
        if value:
            done_flag[0] = True
    PVs["bDone"].add_callback(cb_done)
    PVs["bDone"].use_monitor = True
    t0 = time.time()
    while not done_flag[0]:
        if time.time() - t0 > timeout:
            raise AssertionError("Timeout waiting for bDone")
        time.sleep(0.2)
    pos = PVs["fActPosition"].get(timeout=1.0)
    vprint(f"[INFO] Final position: {pos}, Target: {target}")
    assert_true(abs(pos - target) < pos_tol, f"Final position {pos} matches target {target}")
    if expect_comp is not None:
        fmeas = PVs["fMeasuredBacklashComp"].get(timeout=1.0)
        vprint(f"[INFO] Measured compensation: {fmeas}, expected {expect_comp}")
        if abs(expect_comp) < 0.001:
            assert_true(abs(fmeas) < comp_tol, f"Measured compensation {fmeas} should be ~0")
        else:
            assert_true(abs(fmeas - expect_comp) < comp_tol, f"Measured compensation {fmeas} matches expected {expect_comp}")

def move_abs_offset(offset, min_time=2.5, max_velocity=2200.0, max_accel=15000.0, expect_comp=None):
    target = PVs["fActPosition"].get(timeout=1.0) + offset
    distance = abs(offset)
    velocity = max(15.0, min(max_velocity, distance / min_time))
    acceleration = deceleration = min(max_accel, velocity * 2.0)
    print(f"[MOVE] {target-offset:.2f} -> {target:.2f} [v={velocity}, a={acceleration}]")
    PVs["fPosition"].put(target, timeout=1.0)
    PVs["fVelocity"].put(velocity, timeout=1.0)
    PVs["fAcceleration"].put(acceleration, timeout=1.0)
    PVs["fDeceleration"].put(deceleration, timeout=1.0)
    PVs["nCommand"].put(1, timeout=1.0)
    PVs["bMoveCmd"].put(1, timeout=1.0)
    time.sleep(1.5)
    PVs["bMoveCmd"].put(0, timeout=1.0)
    wait_move_and_check(target, timeout=10.0, expect_comp=expect_comp)

def test_halt_or_reset(offset, do_halt=True):
    target = PVs["fActPosition"].get(timeout=1.0) + offset
    distance = abs(offset)
    velocity = max(15.0, min(2200.0, distance / 2.5))
    acceleration = deceleration = min(15000.0, velocity * 2.0)
    print(f"[MOVE] {target-offset:.2f} -> {target:.2f} [v={velocity}, a={acceleration}]")
    PVs["fPosition"].put(target, timeout=1.0)
    PVs["fVelocity"].put(velocity, timeout=1.0)
    PVs["fAcceleration"].put(acceleration, timeout=1.0)
    PVs["fDeceleration"].put(deceleration, timeout=1.0)
    PVs["nCommand"].put(1, timeout=1.0)
    PVs["bMoveCmd"].put(1, timeout=1.0)
    time.sleep(0.3)
    PVs["bMoveCmd"].put(0, timeout=1.0)
    t0 = time.time()
    while not PVs["bBusy"].get(timeout=0.7):
        if time.time() - t0 > 4.0:
            raise AssertionError("Axis never went busy after move start.")
        time.sleep(0.1)
    if do_halt:
        print("[HALT] Issuing HALT mid-move.")
        PVs["bHalt"].put(1, timeout=1.0)
        time.sleep(0.25)
        PVs["bHalt"].put(0, timeout=1.0)
    else:
        print("[RESET] Issuing RESET mid-move.")
        PVs["bReset"].put(1, timeout=1.0)
        time.sleep(0.25)
        PVs["bReset"].put(0, timeout=1.0)
    wait_pv_flag("bDone", 1, timeout=10.0)
    assert_true(PVs["bDone"].get(timeout=1.0) == 1, f"bDone was set after {'HALT' if do_halt else 'RESET'}.")
    print(f"[PASS] {('HALT' if do_halt else 'RESET')} validated: bDone set.")

def batch_move_tests(num_moves=5):
    print("[STEP] Running batch of {} random absolute moves and events".format(num_moves))
    for i in range(num_moves):
        movetype = random.choice(['normal', 'error', 'halt', 'reset'])
        offset = random.choice([-1,1]) * random.uniform(5.0, 14.0)
        print(f" [BATCH MOVE] #{i+1}: {movetype}, offset={offset:.2f}")
        if movetype == 'normal':
            move_abs_offset(offset)
        elif movetype == 'error':
            current = PVs["fActPosition"].get(timeout=1.0)
            target = current + offset
            PVs["fPosition"].put(target, timeout=1.0)
            PVs["fVelocity"].put(0.0, timeout=1.0)
            PVs["fAcceleration"].put(500.0, timeout=1.0)
            PVs["fDeceleration"].put(500.0, timeout=1.0)
            PVs["nCommand"].put(1, timeout=1.0)
            PVs["bMoveCmd"].put(1, timeout=1.0)
            time.sleep(2)
            PVs["bMoveCmd"].put(0, timeout=1.0)
            time.sleep(1)
            err = PVs["bError"].get(timeout=1)
            assert_true(err, "Should get NC error for zero velocity")
            nerror = PVs["nErrorId"].get(timeout=1)
            assert_true(nerror == 17241, f"Batch move: expected NC error code 17241, got {nerror}")
            PVs["bReset"].put(1, timeout=1.0)
            time.sleep(2)
            # PVs["bReset"].put(0, timeout=1.0)
            # wait_pv_flag("bError", 0, timeout=6.0)
            # # Retry with valid params
            # PVs["fVelocity"].put(20.0, timeout=1.0)
            # PVs["bMoveCmd"].put(1, timeout=1.0)
            # time.sleep(2)
            # PVs["bMoveCmd"].put(0, timeout=1.0)
            # wait_move_and_check(target)
        elif movetype == 'halt':
            test_halt_or_reset(offset, do_halt=True)
        elif movetype == 'reset':
            test_halt_or_reset(offset, do_halt=False)
        else:
            assert False, "Unknown move type"

# --- Main sequence starts here
check_connected()
print("[INFO] All key PVs connected.")
print("[STEP] Performing initial reset...")
PVs["bReset"].put(1, timeout=1.0); time.sleep(0.3); PVs["bReset"].put(0, timeout=1.0); time.sleep(0.3)
err = PVs["bError"].get(timeout=1.0)
errmsg = PVs["sErrorMessage"].get(as_string=True, timeout=1.0)
assert_false(err, f"Error after initial reset. Message: {errmsg}")
print('[PASS] Initial reset: no error, proceeding with moves.')

initial_pos = PVs["fActPosition"].get(timeout=2.0)
print(f"[INFO] Initial axis position: {initial_pos:.2f}")

move_abs_offset(15.0)
move_abs_offset(-8.0)
move_abs_offset(6.0)

print("[STEP] Test HALT during positive move")
test_halt_or_reset(8.0, do_halt=True)

print("[STEP] Test RESET during negative move")
test_halt_or_reset(-6.0, do_halt=False)


print("[STEP] Backlash compensation ON, forward (expect 0)")
PVs["bBacklashEnable"].put(1, timeout=1.0)
PVs["fBacklashComp"].put(1.7, timeout=1.0)
time.sleep(2.0)
move_abs_offset(7.0, expect_comp=0.0)

print("[STEP] Backlash compensation ON, reverse (expect 1.7)")
move_abs_offset(-7.0, expect_comp=1.7)

print("[STEP] Backlash compensation OFF, forward (expect 0)")
PVs["bBacklashEnable"].put(0, timeout=1.0)
time.sleep(2.0)
move_abs_offset(6.0, expect_comp=0.0)

print("[STEP] Backlash compensation ON, forward (expect -0.7 on next reverse only)")
PVs["bBacklashEnable"].put(1, timeout=1.0)
PVs["fBacklashComp"].put(-0.7, timeout=1.0)
time.sleep(2.0)
move_abs_offset(-10.0, expect_comp=0.0)
move_abs_offset(6.0, expect_comp=0.7)

print("[STEP] All halt/reset/backlash compensation tests passed.")

batch_move_tests(num_moves)
print("[PASS] Batch absolute move, error, halt, and reset simulation complete.")