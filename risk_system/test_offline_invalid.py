#!/usr/bin/env python3
"""Offline checks of the detector's safety rules (no ROS):
uniform image, wrong size, blacked-out image, and a valid frame."""
import sys
import cv2
import numpy as np
sys.path.insert(0, '/workspace/risk_system')
import risk_core as rc

ok = True
cases = {
    'uniform_218 (old sky-only bug)': np.full((480, 640, 3), 218, np.uint8),
    'black': np.zeros((480, 640, 3), np.uint8),
    'wrong_size': np.random.randint(0, 255, (240, 320, 3), np.uint8),
}
for name, img in cases.items():
    r = rc.RiskEstimator().process(img)
    good = r['state'] == rc.STATE_INVALID and r['risk'] == 100.0
    ok &= good
    print(f"{'PASS' if good else 'FAIL'} {name}: state={r['state']} reason={r['reason']}")
img = cv2.cvtColor(cv2.imread('/workspace/risk_system/snap_v2_start.png'), cv2.COLOR_BGR2RGB)
r = rc.RiskEstimator().process(img)
good = r['state'] != rc.STATE_INVALID
ok &= good
print(f"{'PASS' if good else 'FAIL'} real frame: state={r['state']} risk={r['risk']}")
sys.exit(0 if ok else 1)
