#!/usr/bin/env python3
"""Run risk_core on saved PNG frames: offline_test.py in1.png [in2.png ...]"""
import sys, json, cv2
sys.path.insert(0, '/workspace/risk_system')
from risk_core import RiskEstimator, draw

for p in sys.argv[1:]:
    est = RiskEstimator()
    bgr = cv2.imread(p)
    rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    for _ in range(3):          # let the floor model settle like in the live node
        res = est.process(rgb)
    res2 = {k: v for k, v in res.items() if k != 'boundary'}
    print(p, json.dumps(res2, ensure_ascii=False))
    cv2.imwrite(p.replace('.png', '_det.png'), draw(bgr, res))
