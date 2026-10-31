#!/usr/bin/env python3
"""Condensed timeline of a slalom run: slalom_timeline.py <csv>"""
import csv, sys
r = list(csv.DictReader(open(sys.argv[1]))); prev = None; t0 = None
for row in r:
    key = (row['state'], row['mode'], row['reason'].split('(')[0][:18])
    try: st = float(row['sim_t'])
    except ValueError: st = 0
    if st and t0 is None: t0 = st
    if key != prev:
        print(f"sim {st-(t0 or 0):5.1f} x={float(row['x']):+.2f} y={float(row['y']):+.2f} yaw={float(row['yaw']):+4.0f} p={row['pitch']:>5} {str(row['state']):<12} r={row['risk']:>5} {str(row['mode']):<11} {row['reason'][:36]:<36} v={row['v']} cl={row['clearance']} {row['closest']} {row['event']}")
    prev = key
print('min clearance', min((float(x['clearance']), x['closest']) for x in r), 'max|pitch|', max(abs(float(x['pitch'])) for x in r))
