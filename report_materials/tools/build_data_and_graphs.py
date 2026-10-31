#!/usr/bin/env python3
"""Builds experiment_data.csv and the two graphs ONLY from risk_system/logs/*.csv.
Run inside the pai_ros2 container: python3 /workspace/report_materials/tools/build_data_and_graphs.py"""
import csv, math
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib import font_manager as fm
FONT = '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc'
fm.fontManager.addfont(FONT)
plt.rcParams['font.family'] = fm.FontProperties(fname=FONT).get_name()
LOG, OUT = '/workspace/risk_system/logs/', '/workspace/report_materials/'
O, S = 'robot_risk_office.sdf', 'robot_risk_office_slalom.sdf'
RUNS = [  # run_id, csv, world, layout, outcome (from logs/slalom_runs.txt and the CSV itself)
 ('slalom_final5', 'slalom_final5.csv', S, 'L3', 'goal_reached_adopted'),
 ('slalom_final4', 'slalom_final4.csv', S, 'L3', 'goal_reached_not_adopted'),
 ('slalom_final3', 'slalom_final3.csv', S, 'L3', 'goal_reached_not_adopted'),
 ('slalom_try6', 'slalom_try6.csv', S, 'L3', 'goal_reached_not_recorded'),
 ('slalom_try5', 'slalom_try5.csv', S, 'L3', 'goal_reached_not_recorded'),
 ('slalom_try4', 'slalom_try4.csv', S, 'L2', 'rejected_clearance_below_zero'),
 ('slalom_try3', 'slalom_try3.csv', S, 'L1', 'failed_stalled_safely'),
 ('slalom_try2', 'slalom_try2.csv', S, 'L1', 'failed_stalled_safely'),
 ('office_20261018_142401', 'demo_run_20261018_142401.csv', O, 'office', 'goal_reached_earlier_code'),
 ('plain_20261018_133839', 'demo_run_20261018_133839.csv', 'robot_risk_world.sdf', 'plain', 'goal_reached_earlier_code'),
]
COLS = ['run_id','world_file','layout','run_outcome','wall_time_s','sim_stamp_s','t_rel_s','gt_x_m','gt_y_m','gt_pitch_deg','gt_yaw_deg',
        'risk_state','risk_score_0_100','nearest_in_path_est_m','detected_objects_est','control_mode','control_reason',
        'cmd_linear_mps','cmd_angular_radps','clearance_gt_m','closest_object','event']
def blank(v):
    return '' if v in (None, 'None') else v
out = []; final5 = []
for rid, fn, world, layout, outcome in RUNS:
    rows = list(csv.DictReader(open(LOG + fn)))
    stamps = [float(r['sim_t']) for r in rows if r['sim_t'] not in ('', None) and float(r['sim_t']) > 0]
    t0 = stamps[0] if stamps else None
    for r in rows:
        sim = float(r['sim_t']) if r['sim_t'] not in ('', None) else 0.0
        has = sim > 0 and t0 is not None      # sim_t 0 = no stamp logged -> left blank, not estimated
        d = [rid, world, layout, outcome, r['wall'], r['sim_t'] if has else '', f'{sim - t0:.3f}' if has else '', r['x'], r['y'], r['pitch'], r['yaw'],
             blank(r['state']), blank(r['risk']), blank(r['nearest_in_path']), blank(r['objects']), blank(r['mode']), blank(r['reason']),
             blank(r['v']), blank(r['w']), r['clearance'], r['closest'], blank(r['event'])]
        out.append(d)
        if rid == 'slalom_final5': final5.append(dict(zip(COLS, d)))
with open(OUT + 'experiment_data.csv', 'w', newline='', encoding='utf-8') as f:
    w = csv.writer(f); w.writerow(COLS); w.writerows(out)
print('experiment_data.csv rows', len(out))

rows = [r for r in final5 if r['t_rel_s'] != '']
T = [float(r['t_rel_s']) for r in rows]
ev = {r['event']: float(r['t_rel_s']) for r in rows if r['event']}
INK, MUTED, GRID = '#0b0b0b', '#52514e', '#e4e3df'
BLUE, ORANGE = '#2a78d6', '#eb6834'
GRAYS = {'GOAL': '#d9d8d3', 'DETOUR': '#8f8e88', 'WAIT_PERSON': '#2e2d2b'}
NAMES = {'GOAL': 'GOAL（目標へ走行）', 'DETOUR': 'DETOUR（迂回）', 'WAIT_PERSON': 'WAIT_PERSON（人を待機）'}
def spans(rows):
    res, cur, st = [], None, None
    for r in rows:
        t = float(r['t_rel_s']); m = r['control_mode']
        if m != cur:
            if cur: res.append((cur, st, t))
            cur, st = m, t
    if cur: res.append((cur, st, float(rows[-1]['t_rel_s'])))
    return res
SP = spans(rows)
def figure(ylabel, ymax, title, TP=10):
    fig, (ax, st) = plt.subplots(2, 1, figsize=(11, 5.9), dpi=160, sharex=True, gridspec_kw={'height_ratios': [12, 1], 'hspace': 0.06})
    fig.patch.set_facecolor('white')
    ax.set_title(title, loc='left', fontsize=13, color=INK, pad=TP)
    ax.set_ylabel(ylabel, fontsize=11, color=INK); ax.set_ylim(0, ymax); ax.set_xlim(0, max(T))
    ax.grid(axis='y', color=GRID, lw=0.8); ax.set_axisbelow(True)
    for s in ('top', 'right'): ax.spines[s].set_visible(False)
    for s in ('left', 'bottom'): ax.spines[s].set_color('#b5b4ae')
    ax.tick_params(colors=MUTED, labelsize=10)
    for name, lab in (('person_walk_start', '人が歩き始める'), ('person_walk_end', '人が横断を終える')):
        if name in ev:
            ax.axvline(ev[name], color=MUTED, lw=1, ls=(0, (4, 3)))
            ax.text(ev[name] + 0.3, ymax * 0.985, lab, fontsize=9, color=MUTED, va='top')
    for m, a, b in SP:
        if m in GRAYS: st.barh(0, b - a, left=a, color=GRAYS[m], height=1, edgecolor='white', lw=0.8)
    st.set_yticks([0]); st.set_yticklabels(['制御モード'], fontsize=10, color=INK); st.set_ylim(-0.5, 0.5)
    for s in st.spines.values(): s.set_visible(False)
    st.tick_params(axis='y', length=0); st.tick_params(axis='x', colors=MUTED, labelsize=10)
    st.set_xlabel('シミュレーション時間 t [s]（t = 0：ログ内で最初に検知結果を受信した時刻）', fontsize=11, color=INK)
    from matplotlib.patches import Patch
    st.legend(handles=[Patch(color=GRAYS[k], label=NAMES[k]) for k in GRAYS], loc='upper center', bbox_to_anchor=(0.5, -2.1), ncol=3, frameon=False, fontsize=9.5)
    return fig, ax
CAP = ('データ：risk_system/logs/slalom_final5.csv（採用ラン。ROS 2トピックのログで、動画の表示値ではない）。\n{}')
# ---- risk
TOP = 0.92
pts = [(float(r['t_rel_s']), float(r['risk_score_0_100'])) for r in rows if r['risk_score_0_100'] != '']
fig, ax = figure('危険度スコア [0〜100、単位なし]', 105, '図：時間経過に対する危険度スコア（スラローム版オフィス環境）')
ax.plot([p[0] for p in pts], [p[1] for p in pts], color=BLUE, lw=1.6, solid_joinstyle='round')
for y, lab in ((30, '減速開始 30'), (70, '停止 70')):
    ax.axhline(y, color='#8f8e88', lw=1)
    ax.text(max(T) - 0.3, y + 1.2, lab, ha='right', fontsize=9.5, color=MUTED)
fig.text(0.09, 0.008, CAP.format('スコアは検出物体ごとの推定リスクの最大値。単眼カメラの推定に基づく相対値で、衝突確率ではない。迂回中（DETOUR）は70以上でも幾何チェックが通れば走行する。'), fontsize=8.5, color=MUTED)
fig.subplots_adjust(bottom=0.23, top=TOP, left=0.09, right=0.98)
fig.savefig(OUT + 'risk_score_graph.png'); plt.close(fig)
# ---- speed
TOP = 0.84
cmd = [(float(r['t_rel_s']), float(r['cmd_linear_mps'])) for r in rows if r['cmd_linear_mps'] != '']
der = []
for i, r in enumerate(rows):
    t = float(r['t_rel_s'])
    j = i
    while j > 0 and t - float(rows[j]['t_rel_s']) < 2.0: j -= 1
    dt = t - float(rows[j]['t_rel_s'])
    if dt >= 1.9:
        der.append((t, math.hypot(float(r['gt_x_m']) - float(rows[j]['gt_x_m']), float(r['gt_y_m']) - float(rows[j]['gt_y_m'])) / dt))
fig, ax = figure('並進速度 [m/s]', 0.40, '図：時間経過に対するロボット速度（スラローム版オフィス環境）', TP=46)
ax.plot([p[0] for p in cmd], [p[1] for p in cmd], color=BLUE, lw=1.4, label='速度指令（/robot_risk/cmd_vel の値を controller_state で記録）')
ax.plot([p[0] for p in der], [p[1] for p in der], color=ORANGE, lw=1.6, label='Gazebo真値の位置から算出した速度（直近約2秒の移動距離÷時間）')
ax.legend(loc='lower left', bbox_to_anchor=(0.0, 1.0), frameon=False, fontsize=9.5, ncol=1, borderaxespad=0)
fig.text(0.09, 0.008, CAP.format('算出速度は直近2秒で平均するため変化が遅れて見える。位置と時刻は別トピックなので、ばらつきを含む。'), fontsize=8.5, color=MUTED)
fig.subplots_adjust(bottom=0.23, top=TOP, left=0.09, right=0.98)
fig.savefig(OUT + 'robot_speed_graph.png'); plt.close(fig)
print('t range', min(T), max(T), 'risk max', max(p[1] for p in pts), 'cmd max', max(p[1] for p in cmd), 'derived max', round(max(p[1] for p in der), 3))
print('modes', [(m, round(a, 1), round(b, 1)) for m, a, b in SP], 'events', ev)
