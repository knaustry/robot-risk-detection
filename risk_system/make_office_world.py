#!/usr/bin/env python3
"""Generate /workspace/robot_risk_office.sdf from /workspace/robot_risk_world.sdf.

Kept unchanged (copied verbatim from the control world): world name
"robot_risk_world", all system plugins, the risk_robot model (camera
/robot_risk/camera, DiffDrive /robot_risk/cmd_vel + /robot_risk/odom, casters),
and the names/poses/collision sizes of obstacle_1, obstacle_2, standing_person,
so every script, service and topic keeps working.
Replaced: floor, light, overview camera, obstacle/person visuals.
Added: office corridor (walls, doors, windows, partition, desks, chairs,
monitors, plants, ceiling, ceiling lights).  Primitive shapes only (no
downloads).  The person's arms reuse human_dummy from camera_test_person.sdf.
Usage: make_office_world.py [src] [dst]"""
import re
import sys

SRC = sys.argv[1] if len(sys.argv) > 1 else '/workspace/robot_risk_world.sdf'
DST = sys.argv[2] if len(sys.argv) > 2 else '/workspace/robot_risk_office.sdf'


def mat(r, g, b):
    return (f'<material><ambient>{r} {g} {b} 1</ambient><diffuse>{r} {g} {b} 1</diffuse>'
            f'<specular>0.1 0.1 0.1 1</specular></material>')


SHADOWS_FURNITURE = False   # furniture/partitions: no shadows (render cost)


def box_visual(name, pose, size, rgb, shadows=True):
    cs = '' if shadows else '<cast_shadows>false</cast_shadows>'
    return (f'<visual name="{name}"><pose>{pose}</pose>{cs}<geometry><box><size>{size}</size></box></geometry>'
            f'{mat(*rgb)}</visual>')


def static_box(name, pose, size, rgb, collide=True, shadows=True, emissive=None):
    col = (f'<collision name="c"><geometry><box><size>{size}</size></box></geometry></collision>'
           if collide else '')
    v = box_visual("v", "0 0 0 0 0 0", size, rgb, shadows)
    if emissive is not None:
        v = v.replace('</material>', f'<emissive>{emissive} {emissive} {emissive} 1</emissive></material>')
    return (f'<model name="{name}"><static>true</static><pose>{pose}</pose><link name="link">{col}'
            f'{v}</link></model>\n')


def chair(name, x, y, yaw):
    v = [box_visual('seat', '0 0 0.45 0 0 0', '0.45 0.45 0.06', (0.15, 0.15, 0.17)),
         box_visual('back', '-0.2 0 0.75 0 0 0', '0.05 0.42 0.55', (0.15, 0.15, 0.17)),
         f'<visual name="post"><pose>0 0 0.22 0 0 0</pose><geometry><cylinder><radius>0.03</radius>'
         f'<length>0.44</length></cylinder></geometry>{mat(0.3, 0.3, 0.3)}</visual>',
         f'<visual name="base"><pose>0 0 0.03 0 0 0</pose><geometry><cylinder><radius>0.28</radius>'
         f'<length>0.04</length></cylinder></geometry>{mat(0.2, 0.2, 0.2)}</visual>']
    v = [x_.replace('<geometry>', '<cast_shadows>false</cast_shadows><geometry>', 1) for x_ in v]
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 {yaw}</pose><link name="link">'
            f'<collision name="c"><pose>0 0 0.45 0 0 0</pose><geometry><box><size>0.5 0.5 0.9</size></box>'
            f'</geometry></collision>{"".join(v)}</link></model>\n')


def desk(name, x, y):
    v = [box_visual('top', '0 0 0.74 0 0 0', '1.4 0.7 0.04', (0.72, 0.55, 0.36)),
         box_visual('leg_l', '-0.66 0 0.36 0 0 0', '0.05 0.66 0.72', (0.35, 0.35, 0.37)),
         box_visual('leg_r', '0.66 0 0.36 0 0 0', '0.05 0.66 0.72', (0.35, 0.35, 0.37)),
         box_visual('monitor', '0 0.22 1.0 0 0 0', '0.55 0.04 0.34', (0.05, 0.05, 0.06)),
         box_visual('screen', '0 0.195 1.0 0 0 0', '0.5 0.01 0.29', (0.25, 0.45, 0.75)),
         box_visual('stand', '0 0.24 0.8 0 0 0', '0.06 0.06 0.12', (0.1, 0.1, 0.1))]
    v = [x_.replace('<geometry>', '<cast_shadows>false</cast_shadows><geometry>', 1) for x_ in v]
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 0</pose><link name="link">'
            f'<collision name="c"><pose>0 0 0.38 0 0 0</pose><geometry><box><size>1.4 0.7 0.76</size></box>'
            f'</geometry></collision>{"".join(v)}</link></model>\n')


def plant(name, x, y):
    return (f'<model name="{name}"><static>true</static><pose>{x} {y} 0 0 0 0</pose><link name="link">'
            f'<collision name="c"><pose>0 0 0.25 0 0 0</pose><geometry><cylinder><radius>0.2</radius>'
            f'<length>0.5</length></cylinder></geometry></collision>'
            f'<visual name="pot"><pose>0 0 0.2 0 0 0</pose><geometry><cylinder><radius>0.18</radius>'
            f'<length>0.4</length></cylinder></geometry>{mat(0.85, 0.85, 0.82)}</visual>'
            f'<visual name="leaves"><pose>0 0 0.75 0 0 0</pose><geometry><sphere><radius>0.32</radius>'
            f'</sphere></geometry>{mat(0.2, 0.5, 0.2)}</visual></link></model>\n')


def point_light(name, x, y):
    return (f'<light name="{name}" type="point"><pose>{x} {y} 2.4 0 0 0</pose><cast_shadows>false</cast_shadows>'
            f'<diffuse>0.55 0.55 0.52 1</diffuse><specular>0.1 0.1 0.1 1</specular>'
            f'<attenuation><range>9</range><constant>0.6</constant><linear>0.05</linear>'
            f'<quadratic>0.01</quadratic></attenuation></light>\n')


s = open(SRC).read()

# --- remove models / lights that the office replaces ---------------------------
for name in ('floor', 'overview_camera'):
    s, n = re.subn(rf'\s*(<!--[^>]*-->\s*)?<model name="{name}">.*?</model>\n', '\n', s, count=1, flags=re.S)
    assert n == 1, name
s, n = re.subn(r'<light name="sun".*?</light>', '', s, count=1, flags=re.S)
assert n == 1

# --- restyle obstacles / person (same names, poses, collision sizes) ------------
# obstacle_1: stacked cardboard boxes (collision 0.35 x 0.8 x 0.9 unchanged)
o1 = re.search(r'<model name="obstacle_1">.*?</model>', s, flags=re.S).group(0)
o1_new = re.sub(r'<visual name="visual">.*?</visual>',
                box_visual('box_low', '0 0 -0.225 0 0 0', '0.35 0.8 0.45', (0.62, 0.45, 0.28)) +
                box_visual('box_high', '0 0.02 0.225 0 0 0.03', '0.34 0.74 0.45', (0.66, 0.49, 0.31)) +
                box_visual('tape', '0 0 0.452 0 0 0', '0.36 0.08 0.004', (0.85, 0.8, 0.6)), o1, flags=re.S)
s = s.replace(o1, o1_new)
# obstacle_2: rolling equipment case (collision 0.6 x 0.6 x 0.7 unchanged)
o2 = re.search(r'<model name="obstacle_2">.*?</model>', s, flags=re.S).group(0)
o2_new = re.sub(r'<visual name="visual">.*?</visual>',
                box_visual('case', '0 0 0 0 0 0', '0.6 0.6 0.7', (0.12, 0.12, 0.14)) +
                box_visual('handle', '0 0 0.37 0 0 0', '0.3 0.05 0.04', (0.5, 0.5, 0.5)), o2, flags=re.S)
s = s.replace(o2, o2_new)
# person: add arms (geometry reused from human_dummy in camera_test_person.sdf,
# scaled to this torso) and a light office shirt
p = re.search(r'<model name="standing_person">.*?</model>', s, flags=re.S).group(0)
p_new = p.replace('<ambient>0.15 0.3 0.85 1</ambient>', '<ambient>0.55 0.7 0.9 1</ambient>') \
         .replace('<diffuse>0.15 0.3 0.85 1</diffuse>', '<diffuse>0.55 0.7 0.9 1</diffuse>')
arms = (box_visual('left_arm', '0 0.16 0.88 0 0 0', '0.09 0.08 0.58', (0.55, 0.7, 0.9)) +
        box_visual('right_arm', '0 -0.16 0.88 0 0 0', '0.09 0.08 0.58', (0.55, 0.7, 0.9)))
p_new = p_new.replace('</link>', arms + '</link>', 1)
s = s.replace(p, p_new)

# --- office -----------------------------------------------------------------------
X0, X1 = -7.0, 7.0            # corridor length
YS, YN = -2.6, 2.1            # south wall / north partition
office = ['\n    <!-- ===== Office (generated by risk_system/make_office_world.py) ===== -->\n',
          '<light name="sun" type="directional"><pose>0 0 8 0 0 0</pose><cast_shadows>true</cast_shadows>'
          '<visualize>false</visualize>'
          '<direction>0.02 0.01 -1</direction><diffuse>0.6 0.6 0.58 1</diffuse>'
          '<specular>0.15 0.15 0.15 1</specular></light>\n']
# (point lights removed: they cut the real-time factor from ~0.88 to ~0.35;
#  ambient scene light + emissive ceiling panels give the indoor look cheaply)
# floor: corridor + open-office area (uniform light tile colour; top surface z=0)
office.append(static_box('floor', f'0 0.7 -0.05 0 0 0', f'{X1 - X0} 6.8 0.1', (0.50, 0.49, 0.46)))
# ceiling: visual only, no shadows (so the sun still lights the scene)
office.append(static_box('ceiling', '0 0.7 2.55 0 0 0', f'{X1 - X0} 6.8 0.1', (0.93, 0.93, 0.92),
                         collide=False, shadows=False, emissive=0.45))
for i, x in enumerate(range(-6, 7, 2)):     # ceiling light panels (visual)
    office.append(static_box(f'light_panel_{i}', f'{x} 0 2.495 0 0 0', '0.6 0.6 0.01', (1, 1, 0.97),
                             collide=False, shadows=False, emissive=1.0))
# walls (no shadows: keeps the corridor floor evenly lit)
W = (0.93, 0.93, 0.91)
office.append(static_box('wall_south', f'0 {YS - 0.05} 1.25 0 0 0', f'{X1 - X0} 0.1 2.5', W, shadows=False, emissive=0.3))
office.append(static_box('wall_west', f'{X0 - 0.05} 0.7 1.25 0 0 0', '0.1 6.8 2.5', W, shadows=False, emissive=0.3))
office.append(static_box('wall_east', f'{X1 + 0.05} 0.7 1.25 0 0 0', '0.1 6.8 2.5', W, shadows=False, emissive=0.3))
office.append(static_box('wall_north_office', f'0 4.15 1.25 0 0 0', f'{X1 - X0} 0.1 2.5', W, shadows=False, emissive=0.3))
# skirting board + doors + window band on the south wall (visual details)
office.append(static_box('skirting_south', f'0 {YS + 0.005} 0.05 0 0 0', f'{X1 - X0} 0.01 0.1',
                         (0.35, 0.35, 0.37), collide=False, shadows=False))
for i, x in enumerate((-4.0, 0.5, 4.5)):
    office.append(static_box(f'door_{i}', f'{x} {YS + 0.01} 1.05 0 0 0', '0.9 0.02 2.1',
                             (0.55, 0.42, 0.3), collide=False, shadows=False))
    office.append(static_box(f'door_handle_{i}', f'{x + 0.35} {YS + 0.03} 1.0 0 0 0', '0.12 0.03 0.03',
                             (0.7, 0.7, 0.72), collide=False, shadows=False))
for i, x in enumerate((-2.0, 2.5)):
    office.append(static_box(f'window_{i}', f'{x} {YS + 0.01} 1.5 0 0 0', '1.6 0.02 0.9',
                             (0.62, 0.74, 0.82), collide=False, shadows=False))
office.append(static_box('door_east', f'{X1 - 0.01} 0 1.05 0 0 0', '0.02 1.0 2.1', (0.55, 0.42, 0.3),
                         collide=False, shadows=False))
# north: low partition (1.2 m) with a gap, open-office desks behind it
office.append(static_box('partition_a', f'-3.6 {YN + 0.04} 0.6 0 0 0', '6.8 0.08 1.2', (0.75, 0.78, 0.8), shadows=False))
office.append(static_box('partition_b', f'3.9 {YN + 0.04} 0.6 0 0 0', '6.2 0.08 1.2', (0.75, 0.78, 0.8), shadows=False))
for i, x in enumerate((-5.2, -3.4, -1.6, 2.4, 4.2)):
    office.append(desk(f'desk_{i}', x, 3.1))
    office.append(chair(f'chair_{i}', x, 2.55, 1.5708))
office.append(plant('plant_0', -6.6, -2.25))
office.append(plant('plant_1', 6.6, -2.25))
office.append(plant('plant_2', 0.35, 2.45))
office.append(static_box('cabinet_0', '6.7 3.4 0.6 0 0 0', '0.5 1.2 1.2', (0.6, 0.62, 0.65), shadows=False))
office.append(static_box('water_cooler', '-6.7 1.6 0.55 0 0 0', '0.35 0.35 1.1', (0.9, 0.9, 0.92), shadows=False))
# external "CCTV" camera in the ceiling corner (video only, NOT the robot camera)
office.append('''
    <model name="overview_camera">
      <static>true</static>
      <pose>-6.75 -2.35 2.35 0 0.36 0.36</pose>
      <link name="link">
        <sensor name="overview" type="camera">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <topic>/robot_risk/overview_camera</topic>
          <camera>
            <horizontal_fov>1.35</horizontal_fov>
            <image><width>960</width><height>540</height><format>R8G8B8</format></image>
            <clip><near>0.1</near><far>40</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
''')
s = s.replace('<world name="robot_risk_world">',
              '<world name="robot_risk_world">\n    <scene><ambient>0.55 0.55 0.55 1</ambient>'
              '<background>0.8 0.8 0.8 1</background><shadows>true</shadows></scene>', 1)
s = s.replace('\n  </world>', ''.join('    ' + o if not o.startswith('\n') else o for o in office) + '\n  </world>', 1)
open(DST, 'w').write(s)
print('written', DST)
