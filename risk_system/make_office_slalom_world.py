#!/usr/bin/env python3
"""Generate /workspace/robot_risk_office_slalom.sdf from robot_risk_office.sdf
(source is not modified). Same world name, robot, plugins and topics.
Changes: corridor extended to x=8.5; obstacle_1/2 moved, obstacle_3 added
(alternating sides); person starts beside the corridor to walk across it."""
import copy, json, sys
import xml.etree.ElementTree as ET
SRC = '/workspace/robot_risk_office.sdf'; DST = '/workspace/robot_risk_office_slalom.sdf'
X0, X1 = -7.0, 8.5
OBST = {'obstacle_1': (-1.4, 0.4), 'obstacle_2': (1.4, -1.0), 'obstacle_3': (5.0, 0.55)}
PERSON0 = (3.2, -2.2)
t = ET.parse(SRC); w = t.getroot().find('world')
M = {m.get('name'): m for m in w.findall('model')}
def pose(m, x=None, y=None):
    p = m.find('pose'); v = p.text.split()
    if x is not None: v[0] = str(x)
    if y is not None: v[1] = str(y)
    p.text = ' '.join(v)
def resize_x(m, L):
    for s in m.iter('size'):
        v = s.text.split(); v[0] = str(L); s.text = ' '.join(v)
c, L = (X0 + X1) / 2, X1 - X0
for n in ('floor', 'ceiling', 'wall_south', 'wall_north_office', 'skirting_south'):
    pose(M[n], x=c); resize_x(M[n], L)
pose(M['wall_east'], x=X1 + 0.05); pose(M['door_east'], x=X1 - 0.01)
pose(M['plant_1'], x=8.1); pose(M['cabinet_0'], x=8.2)
pb0, pb1 = 0.8, X1
pose(M['partition_b'], x=(pb0 + pb1) / 2); resize_x(M['partition_b'], pb1 - pb0)
def clone(src, name, x=None, y=None):
    m = copy.deepcopy(M[src]); m.set('name', name); pose(m, x, y); w.append(m); M[name] = m
clone('desk_4', 'desk_5', x=6.0); clone('chair_4', 'chair_5', x=6.0)
clone('door_2', 'door_3', x=7.4); clone('door_handle_2', 'door_handle_3', x=7.75)
clone('window_1', 'window_2', x=6.2)
for i, x in enumerate((8,)):
    clone('light_panel_6', f'light_panel_{7 + i}', x=x)
# obstacle_3: tall storage bin (0.5 x 0.5 x 0.8), blue-grey
o3 = ET.fromstring('''<model name="obstacle_3"><static>true</static><pose>0 0 0.4 0 0 0</pose><link name="link">
<collision name="collision"><geometry><box><size>0.5 0.5 0.8</size></box></geometry></collision>
<visual name="bin"><geometry><box><size>0.5 0.5 0.8</size></box></geometry>
<material><ambient>0.2 0.33 0.5 1</ambient><diffuse>0.2 0.33 0.5 1</diffuse></material></visual>
<visual name="lid"><pose>0 0 0.41 0 0 0</pose><geometry><box><size>0.54 0.54 0.03</size></box></geometry>
<material><ambient>0.1 0.15 0.25 1</ambient><diffuse>0.1 0.15 0.25 1</diffuse></material></visual>
</link></model>'''); w.append(o3); M['obstacle_3'] = o3
for n, (x, y) in OBST.items(): pose(M[n], x, y)
pose(M['standing_person'], *PERSON0)
ET.indent(t) if hasattr(ET, 'indent') else None
t.write(DST, xml_declaration=True, encoding='unicode')
# static footprint list for the clearance logger (x0,x1,y0,y1)
S = {'obstacle_1': (-1.575, -1.225, 0.0, 0.8), 'obstacle_2': (1.1, 1.7, -1.3, -0.7),
     'obstacle_3': (4.75, 5.25, 0.3, 0.8), 'wall_south': (X0, X1, -2.7, -2.6),
     'partition_a': (-7.0, -0.2, 2.1, 2.18), 'partition_b': (pb0, pb1, 2.1, 2.18), 'wall_east': (X1, X1 + 0.1, -2.7, 4.1)}
json.dump({'boxes': S, 'person0': PERSON0}, open('/workspace/risk_system/scene_slalom.json', 'w'), indent=1)
print('written', DST)
