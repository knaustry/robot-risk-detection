#!/bin/bash
# Crops scene images from recordings (no re-simulation). Run inside pai_ros2.
V=/workspace/risk_system/videos; R=$V/raw_slalom_final5.mp4; O=$V/raw_demo_20261018_142401.mp4
S=/workspace/report_materials/scene_images
rm -rf $S/robot_camera_detection $S/video_composites
mkdir -p $S/environment $S/robot_camera_detection $S/video_composites $S/before_after $S/limitations
mv -f $S/01_slalom_world_overview_clean.png $S/environment/ 2>/dev/null; mv -f $S/02_slalom_robot_camera_clean.png $S/environment/ 2>/dev/null
cp -p /workspace/risk_system/snap_v3_overview.png $S/environment/03_plain_world_overview.png
ffmpeg -v error -y -ss 10 -i $O -frames:v 1 -vf crop=800:450:0:0 $S/environment/04_office_world_overview_from_video.png
i=1
for spec in 5:normal_drive 9:first_stop_and_turn 20:passing_obstacle1 28:toward_obstacle2 37:wait_for_person 42:resume 52:toward_obstacle3 62:goal; do
  t=${spec%%:*}; n=${spec##*:}; k=$(printf %02d $i)
  ffmpeg -v error -y -ss $t -i $R -frames:v 1 $S/video_composites/${k}_t${t}s_$n.png
  ffmpeg -v error -y -ss $t -i $R -frames:v 1 -vf crop=480:360:800:0 $S/robot_camera_detection/${k}_t${t}s_$n.png
  i=$((i+1))
done
cp -p /workspace/robot_risk_camera.png $S/before_after/01_before_fix_uniform_gray.png
cp -p /workspace/risk_system/robot_risk_camera_after_fix.png $S/before_after/02_after_caster_fix.png
cp -p /workspace/risk_system/slalom_stuck.png $S/limitations/01_false_person_case_and_plant_try2.png
