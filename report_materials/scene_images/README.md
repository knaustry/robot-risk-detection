# scene_images の内容

動画から切り出した画像は、録画時のシミュレーション画像と表示をそのまま含みます。動画中の数値は録画ノードの表示値で、`experiment_data.csv` の値とは記録方法とサンプリング時刻が異なります。**レポートの数値には使わず、CSVの値を引用してください。**

## 動画とログの時刻

`raw_slalom_final5.mp4` の再生時刻は、ログの t_rel_s より約2.3秒進んでいます。動画内のイベント表示（人が歩き始める：30.7 s）と、ログの `person_walk_start`（28.38 s）の差から求めた概算です。

## environment（環境）

| ファイル | 内容 | 出どころ |
|---|---|---|
| 01_slalom_world_overview_clean.png | スラロームのワールド全景、文字なし | 実行中のGazeboの `/robot_risk/overview_camera` を2026-10-31に取得 |
| 02_slalom_robot_camera_clean.png | 出発点の搭載カメラ、文字なし | 同時に `/robot_risk/camera` を取得 |
| 03_plain_world_overview.png | 基本ワールドの全景 | `risk_system/snap_v3_overview.png` |
| 04_office_world_overview_from_video.png | オフィスのワールド全景 | `raw_demo_20261018_142401.mp4` の10 s、外部視点を切り出し。上部に文字あり |

## robot_camera_detection（搭載カメラと認識結果）

`raw_slalom_final5.mp4` の右上を切り出したものです。

| ファイル | 動画の時刻 | 場面 |
|---|---|---|
| 01_t5s_normal_drive.png | 5 s | 通常走行 |
| 02_t9s_first_stop_and_turn.png | 9 s | 最初の障害物で停止した直後 |
| 03_t20s_passing_obstacle1.png | 20 s | 1個目の障害物（段ボール箱）の横を右側へ迂回中 |
| 04_t28s_toward_obstacle2.png | 28 s | 1個目を通過し、2個目（ケース）を左へ避けに向かう |
| 05_t37s_wait_for_person.png | 37 s | 人の前で待機 |
| 06_t42s_resume.png | 42 s | 人を待った後に再開して走行中 |
| 07_t52s_toward_obstacle3.png | 52 s | 3個目の障害物へ向けて迂回中 |
| 08_t62s_goal.png | 62 s | 目標地点 |

## video_composites（動画の全画面）

上と同じ時刻の全画面です（1280×720）。外部視点、搭載カメラ、状態表示、グラフを含みます。

## before_after（修正の前後）

| ファイル | 内容 | 確認した値 |
|---|---|---|
| 01_before_fix_uniform_gray.png | ロボットが傾き空しか映らない時の搭載カメラ | 全画素218、標準偏差0 |
| 02_after_caster_fix.png | キャスター追加後の搭載カメラ | 最小30、最大218、標準偏差59.14 |

## limitations（限界の例）

| ファイル | 内容 |
|---|---|
| 01_false_person_case_and_plant_try2.png | ケースと奥の植物が重なり「person」と誤判定（try2の停止後の画面。ファイル取得時刻は走行中ではない） |
| 02_person_label_switch_final5.png | 採用ランで、近くの人のラベルが切り替わる時間帯（動画37.8 s）。ログでは t≈35 s 以降、約1.85 mの物体が obstacle になっている |
