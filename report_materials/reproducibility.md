# 再現手順

コマンドはすべてMacのターミナルで実行します。パスは次の対応です。

| Mac | コンテナ |
|---|---|
| `~/Desktop/261001_pai_ros2_no2` | `/workspace` |

## 開発環境（実測）

| 項目 | 値 |
|---|---|
| ホスト | macOS 27.0.1（arm64）、Docker 29.7.2 |
| コンテナイメージ | `airobotbook/ros2-desktop-ai-robot-book-humble:latest`（`sha256:0380d72bdbae44bf1714ccf6f725b7409e336f36ac27af7a7425864acbd8ccc0`） |
| コンテナ内OS | Ubuntu 22.04.5 LTS（x86_64、Rosettaで動作） |
| コンテナの資源 | 8 CPU、メモリ 7.8 GB |
| ROS 2 | Humble、rclpy 3.3.15、ros_gz_bridge 0.244.16、rmw_fastrtps_cpp |
| シミュレータ | Ignition Gazebo 6.16.0（ogre2、ヘッドレス） |
| Python | 3.10.12 |
| ライブラリ | OpenCV 4.5.4、NumPy 1.24.4、Pillow 9.0.1 |
| 動画 | ffmpeg 4.4.2（libx264） |
| グラフのみ | matplotlib 3.5.1、Noto Sans CJK JP |
| `pose_client` のビルド | g++、ignition-transport11、ignition-msgs8（イメージに含まれる） |

追加のインストールはしていません。`ultralytics`（YOLO）は入っていますが、このシステムでは使いません。

## 1. コンテナの起動

作業フォルダで実行します（`run_pai_container.sh`）。

```bash
bash run_pai_container.sh
```

すでにあれば次のコマンドで起動します。

```bash
docker start pai_ros2
```

## 2. 必要なファイルの生成

スラローム用のワールドと `pose_client` は作成済みです。作り直す時だけ実行します。

```bash
docker exec pai_ros2 bash -c 'cd /workspace/risk_system && python3 make_office_slalom_world.py && ign sdf -k /workspace/robot_risk_office_slalom.sdf'
```
```bash
docker exec pai_ros2 bash -c 'cd /workspace/risk_system && g++ -O1 -std=c++17 pose_client.cpp -o pose_client $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8)'
```

## 3. シミュレーションの起動

別のGazeboが動いていれば先に止めます。

```bash
docker exec pai_ros2 bash /workspace/risk_system/stop_all.sh --all
```
```bash
docker exec -e WORLD=/workspace/robot_risk_office_slalom.sdf pai_ros2 bash /workspace/risk_system/start_sim.sh
```

## 4. 実験の実行（採用ランと同じ設定）

名前 `myrun` は好きなものに変えられます。同じ名前のログは上書きされます。

```bash
docker exec pai_ros2 bash /workspace/risk_system/run_slalom.sh myrun record
```

結果の出力先：

| 種類 | パス |
|---|---|
| ログ | `risk_system/logs/slalom_myrun.csv`、`.summary.txt`、`.controller.log`、`.scenario.log` |
| 動画の素材 | `risk_system/videos/raw_slalom_myrun.mp4` |
| 設定値の記録 | `risk_system/logs/slalom_runs.txt` に追記 |

採用ランの設定は、`run_slalom.sh` の既定値と同じです。

| 設定 | 値 | 設定 | 値 |
|---|---|---|---|
| GOAL / GOAL_Y | 11.0 / −0.4 m | V_MIN | 0.15 m/s |
| HOLD | 1.0 s | V_PASS | 0.22 m/s |
| TRIG | 5.0 m | W_TURN | 0.6 rad/s |
| WALK | 0.4 m/s | W_MAX | 0.7 rad/s |
| V_MAX | 0.30 m/s | SIDE_LAT | 1.0 m |
| V_DETOUR | 0.25 m/s | DEPTH | 1.0 m |
| V_PERSON | 0.12 m/s | MEMORY | on |

## 5. 動画の変換

```bash
docker exec pai_ros2 bash -c 'cd /workspace/risk_system/videos && ffmpeg -y -i raw_slalom_myrun.mp4 -c:v libx264 -preset medium -crf 20 -pix_fmt yuv420p -r 10 -movflags +faststart myrun.mp4'
```

## 6. 個別の測定

コントローラを止めて行います。`test_faults.py` は既定の遅い速度で動くので、0.30 m/sの試験には次の環境変数を付けました。

```bash
docker exec pai_ros2 bash -c 'source /opt/ros/humble/setup.bash; cd /workspace/risk_system; python3 test_speed_metrics.py'
```
```bash
docker exec -e FAULT_LOG=slalom_fault_tests.txt -e T4_MIN_V=0.25 -e 'CTRL_PARAMS=-p v_max:=0.30 -p v_detour:=0.25 -p v_person:=0.12 -p v_min_slow:=0.15 -p v_pass:=0.22 -p w_turn:=0.6 -p w_max:=0.7 -p person_side_lat:=1.0 -p obst_depth:=1.0 -p use_memory:=true -p person_hold:=1.0' pai_ros2 bash -c 'source /opt/ros/humble/setup.bash; cd /workspace/risk_system; python3 test_faults.py'
```

距離と人の判定の試験（オフィスの配置で実施。スラロームのワールドでは obstacle_1 と人の位置が異なるので、そのまま使えません）：

```bash
docker exec pai_ros2 bash /workspace/risk_system/stop_all.sh --all
```
```bash
docker exec -e WORLD=/workspace/robot_risk_office.sdf pai_ros2 bash /workspace/risk_system/start_sim.sh
```
```bash
docker exec pai_ros2 bash -c 'source /opt/ros/humble/setup.bash; cd /workspace/risk_system; bash start_nodes.sh --no-controller; sleep 6; python3 test_detection_sweep.py'
```

この試験は `logs/detection_sweep.csv` を上書きします。`run_office_validation.sh` は既存の動画 `robot_risk_demo_office.mp4` も上書きするので、使わないでください。

## 7. この資料の再生成

```bash
docker exec pai_ros2 python3 /workspace/report_materials/tools/build_data_and_graphs.py
```
```bash
docker exec pai_ros2 bash /workspace/report_materials/tools/crop_scene_images.sh
```

## 再現性の確認状況と注意

- **確認済み：** 同じ配置（L3）で、ほぼ同じ設定の4回（try6、final3、final4、final5）がすべて目標に到達しました。最小距離は0.249〜0.268 m、走行時間は57.6〜59.2 sです。final5の設定そのものでの実行は1回だけです。
- **未確認：** ほかのマシンでの実行。実時間比（RTF、計測値0.46〜0.50）が変わると、実時間で判定するタイムアウトの効き方が変わり、結果も変わる可能性があります。
- CPU負荷が高いと結果が変わる場合があります。古い検出器などのプロセスは止めてください（`status.sh` で確認できます）。
- 乱数は使っていませんが、プロセス間のタイミングにより毎回少し変わります。`latest` タグのイメージは将来変わる可能性があるので、上記のダイジェストを記録しています。
- コードのバージョン管理はしていません（Gitリポジトリではありません）。主要な変更前のコピーは `risk_system/backup_before_*` にあります。
