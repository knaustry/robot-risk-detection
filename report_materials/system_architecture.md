# システム構成

根拠：`risk_system/` のソースコードと `/workspace/robot_risk_office_slalom.sdf`。行番号は2026-10-31時点のものです。

## 図1：全体構成

各ノードは別プロセスです。実線はROS 2トピックまたはGazeboの通信、点線はテストと動画専用で制御には使いません。

```mermaid
flowchart TB
  subgraph GZ["Ignition Gazebo 6（ヘッドレス）"]
    ROBOT["risk_robot<br/>差動二輪＋キャスター2個"]
    CAM["front_camera<br/>640×480、設定15 Hz"]
    DD["DiffDrive<br/>車輪半径0.12 m、間隔0.42 m"]
    OV["overview_camera<br/>動画専用"]
    SCENE["オフィス環境<br/>障害物3個、人"]
    ROBOT --- CAM
    ROBOT --- DD
  end

  subgraph BR["ros_gz_bridge"]
    BC["bridge_cam<br/>カメラ専用"]
    BM["bridge<br/>odom, cmd_vel, overview"]
    BP["posebridge<br/>真値の姿勢"]
  end

  subgraph NODES["ROS 2 Humble ノード"]
    DET["robot_risk_detector<br/>認識と危険度計算"]
    CTRL["robot_risk_safety_controller<br/>移動制御"]
    WD["robot_risk_cmd_watchdog<br/>指令途絶時に停止"]
  end

  subgraph TOOLS["記録とシナリオ（制御には不使用）"]
    REC["robot_risk_recorder<br/>動画の合成"]
    LOGGER["run_logger<br/>CSVログ"]
    SCN["demo_walk_scenario<br/>人を歩かせる"]
    PC["pose_client<br/>set_pose サービス"]
  end

  CAM --> BC
  BC -->|"/robot_risk/camera"| DET
  DET -->|"/robot_risk/risk_state"| CTRL
  DD -->|"odom"| BM
  BM -->|"/robot_risk/odom"| CTRL
  CTRL -->|"/robot_risk/cmd_vel"| BM
  BM -->|"速度指令"| DD
  CTRL -->|"/robot_risk/cmd_vel を監視"| WD
  WD -->|"途絶時のみゼロ速度"| BM

  OV -.-> BM
  BM -.->|"/robot_risk/overview_camera"| REC
  DET -.->|"/robot_risk/detection_image"| REC
  DET -.->|"risk_state"| LOGGER
  CTRL -.->|"/robot_risk/controller_state"| REC
  CTRL -.->|"controller_state"| LOGGER
  ROBOT -.-> BP
  BP -.->|"dynamic_pose/info"| LOGGER
  CTRL -.->|"controller_state"| SCN
  DET -.->|"risk_state（時刻）"| SCN
  SCN -.-> PC
  PC -.->|"人の位置を更新"| SCENE
  SCN -.->|"/robot_risk/demo_event"| REC
  SCN -.->|"demo_event"| LOGGER
```

## 図2：認識から速度指令までの処理

```mermaid
flowchart LR
  A["カメラ画像<br/>rgb8"] --> B{"画像は有効か"}
  B -->|"無効"| I["INVALID<br/>リスク100"]
  B -->|"有効"| C["床の境界を列ごとに検出"]
  C --> D["平坦床モデルで<br/>推定距離に換算"]
  D --> E["物体にまとめる<br/>幅、高さ、左右端"]
  E --> F["人らしさの判定<br/>形状ルール"]
  F --> G["物体ごとのリスク<br/>0〜100"]
  G --> H["判定状態<br/>最大リスクなど"]
  H --> J["safety_controller"]
  I --> J
  J --> K["/robot_risk/cmd_vel"]
```

## 図3：制御モードの遷移

```mermaid
stateDiagram-v2
  [*] --> GOAL
  GOAL --> DETOUR: STOPの原因が障害物
  DETOUR --> GOAL: 経由点に到達、または10秒ふさがれた
  GOAL --> WAIT_PERSON: 進路上の人のリスクが60以上
  DETOUR --> WAIT_PERSON: 進路上の人のリスクが60以上
  WAIT_PERSON --> GOAL: 近くに物体がなくなり1秒経過、または12秒経過
  GOAL --> [*]: 目標から0.3 m以内
```

カメラ無効や通信途絶では、モードに関わらず速度をゼロにします（`robot_risk_safety_controller.py` の `tick`、223行〜）。

## 各要素の役割

| 要素 | 役割 | 根拠 |
|---|---|---|
| Gazebo | 物理とカメラ描画。ワールドは `robot_risk_office_slalom.sdf` | `start_sim.sh` |
| ROS 2 | ノード間の通信 | 各ノードのソース |
| カメラ | 水平画角1.05 rad、床から0.67 m、下向き0.10 rad | SDF、`risk_core.py:24-26` |
| 認識処理 | 床境界の検出と推定距離、物体化、人の判定 | `risk_core.py:121`（boundary）、`:145`（process）、`:207`（_top_row）、`:273`（独立判定） |
| 危険度計算 | `100 × 接近度 × (0.3 + 0.7 × 進路係数) × (0.7 + 0.3 × 信頼度)` | `risk_core.py:289` |
| 判定閾値 | 減速30、停止70、解除60 | `risk_core.py:41-43` |
| 移動制御 | 速度決定、迂回、人の待機 | `robot_risk_safety_controller.py:155`（plan_detour）、`:185`（segment_clear）、`:201`（memory_blocks）、`:256`（人の条件） |
| タイムアウト | カメラ1.5 s、リスク状態とオドメトリ1.5 s、指令0.5 s（いずれも実時間） | `robot_risk_detector.py:29`、`robot_risk_safety_controller.py:33-34`、`robot_risk_cmd_watchdog.py:15` |

## 主なトピック

| トピック | 型 | 発行元 → 購読先 |
|---|---|---|
| `/robot_risk/camera` | `sensor_msgs/Image` | bridge_cam → detector |
| `/robot_risk/risk_state` | `std_msgs/String`（JSON） | detector → controller, recorder, logger, scenario |
| `/robot_risk/risk_score` | `std_msgs/Float32` | detector → 購読先なし |
| `/robot_risk/detection_image` | `sensor_msgs/Image` | detector → recorder |
| `/robot_risk/odom` | `nav_msgs/Odometry` | bridge → controller |
| `/robot_risk/cmd_vel` | `geometry_msgs/Twist` | controller, watchdog → bridge |
| `/robot_risk/controller_state` | `std_msgs/String`（JSON） | controller → recorder, logger, scenario |
| `/robot_risk/demo_event` | `std_msgs/String` | scenario → recorder, logger |
| `/world/robot_risk_world/dynamic_pose/info` | `tf2_msgs/TFMessage` | posebridge → logger, テスト |

## 注意点

- 真値の姿勢（`posebridge`）はログと評価だけに使い、検知と制御には入力していません。
- `/robot_risk/cmd_vel` の通常の発行元はコントローラだけです。watchdogは、走行中に指令が0.5秒途絶えた時だけゼロ速度を送ります。
- cmd_velのブリッジ自体が止まると、この構成では止められません（DiffDriveが最後の指令を保持するため）。この故障は試験していません。
