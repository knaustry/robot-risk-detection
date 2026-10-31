# experiment_data.csv の説明

`tools/build_data_and_graphs.py` が `risk_system/logs/` の実行ログだけから生成します。値の変更や補間はしていません。元ログにない項目は空欄です。

## 列

| 列 | 単位 | 内容 | 由来 |
|---|---|---|---|
| run_id | | ランの名前 | 元ファイル名 |
| world_file | | 使用ワールド | 実行時の記録 |
| layout | | 配置（下表） | 実行時の記録 |
| run_outcome | | 結果の分類 | `logs/slalom_runs.txt` とログ |
| wall_time_s | s（実時間） | ログ開始からの実時間 | `wall` |
| sim_stamp_s | s（シミュレーション） | 検知結果に付いたカメラ時刻 | `sim_t`。0または未記録は空欄 |
| t_rel_s | s（シミュレーション） | そのランの最初の sim_stamp_s からの経過 | 計算値 |
| gt_x_m, gt_y_m | m | Gazebo真値の位置 | `x`, `y` |
| gt_pitch_deg, gt_yaw_deg | ° | Gazebo真値の姿勢 | `pitch`, `yaw` |
| risk_state | | 判定状態 | risk_state トピック |
| risk_score_0_100 | 無次元 | 危険度スコア（推定値） | risk_state トピック |
| nearest_in_path_est_m | m | 進路上の最も近い物体までの**推定**距離 | risk_state トピック |
| detected_objects_est | | 検出物体（`ラベル@推定距離m`、床端は除外） | risk_state トピック |
| control_mode, control_reason | | 制御モードと理由 | controller_state トピック |
| cmd_linear_mps, cmd_angular_radps | m/s, rad/s | 速度指令の値（実際の速度ではない） | controller_state トピック |
| clearance_gt_m | m | ロボットの外接円（半径0.33 m）と物体の最小距離 | 真値から計算（ログ時） |
| closest_object | | 上記の相手 | 同上 |
| event | | シナリオのイベント | demo_event トピック |

## ラン

| run_id | 配置 | 記録行 | 走行時間 [s] | 目標到達 | 最小距離 [m] | 最大ピッチ [°] | 最大指令 [m/s] | 備考 |
|---|---|---|---|---|---|---|---|---|
| slalom_final5 | L3 | 660 | 59.2 | はい | 0.268 | 0.78 | 0.30 | **採用** |
| slalom_final4 | L3 | 661 | 57.8 | はい | 0.249 | 0.78 | 0.30 | 待機条件の修正前 |
| slalom_final3 | L3 | 745 | 57.6 | はい | 0.254 | 0.80 | 0.30 | 待機条件の修正前 |
| slalom_try6 | L3 | 630 | 58.0 | はい | 0.255 | 0.84 | 0.30 | 録画なし |
| slalom_try5 | L3 | 725 | 67.3 | はい | 0.310 | 0.77 | 0.30 | 録画なし、速度設定が異なる |
| slalom_try4 | L2 | 769 | 74.0 | はい | **−0.022** | 0.82 | 0.30 | **不合格**（事実上の接触） |
| slalom_try3 | L1 | 1131 | 112.7 | いいえ | 0.213 | 0.78 | 0.30 | 途中で停止 |
| slalom_try2 | L1 | 1125 | 103.4 | いいえ | 1.77 | 0.77 | 0.30 | 誤判定で待機し続けた |
| office_20261018_142401 | office | 1202 | 記録なし | はい | 0.254 | 0.77 | 0.15 | 実時間255 s。古いコード |
| plain_20261018_133839 | plain | 649 | 記録なし | はい | 0.263 | 0.77 | 0.15 | 実時間136 s。古いコード |

走行時間は t_rel_s の最大値です。

| 配置 | 内容 |
|---|---|
| L1 | obstacle_1 (−1.4, 0.25)、obstacle_2 (1.4, −0.7)、obstacle_3 (4.6, 0.3)、人 x=3.0 |
| L2 | obstacle_1 (−1.4, 0.4)、obstacle_2 (1.4, −0.6)、obstacle_3 (5.0, 0.55)、人 x=3.2 |
| L3 | obstacle_1 (−1.4, 0.4)、obstacle_2 (1.4, −1.0)、obstacle_3 (5.0, 0.55)、人 x=3.2 |

L1とL2は現在のSDFとは異なります。L1、L2の配置は開発中の生成スクリプトの値で、当時のSDFは保存していません。

## 注意

- 含めていないラン：`slalom_try1`、`slalom_final1`、`slalom_final2` は起動の誤りで走行していません。
- office と plain のランは sim_t が0で記録されているため、シミュレーション時間は空欄です。実時間とは比較できますが、スラロームのランとは時間軸が異なります。
- office と plain の clearance は、障害物と人だけが対象です。壁や家具は含みません。スラロームでは、障害物、人、壁、間仕切りを対象にしています。
- 人の真値位置と、物体ごとの横位置は記録していません。人とのクリアランスは clearance_gt_m に含まれていますが、人の位置を時系列で取り出すことはできません。
- `logs/slalom_runs.txt` の try5 の「length ~76」は、当時のログスクリプトの誤りで出た値です（後で修正）。正しい走行時間は67.3 sです。
- 動画に表示された値はこのCSVに含まれていません。
