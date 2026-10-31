# 新しいクローンでの再現性確認手順

新しいフォルダと**別のコンテナ**で、リポジトリだけから最終実験が動くかを確かめます。

**この手順はまだ実行していません。** 既存のコンテナ `pai_ros2`、プロジェクトのフォルダ、既存のログは変更しません。GitHubへのpushも不要です。

## 方針

| 項目 | 内容 |
|---|---|
| コンテナ | `pai_ros2_verify` を新しく作り、終わったら削除する |
| フォルダ | `~/robot_risk_verify/repo`（プロジェクトの外） |
| ポート | 公開しない。`run_pai_container.sh` は名前とポートが既存と衝突するので使わない |
| 既存の `pai_ros2` | 止めない。動かしたまま確認するとCPUを取り合い、実時間比が下がります |

## 1. 確認用のクローンを作る

GitHubに登録する前でも確認できます。Gitの情報はプロジェクトの外に置くので、プロジェクトに `.git` は作られません。

```bash
P="$PWD"; V="$HOME/robot_risk_verify"; mkdir -p "$V"; git --git-dir="$V/src.git" --work-tree="$P" init -q; git --git-dir="$V/src.git" --work-tree="$P" add -A; git --git-dir="$V/src.git" --work-tree="$P" -c user.name=verify -c user.email=verify@example.invalid commit -qm "verification snapshot"; git clone -q "$V/src.git" "$V/repo"; ls "$V/repo"
```

GitHubに登録した後は、代わりに `git clone <リポジトリのURL> ~/robot_risk_verify/repo` を使えます。

**確認：** 次が0件であること。動画、バイナリ、PIDファイル、キャッシュが含まれていないことを示します。

```bash
cd ~/robot_risk_verify/repo && find . -path ./.git -prune -o -type f \( -name '*.mp4' -o -name '*.pt' -o -name '*.pyc' -o -name '*.pid' -o -name '.DS_Store' -o -name 'pose_client' \) -print | wc -l
```

## 2. 別のコンテナを起動する

確認時のイメージIDが `sha256:0380d72b...` であることも見ます。

```bash
docker image inspect airobotbook/ros2-desktop-ai-robot-book-humble:latest -f '{{.Id}}'
```
```bash
docker run -d --name pai_ros2_verify --platform linux/amd64 --shm-size=512m -v "$HOME/robot_risk_verify/repo":/workspace airobotbook/ros2-desktop-ai-robot-book-humble:latest
```

## 3. ビルドとワールドの確認

```bash
docker exec pai_ros2_verify bash -c 'cd /workspace/risk_system && g++ -O1 -std=c++17 pose_client.cpp -o pose_client $(pkg-config --cflags --libs ignition-transport11 ignition-msgs8) && ls -l pose_client'
```

SDFは生成スクリプトから同じものが作れるはずです。

```bash
docker exec pai_ros2_verify bash -c 'cd /workspace && python3 risk_system/make_office_world.py /workspace/robot_risk_world.sdf /tmp/office.sdf && md5sum /tmp/office.sdf robot_risk_office.sdf robot_risk_office_slalom.sdf risk_system/scene_slalom.json && ign sdf -k robot_risk_office_slalom.sdf'
```

| ファイル | 期待するMD5 |
|---|---|
| `/tmp/office.sdf`、`robot_risk_office.sdf` | `a41553116b05dfffe96a0d06ecb217db` |
| `robot_risk_office_slalom.sdf` | `505c967df70ca209d90bdf6df2c17e43` |
| `scene_slalom.json` | `9ff2c77bd702ff21e9f935d765aed905` |

## 4. シミュレーションの起動

```bash
docker exec -e WORLD=/workspace/robot_risk_office_slalom.sdf pai_ros2_verify bash /workspace/risk_system/start_sim.sh
```

約20秒待ってから確認します。カメラが届かない場合は、下の「描画の依存パッケージ」を見てください。

```bash
docker exec pai_ros2_verify bash -c 'source /opt/ros/humble/setup.bash; cd /workspace/risk_system; python3 gt_pose.py 3; python3 topic_stamps.py /robot_risk/camera /robot_risk/overview_camera'
```

**期待：** ロボットが (−5.0, 0.0)、ピッチ1°未満、両カメラのメッセージが届く。

### 描画の依存パッケージ（必要な時だけ）

開発用コンテナには、次のパッケージを後から追加していました。新しいコンテナで必要かは未確認です。**まず入れずに試し、必要だった場合はREADMEに追記してください。**

```bash
docker exec pai_ros2_verify bash -c 'apt-get update && apt-get install -y xvfb libgl1-mesa-dri libegl1'
```

## 5. 実験の実行

```bash
docker exec pai_ros2_verify bash /workspace/risk_system/run_slalom.sh verify1 record
```

実時間で約2.5〜4分かかります。上限は300秒です。

```bash
docker exec pai_ros2_verify bash -c 'cd /workspace/risk_system; cat logs/slalom_verify1.summary.txt; python3 slalom_timeline.py logs/slalom_verify1.csv | tail -1; grep -E "detour |goal reached|person" logs/slalom_verify1.controller.log | cut -c60-130'
```

## 6. 判定

| 項目 | 合格の基準 | 参考：`slalom_final5` |
|---|---|---|
| 目標到達 | ログに `goal reached` | あり |
| 最小クリアランス | 0.20 m以上 | 0.268 m |
| 最大ピッチ | 転倒なし（2°未満を目安） | 0.78° |
| 迂回の順序 | 右 → 左 → 右 | 同じ |
| 人の待機 | 待機して自動で再開 | 2.9 s |
| INVALID | 0回を目安 | 0回 |
| 走行時間 | 参考のみ。60 s前後 | 59.2 s |

実時間比も記録してください（最終実験は約0.42）。別のコンテナと同時に動かすと下がる可能性があります。数値が少し変わるのは、プロセスのタイミングによるものです。

**動画を確認する場合：**

```bash
docker exec pai_ros2_verify bash -c 'cd /workspace/risk_system/videos; ffprobe -v error -show_entries format=duration -of default=nw=1 raw_slalom_verify1.mp4'
```

## 7. 後片付け

新しく作ったものだけを削除します。名前をよく確認してから実行してください。`pai_ros2` は削除しないでください。

```bash
docker rm -f pai_ros2_verify
```
```bash
rm -rf "$HOME/robot_risk_verify"
```

## 結果の記録欄

| 項目 | 結果 |
|---|---|
| 実施日 | （未実施） |
| 追加パッケージの要否 | （未確認） |
| 目標到達 | （未確認） |
| 最小クリアランス | （未確認） |
| 実時間比 | （未確認） |
