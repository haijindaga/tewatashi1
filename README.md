# tewatashi1

UR7e（PolyScope 5）＋ Robotiq 2F-85 ＋ RealSense D435（グリッパ取付）で、
隠れた物体を取り出してから人に手渡す（R2H handover）実験を行うための ROS 2 Humble ワークスペース。

構成は UR 公式チュートリアル
[Universal_Robots_ROS2_Tutorials / my_robot_cell](https://github.com/UniversalRobots/Universal_Robots_ROS2_Tutorials)
の作法に合わせています（description / control / moveit_config の 3 パッケージ構成）。

> **現状：骨格のみ。** グリッパとカメラは「ぶつからないための仮の箱」で、まだ動きません。
> 実測していない寸法は仮の値です（下の「研究室で測るもの」）。

## パッケージ

| パッケージ | 役割 |
|---|---|
| `tewatashi_description` | セル（机・UR7e・グリッパ外形・カメラ外形）の URDF と表示用 launch |
| `tewatashi_control` | ur_robot_driver を包む起動 launch。関節範囲を狭めた `joint_limits.yaml` |
| `tewatashi_moveit_config` | MoveIt 設定（OMPL / Pilz / CHOMP）。初期姿勢 `home` |
| `tewatashi_task` | 常駐するタスクノード（YASMIN の状態機械）と操作パネル（Web） |
| `tewatashi_perception` | 認識のサービスノード（いまは SAM3）。専用の venv の Python で動く |
| `tewatashi_interfaces` | タスクと認識ノードの間のサービス定義 |
| `tewatashi_bringup` | 全部を 1 コマンドで起動する launch |

## 前提

- Ubuntu 22.04 / ROS 2 Humble（apt で導入済み）
- `ros-humble-ur`、`ros-humble-moveit`、YASMIN、pymoveit2、rosbridge（下の `rosdep install` が package.xml を見て入れます）

```bash
sudo apt update
```

## ビルド

```bash
mkdir -p ~/projects/handover_ws/src
cd ~/projects/handover_ws/src
git clone https://github.com/haijindaga/tewatashi1.git
cd ~/projects/handover_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -y
colcon build --symlink-install
source install/setup.bash
```

## 認識用の venv（最初に 1 回だけ）

SAM3 などの深層学習ライブラリは ROS の apt / rosdep にないので、ROS 公式ドキュメント
「Using Python Packages with ROS 2」の作法どおり、システムの Python（3.10）の上に
`--system-site-packages` 付きの venv を作って入れます（conda は使いません）。

```bash
cd ~/projects/handover_ws
python3 -m venv --system-site-packages venv_perception
touch venv_perception/COLCON_IGNORE
source venv_perception/bin/activate
pip install "numpy<2" torch torchvision --index-url https://download.pytorch.org/whl/cu128
pip install -r src/tewatashi1/tewatashi_perception/requirements.txt
deactivate
```

- `numpy<2`：ROS Humble は NumPy 1 系前提
- `pillow>=9.1`：Ubuntu 22.04 標準の Pillow 9.0 では transformers が動かない
- SAM3 のモデル（facebook/sam3）は Hugging Face で利用申請とログイン（`hf auth login`）が必要
- 実測：SAM3（float32）1 枚で GPU ピーク 4.0 GB

起動時は `bringup.launch.py` が venv の Python で `sam3_server` を立ち上げます
（venv の場所が違うときは `perception_python:=<venv>/bin/python`、使わないときは `use_perception:=false`）。

## ふだんの使い方（1 コマンドで全部起動）

```bash
source ~/projects/handover_ws/install/setup.bash
ros2 launch tewatashi_bringup bringup.launch.py          # 模擬ハードウェア
```

ドライバ・MoveIt・RViz・タスクノード・状態機械ビューア・操作パネルが立ち上がり、
ブラウザで操作パネル（http://127.0.0.1:8080）が開きます。

1. 起動後、まず **初期位置（home）への軌道** が計画され、RViz に表示されて承認待ちになります
2. パネルの「この動作を実行」で動く（確認せずに動くことはありません。模擬のときだけ確認を外せます）
3. 「スタート」でタスク（home → 観察姿勢 → 画像取得 → 切り出し → home）を最初から実行
4. おかしいと思ったら **「停止」（または Esc キー）**。やり直すときは「初期位置へ」→「スタート」
5. 終わるときはターミナルで Ctrl+C

パネル右側は YASMIN の状態機械ビューア（いまどの段階か）です。

### カメラなしで認識を試す（写真をカメラの代わりに流す）

```bash
sudo apt install ros-humble-image-publisher   # 初回のみ
ros2 run image_publisher image_publisher_node ~/写真.jpg \
  --ros-args -r image_raw:=/camera/camera/color/image_raw
```

これを別のターミナルで流しながら「スタート」すると、画像取得 → 切り出し（SAM3）まで進み、
結果はパネル右下の「切り出し結果」に表示されます（web_video_server 経由）。切り出す物は実行中でも変えられます：

```bash
ros2 param set /tewatashi_task target_object mug
```
画像やログをファイルに大量に書き出すことはしません（パネルのログは直近 30 行）。

### 実機（研究室で、下のチェックが済んでから）

```bash
ros2 launch tewatashi_bringup bringup.launch.py \
  use_fake_hardware:=false robot_ip:=<ロボットのIP> \
  kinematics_params_file:=<ur_calibrationで取り出したyaml>
```

ペンダント側で External Control プログラムを再生してから操作します。

## 個別に動作確認するとき

### 1. 見た目だけ確認（ロボットは動かさない）

```bash
ros2 launch tewatashi_description view_cell.launch.py
```

スライダで関節を動かし、机・グリッパ箱・カメラ箱の位置関係を見る。

### 2. 模擬ハードウェアで MoveIt（実機なし）

ターミナルを 3 つ使う（どれも最初に `source ~/projects/handover_ws/install/setup.bash`）。

```bash
# 1つ目：ドライバ（模擬）。use_fake_hardware は既定で true
ros2 launch tewatashi_control start_robot.launch.py

# 2つ目：MoveIt
ros2 launch tewatashi_moveit_config move_group.launch.py

# 3つ目：RViz
ros2 launch tewatashi_moveit_config moveit_rviz.launch.py
```

RViz の MotionPlanning パネルで：

1. Planning Group が `ur_arm` になっていることを確認
2. 「Planning Pipeline」を `pilz_industrial_motion_planner`、Planner を `PTP` にする
   （毎回同じ動きになる。衝突が見つかれば動かない）
3. Goal State に `home` を選び、Plan → 軌道を目で確認 → Execute

速度・加速度は既定で 10%（`joint_limits.yaml` の scaling 0.1）。

## 安全のための設定（どこに何があるか）

| 対策 | 場所 | 出典・根拠 |
|---|---|---|
| 関節範囲を ±180° に制限（肘は UR 標準で ±180°） | `tewatashi_control/config/ur7e/joint_limits.yaml`、`tewatashi_moveit_config/config/joint_limits.yaml` | UR 標準は肘以外 ±360°。ケーブル巻き込み・無理な姿勢を避けるための本プロジェクトの選択 |
| 速度・加速度 10% | `tewatashi_moveit_config/config/joint_limits.yaml` | MoveIt Setup Assistant の既定値 |
| 机・グリッパ・カメラを衝突モデルに入れる | `tewatashi_description/urdf/tewatashi_cell_macro.xacro` | UR 公式チュートリアル（my_robot_cell） |
| 起動既定は模擬ハードウェア | `tewatashi_control/launch/start_robot.launch.py` | 本プロジェクトの選択 |
| 軌道は必ず MoveIt を通す | `tewatashi_task/tewatashi_task/motion.py` | ros2_control の軌道コントローラは関節範囲を検査しないため |
| 速度倍率をプログラムから必ず明示（0 < 倍率 ≤ 0.3） | `tewatashi_task/config/task.yaml`、`motion.py` | move_group は倍率 0.0 を **1.0（最高速）** として扱う（MoveIt 2 Humble の TOTG 実装）。`joint_limits.yaml` の 0.1 は RViz 等からの操作にしか効かない |
| 動作前に計画を RViz に表示して承認 | `tewatashi_task`（実機では外せない） | MoveIt チュートリアルの「次へ」で進める確認と同じ考え方 |
| Pilz PTP で計画（毎回同じ動き、衝突があれば動かない） | `motion.py` | Pilz industrial motion planner |
| 停止ボタン・Esc | 操作パネル → `~/stop` | ソフトウェア停止（軌道実行の取り消し）。非常時はペンダントの非常停止 |
| Web 系はすべて 127.0.0.1 のみで待ち受け | `tewatashi_bringup/launch/bringup.launch.py` | 研究室ネットワークからロボットを操作されないため |
| 最後の砦 | ペンダントの Safety 設定・非常停止 | UR マニュアル |

**注意：** 2 つの `joint_limits.yaml` は手で同期しています。片方を変えたらもう片方も。

## 研究室で測るもの（仮の値）

`tewatashi_description/urdf/tewatashi_cell_macro.xacro` の引数：

- [ ] 机の大きさ・ロボット基台からの位置（`table_size_*`, `table_center_*`）
- [ ] フランジから指先中心までの距離（`tcp_offset_z`、2F-85 単体なら約 0.174 m＋カップリング）
- [ ] カメラ取付位置・向き（`camera_mount_xyz`, `camera_mount_rpy`）→ 後でハンドアイ校正で置き換え
- [ ] 観察姿勢 `observe`（`tewatashi_task/config/task.yaml`、いまは計算で決めた仮の値）

ロボット・PC 側：

- [ ] Robotiq URCap が入っているか（グリッパドライバの選び方が変わる）
- [ ] 有線 LAN と IP（PC とロボットを同じサブネットに）
- [ ] External Control URCap とホスト IP の設定
- [ ] `ros2 launch ur_calibration calibration_correction.launch.py` でキャリブレーションを取り出す
- [ ] ペンダントの Safety 設定（速度・関節範囲・停止距離）
- [ ] リアルタイムスケジューリング（UR ドライバのドキュメントに従う）
- [ ] D435 が USB 3 で認識されるか（`rs-enumerate-devices`）

## 衝突行列について

`tewatashi_cell.srdf` の `disable_collisions` は手書きで、最小限にしています。
実寸が決まったら Setup Assistant で作り直すのが作法です：

```bash
ros2 launch tewatashi_moveit_config setup_assistant.launch.py
```

## ライセンス

BSD-3-Clause（SAM3 のモデル自体は Meta の SAM License）。UR 公式チュートリアル、ur_description、roslibjs 由来の部分は `third_party/` のライセンスに従います。
