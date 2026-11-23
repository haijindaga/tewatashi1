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

## 前提

- Ubuntu 22.04 / ROS 2 Humble（apt で導入済み）
- `ros-humble-ur`（ur_robot_driver 2.13 系、ur_description 2.10 系）、`ros-humble-moveit`

```bash
sudo apt install ros-humble-ur ros-humble-moveit
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

## 動作確認

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

### 3. 実機（研究室で、下のチェックが済んでから）

```bash
ros2 launch tewatashi_control start_robot.launch.py \
  use_fake_hardware:=false robot_ip:=<ロボットのIP> \
  kinematics_params_file:=<ur_calibrationで取り出したyaml>
```

ペンダント側で External Control プログラムを再生してから Plan / Execute。

## 安全のための設定（どこに何があるか）

| 対策 | 場所 | 出典・根拠 |
|---|---|---|
| 関節範囲を ±180° に制限（肘は UR 標準で ±180°） | `tewatashi_control/config/ur7e/joint_limits.yaml`、`tewatashi_moveit_config/config/joint_limits.yaml` | UR 標準は肘以外 ±360°。ケーブル巻き込み・無理な姿勢を避けるための本プロジェクトの選択 |
| 速度・加速度 10% | `tewatashi_moveit_config/config/joint_limits.yaml` | MoveIt Setup Assistant の既定値 |
| 机・グリッパ・カメラを衝突モデルに入れる | `tewatashi_description/urdf/tewatashi_cell_macro.xacro` | UR 公式チュートリアル（my_robot_cell） |
| 起動既定は模擬ハードウェア | `tewatashi_control/launch/start_robot.launch.py` | 本プロジェクトの選択 |
| 軌道は必ず MoveIt を通す | — | ros2_control の軌道コントローラは関節範囲を検査しないため |
| 最後の砦 | ペンダントの Safety 設定・非常停止 | UR マニュアル |

**注意：** 2 つの `joint_limits.yaml` は手で同期しています。片方を変えたらもう片方も。

## 研究室で測るもの（仮の値）

`tewatashi_description/urdf/tewatashi_cell_macro.xacro` の引数：

- [ ] 机の大きさ・ロボット基台からの位置（`table_size_*`, `table_center_*`）
- [ ] フランジから指先中心までの距離（`tcp_offset_z`、2F-85 単体なら約 0.174 m＋カップリング）
- [ ] カメラ取付位置・向き（`camera_mount_xyz`, `camera_mount_rpy`）→ 後でハンドアイ校正で置き換え

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

BSD-3-Clause。UR 公式チュートリアルおよび ur_description 由来の部分は `third_party/` のライセンスに従います。
