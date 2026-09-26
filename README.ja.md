# experiment_cat 日本語ガイド

## emcl2による自己位置推定

```bash
cd ~/ros2_ws
vcs import src < src/experiment_cat/dependencies.repos
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/emcl2_ros2 src/experiment_cat --ignore-src --rosdistro humble -y
colcon build --packages-select emcl2 cat_bringup experiment_cat --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ros2 launch experiment_cat navigation.launch.py slam:=false map:=$HOME/ros2_ws/indoor_loop_map_vio.yaml
```

RVizの「2D Pose Estimate」で初期位置を指定してから、Aで走行許可・Nav2 Goalで目標を指定します。
AMCLは起動せず、emcl2のみが`map → odom`を発行します。地図作成時（slam:=true）はSLAMを使用します。

自己位置推定のみを使う場合は、センサ・odom・TFを別途起動した状態で以下を実行します。
navigation.launch.pyとの同時起動は不要です。

```bash
ros2 launch experiment_cat localization.launch.py map:=$HOME/ros2_ws/indoor_loop_map_vio.yaml
```

設定は`config/emcl2.yaml`（`localization_params_file:=絶対パス`で変更可能）。
500粒子、odom更新30 Hz、scanは5点ごとに評価します。
膨張リセットはalpha_threshold=0.5、位置半径0.1 m・姿勢半径0.2 radを初期値としています。
`sensor_reset: false`は別のセンサリセット機能の設定で、膨張リセットの無効化ではありません。
主な出力は`/mcl_pose`、`/particlecloud`（PoseArray）、`/alpha`、`map → odom`です。
`/scan`はcat_bringupからReliableで配信します。オドメトリの選択方法は変更ありません。
膨張リセットは推定の復帰を試みる機能であり、常に復帰できる保証や走行許可の判断ではありません。

共通のセンサ・制御・オドメトリ起動は`cat_bringup`パッケージへ分離しました。
通常起動は`ros2 launch cat_bringup bringup.launch.py`です。ジョイスティックは既定で有効、
Xは手動、Aは外部`/cmd_vel`入力を選択します。切替は共通スムーサーでゼロまで減速してから行います。
従来の`experiment_cat bringup.launch.py`も互換入口として利用できます。
Nav2専用launchのA/X切替・安全監視は従来の構成です。詳しくは`cat_bringup/README.md`を参照してください。

[English README](README.md)

DDSM115差動二輪ロボットをROS 2 Humble / Nav2で動かすための設定です。
屋内の障害物検出と地図作成にはRPLIDAR S1を使用します。
通常の `/odom` はZED VIOです。車輪＋ZED IMUのEKFは独立して `/wheel_odom` を発行します。
GNSSは現在の走行構成には統合していません。

## Nav2の安全停止・ジョイスティック（2026-09追加）

`navigation.launch.py` は常にjoyとnavigation_safetyを起動します。
**起動直後はブレーキ状態です。初期位置設定・センサ確認後に明示的な走行許可が必要です。**
この節が下の旧起動手順より優先します。別のbringup/teleopを重複起動しないでください。

| 操作 | 動作 |
|---|---|
| A | Nav2走行を許可。B/Y後は健康・静止状態を2秒確認して復帰 |
| X | 手動走行へ切替。右スティック、上限1.0 m/s |
| B | 即時ブレーキ |
| Y | 一旦ブレーキし、停止確認後にフリー |

BとYを同時に押すとBを優先します。Yは車輪RPMが±1以下で0.3秒続いた場合だけフリーにします。
3秒以内に確認できなければブレーキのままです。**フリーは車体を保持せず、坂で転がります。**
joyトピックが0.5秒途絶えるとフリー中でもブレーキにします。
無線受信機が切断をROSへ通知しない場合は無線リンク断を検出できないことがあります。

### 起動・再許可

1. 既存のNav2/bringupを停止し、更新版を起動。
2. RVizの2D Pose Estimateで現在位置・方向を設定。
3. 車輪停止、LiDAR/odom/TF、ジョイスティック受信が正常であることを確認。
4. B/Yを離して2秒以上待ち、Aを押してNav2走行を許可。
5. 新しいゴールを指定するか、B/Y前の有効なゴールへ復帰。

```bash
ros2 service call /navigation_safety/arm std_srvs/srv/Trigger '{}'
ros2 topic echo /navigation_safety/status
```

サービスによる許可も利用できます。Yの後にA/Xを押した場合は自動的にブレーキを経由し、
停止・健康状態が2秒続いてから選択モードへ入ります。
異常の原因を直しても自動復帰しません。VIOアダプタが停止をラッチした場合はlaunchの再起動も必要です。

```bash
# ジョイスティックと同じ操作をサービスから実施
ros2 service call /navigation_safety/brake std_srvs/srv/Trigger '{}'
ros2 service call /navigation_safety/free std_srvs/srv/Trigger '{}'
```

### 障害物回避（Navfn + DWB）

追加の認識モデルやGPUは使いません。標準のNavfnが地図と現在のLiDAR障害物を使って
既定BTで1 Hzの経路再計画を行い、DWBが1.7秒先までの速度候補を評価します。
global costmapは2 Hz、local costmapは10 Hz、制御は30 Hzです。
local costmapは6×6 mとし、候補軌跡の詳細配信を無効にして計算・通信負荷を抑えています。

インフレーション半径0.8 mは通行禁止幅でも、車体から必ず空ける距離でもありません。
壁や人から離れるほど低くなるコストを付け、通れる隙間の中で離れた経路を選ばせます。
コストは単純加算されないため、両側の色付き領域が重なるだけで通行不能にはなりません。
DWBでは`ObstacleFootprint`も有効にし、旋回時の長方形車体の輪郭を評価します。
`GoalAlign`は重み24・前方評価点0.1 mで有効にし、`BaseObstacle`の重みは0.5です。
速度上限は並進1.0 m/s・旋回1.0 rad/s、スムーサー上限はそれぞれ1.1です。

反映後はnavigationを再起動し、RVizで以下を確認してください。

1. `/global_costmap/costmap` と `/local_costmap/costmap` に人の位置の障害物セルが出ること。
2. `/plan`（Path）が人の左右いずれかを通る経路へ更新されること。
3. `/local_plan`（Path）がその経路に向かい、ロボットが迂回すること。

1が出ない場合はscan/TF/障害物レイヤー、1が出て2が変わらない場合は再計画や通行可能幅、
2が変わるのに停止する場合はDWB・Collision Monitor・安全監視を切り分けます。
`ros2 topic echo /navigation_safety/status`も確認できます。これらはbagなしで確認可能です。
停止領域に既に入った場合は旋回も停止します。全ての配置で無停止の回避を保証する設定ではありません。

### 停止の仕組みと限界

- `/cmd_vel` は直接車輪へ渡さず、監視ノードが `/cmd_vel_safe` に転送します。
- Nav2の速度指令はCollision Monitorを通り、前方停止領域（base_linkからx=0.325 m、幅0.38 m）で停止指令を出します。footprint前端から約0.25 mですが、実停止距離は速度・遅延に依存します。
- local/global costmapのLiDAR障害物レイヤーとDWBが迂回を試みます。迂回不能なら停止します。
- odom/scan/joy、車輪フィードバック、`odom -> base_link` と `map -> odom` の更新を監視。
- odom/scanは0.5秒、車輪RPMは0.3秒、map TFは1.5秒を超える古さで走行を禁止。
  自己位置推定器の未来時刻TFを許容しますが、29秒の停止を許容時間で隠しません。
- 監視ノードは20Hzでドライバーへモード信号を送信。0.3秒途絶えたらドライバー側もブレーキ。
  途絶後に走行信号だけが戻っても復帰しません。
- 通常終了・SIGINT・例外・途中の初期化失敗でも、ドライバーはポートを閉じる前に
  各モーターへゼロ指令・速度モード・ブレーキを最大3回送信します。停止応答がなければ警告します。
- Nav2中は通常のブレーキ解除・freewheelサービスで安全ラッチを迂回できません。
- センサ異常がなくても、古い速度指令は0.3秒で転送を止めます。

この監視はNav2のlaunchに適用します。手動bringup/校正は既存操作を維持しますが、
ドライバー終了時の停止処理は共通です。通常終了後の保持状態はブレーキになります。
**SIGKILL、USB通信断、電源断、OS停止ではソフトウェア停止を保証できません。**
物理非常停止を必ず残してください。Yや停止サービスの成功は物理停止の保証ではありません。
初回は車輪を浮かせて、B/Y・センサ停止・終了時停止を監視下で確認してください。

## オドメトリの選択（2026-09更新）

この節の仕様が、以下に残る旧手順の「通常はEKF」の説明より優先します。

| トピック | 内容 | 親 / 子フレーム |
|---|---|---|
| `/wheel/odom` | 車輪だけの従来の推定 | odom / base_link（TFなし） |
| `/wheel_odom` | 車輪＋IMUの独立EKF | wheel_odom / base_link（TFなし） |
| `/zed/zed_node/odom` | ZEDの元のVIO | zed_odom / zed_camera_link（TFなし） |
| `/odom` | 選択した走行用推定 | odom / base_link（TFあり） |

標準は `odom_source:=vio`。`bringup.launch.py`、`mapping.launch.py`、`navigation.launch.py` に共通です。
VIOのカメラ取付位置をTFで補正し、開始時の車体位置・方位を原点として2D化します。
速度は変換済み位置・方位の差分です。ZEDの出力周期（設定15fps）に従い、30Hzへ補間しません。
共分散は取り付けオフセットを考慮した保守的な対角近似です。
EKFの独立原点 `wheel_odom` をVIOの `odom` と同一視する静的TFは発行しません。

```bash
# 通常：VIOが /odom。車輪＋IMUも /wheel_odom に並行出力
ros2 launch experiment_cat bringup.launch.py enable_joystick:=true

# 軽量：VIO・深度処理OFF、IMUとRGBはON。EKFを /odom にコピー
ros2 launch experiment_cat bringup.launch.py enable_joystick:=true odom_source:=wheel

# 地図作成も標準でVIO。軽量版は odom_source:=wheel を追加
ros2 launch experiment_cat mapping.launch.py

# 保存地図で走行する例
ros2 launch experiment_cat navigation.launch.py slam:=false map:=/絶対パス/map.yaml odom_source:=vio
```

切替時はロボットを停止してlaunch全体を終了し、選択を変えて再起動してください。
走行中の動的切替や、VIO故障時の自動フォールバックは行いません。
`odom_source` を変える際は、以前の `zed_config` を指定したままにしないでください。
カメラなしで使う場合は `use_zed:=false odom_source:=wheel` を指定します。

VIO切断中は新しい `/odom`/TFを出しません。受信時に1秒超のデータ間隔、時刻巻き戻り、
フレーム変更、大きな位置ジャンプを検出した場合は再起動まで出力を止めます。
すべての追跡異常を検出するものではなく、モーターの非常停止機能でもありません。
異常時はロボットを停止してから再起動・初期位置設定を行ってください。

`manual_loop_test` は `/wheel_odom` も記録します（旧bagの `/odom` はEKFでした）。
`odom_tests` と `robot_calibration` は校正条件を変えないため明示的にwheelモードを使用します。
直接 `ekf.launch.py` を使う旧テストは従来の `/odom`/TF設定を維持しています。

```bash
ros2 topic echo /odom --once
ros2 topic echo /wheel_odom --once
ros2 run tf2_ros tf2_echo odom base_link
```

## 目次

- [準備](#準備)
- [用途別の起動方法](#用途別の起動方法)
- [ジョイスティック操作](#ジョイスティック操作)
- [周回して軌跡と地図用データを記録](#周回して軌跡と地図用データを記録)
- [rosbagから2D占有格子地図を作成](#rosbagから2d占有格子地図を作成)
- [車輪・EKF・VIOの軌跡を比較](#車輪ekfvioの軌跡を比較)
- [保存した地図で自律走行](#保存した地図で自律走行)
- [オドメトリ校正テスト](#オドメトリ校正テスト)
- [センサ・TF・設定ファイル](#センサtf設定ファイル)
- [困ったとき](#困ったとき)

## 準備

新しいターミナルを開くたびに実行します。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
```

コードを変更した場合はビルドします。

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select ddsm115_controller experiment_cat --symlink-install
source install/setup.bash
```

別の用途へ切り替えるときは、前のlaunchをCtrl+Cで停止してください。
複数のbringupを同時に動かすと、モーター・カメラ・TF・速度指令が競合します。
bagの再生時も実機のlaunchは停止します。

## 用途別の起動方法

以下は同時に実行する一覧ではありません。目的に合うものを選びます。

| 目的 | コマンド |
|---|---|
| 車体モデルだけ表示（モーター接続なし） | `ros2 launch experiment_cat visualize.launch.py` |
| センサ・車輪・EKFを確認 | `ros2 launch experiment_cat bringup.launch.py` |
| 手動走行 | `ros2 launch experiment_cat bringup.launch.py enable_joystick:=true` |
| ZED単体 | `ros2 launch experiment_cat zed_sensors.launch.py` |
| LiDAR単体 | `ros2 launch experiment_cat rplidar_s1.launch.py` |
| 手動走行しながら地図を作る | `ros2 launch experiment_cat mapping.launch.py` |
| 周回を記録し、後から地図・軌跡を確認 | `ros2 launch experiment_cat manual_loop_test.launch.py` |
| Nav2起動（既定はSLAM併用） | `ros2 launch experiment_cat navigation.launch.py` |

`bringup.launch.py`は自動で走行指令を出しませんが、外部の`/cmd_vel`を受け付けます。
`use_base:=false`、`use_zed:=false`、`use_lidar:=false`で個別に省略できます。
カメラだけを確認する場合は上表のZED単体launchが簡単です。

LiDARのデバイス名`/dev/rplidar`を設定していない場合は、一度だけ実行します。

```bash
sudo install -m 644 \
  ~/ros2_ws/src/experiment_cat/udev/99-experiment-rplidar.rules \
  /etc/udev/rules.d/99-experiment-rplidar.rules
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=tty
udevadm settle
ls -l /dev/rplidar
```

## ジョイスティック操作

| 操作 | 動作 |
|---|---|
| X | 通常ドライブ：上限1.0 m/s |
| A | 高速ドライブ：上限6 km/h（約1.67 m/s） |
| B | ブレーキ |
| Y | フリーモード |
| 右スティック上下 | 前進・後退 |
| 右スティック左右 | カーブの曲率 |

手動操作は曲率制御なので、並進ゼロで旋回入力だけを入れてもその場旋回はしません。
速度指令は`/cmd_vel_teleop`からNav2 velocity smootherを通り、`/cmd_vel`になります。
調整先は[manual_control.yaml](config/manual_control.yaml)です。
フリーモードで手押しする場合も、車輪が接地して回転し、電源・通信が有効なら車輪odomを取得できます。

## 周回して軌跡と地図用データを記録

### 1. 起動

```bash
ros2 launch experiment_cat manual_loop_test.launch.py
```

手動走行、LiDAR、EKF、ZED VIO、RViz、rosbag記録が起動します。
このlaunchはNav2自律走行やSLAMを起動しません。

### 2. 走り始める前の確認

`Recording...`を確認し、別ターミナルで実行します。

```bash
ros2 topic echo /zed/zed_node/odom --once --field pose.pose
ros2 topic echo /localization/imu_status --once --qos-durability transient_local
ros2 topic hz /scan
```

状態が`wheel_imu`であることを確認します。`hz`はCtrl+Cで終了できます。
VIOはメッセージが出るだけでなく、少し動かしたときに位置が変化することも確認してください。
過去のbagでは、車輪・EKFが移動してもVIOがほぼ原点のままという例がありました。

### 3. 一周して停止

1. 車軸中心の開始位置と車体の向きを床にマークします。
2. Xを押し、右スティックでゆっくり周回します。
3. 開始位置と向きへ戻り、Bで停止します。
4. 約5秒静止してから、launch側でCtrl+Cを1回押します。
5. `Recording stopped`と各プロセスの終了を待ちます。

実際の終了位置・角度のずれも測っておくと、推定軌跡と比較できます。

### 保存内容

起動したディレクトリに`manual_loop_YYYYMMDD_HHMMSS/`が作られます。

| 記録トピック | 内容 |
|---|---|
| `/wheel/odom` | 車輪だけの軌跡 |
| `/odom` | 車輪＋IMUのEKF軌跡 |
| `/zed/zed_node/odom` | ZED VIOの参考軌跡 |
| `/scan` | 2D LiDARの測距 |
| `/tf` | 時間変化する座標変換 |
| `/tf_static` | センサ取付位置などの固定座標変換 |

画像・モーター情報・操作指令・IMU生データは記録しません。
IMUは走行中の推定に使用しますが、このbagからEKFを設定変更して再計算することはできません。

```bash
# フォルダ名は実際に保存されたものへ変更してください
ros2 bag info ~/ros2_ws/manual_loop_20260915_213541
```

6トピックすべてのメッセージ数が0でないことを確認します。
任意のオプションは次のとおりです。

```bash
# 起動から600秒で自動終了（準備時間も含む）
ros2 launch experiment_cat manual_loop_test.launch.py duration:=600

# 名前を指定。既存のフォルダ名は使えません
ros2 launch experiment_cat manual_loop_test.launch.py bag_name:=indoor_loop_02 rviz:=false
```

VIO用設定は[zed_vio_test.yaml](config/zed_vio_test.yaml)です。
GEN_1 VIOとPERFORMANCE深度処理を使用し、深度画像・点群は配信しません。
通常のbringupよりGPU負荷が増えます。VIOはEKFには融合せず、独立した出力として記録します。
ただし同じIMUを共有するため、厳密に独立した正解データではありません。

## rosbagから2D占有格子地図を作成

実機のlaunchをすべて停止し、各ターミナルで「準備」のsourceを実行してください。

### ターミナル①：SLAM起動

```bash
ros2 launch slam_toolbox online_sync_launch.py \
  use_sim_time:=true \
  slam_params_file:=$HOME/ros2_ws/src/experiment_cat/config/slam_toolbox.yaml
```

### ターミナル②：設定済みRVizを開く

```bash
rviz2 -d ~/ros2_ws/src/experiment_cat/rviz/bag_mapping.rviz \
  --ros-args -p use_sim_time:=true
```

地図、LiDAR、車輪odom（オレンジ）、EKF（青）が設定済みです。
Fixed Frameは`map`です。VIOは別座標系なので、この画面には重ねません。

### ターミナル③：bag再生

以下のフォルダ名を自分のbagに置き換えてください。

```bash
ros2 bag play ~/ros2_ws/manual_loop_20260915_213541 \
  --clock --rate 0.5 \
  --topics /scan /tf /tf_static /wheel/odom /odom
```

半分の速度で再生します。地図作成だけなら`/scan /tf /tf_static`で足ります。
bagのTFが`odom → base_link → laser`を供給し、SLAMが`map → odom`を作ります。
別のEKFや実機ドライバーを同時に起動しないでください。
やり直す場合はSLAMも再起動し、RVizの履歴をリセットしてからbagを再生します。

### 地図を保存

再生と地図処理が終わってもSLAMは起動したままにし、別ターミナルで実行します。
既存の地図を残す場合は出力名を変更してください。

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/indoor_loop_map \
  --fmt pgm --free 0.25 --occ 0.65 \
  --ros-args -p use_sim_time:=false -p save_map_timeout:=10.0
```

`Map saved successfully`を確認します。
`indoor_loop_map.yaml`と`indoor_loop_map.pgm`の両方を保管してください。
保存ノードだけ実時間を使うことで、bagの時計が止まった後もタイムアウト処理が動きます。

## 車輪・EKF・VIOの軌跡を比較

このワークスペースの描画スクリプトは、ROSを再生せずbagを直接読みます。
スクリプトはパッケージの外にある`~/ros2_ws/analysis/`に置かれています。
別PCへ移す場合は、このスクリプトも別途コピーする必要があります。

```bash
PYTHONNOUSERSITE=1 MPLCONFIGDIR=/tmp/odom-plot-cache \
  /usr/bin/python3 ~/ros2_ws/analysis/plot_bag_trajectory.py \
  ~/ros2_ws/manual_loop_20260915_213541
```

出力先は`~/ros2_ws/analysis/バッグ名/`です。

| ファイル | 内容 |
|---|---|
| `trajectory_xy.png` / `.pdf` | 縦横同尺度のXY軌跡比較 |
| `trajectory.png` / `.pdf` | 軌跡と向きの時間変化 |
| `wheel_odom.csv` | 車輪軌跡 |
| `odom.csv` | EKF軌跡 |
| `zed_zed_node_odom.csv` | VIO軌跡 |
| `summary.json` | 終了位置、角度、積算移動距離など |

オレンジが車輪、青がEKF、緑がVIOです。
VIOは記録された固定TFを使い、カメラ位置から`base_link`位置へ変換します。
3本がそろう時間範囲を使い、開始位置・向きを合わせます。
これは推定軌跡の比較であり、実測値による正解軌跡ではありません。
VIOが再起動して位置がリセットされた場合、スクリプトは軌跡を自動でつなぎ直しません。
VIOが動かない場合は正解として評価せず、カメラの追跡状態を先に確認します。

## 保存した地図で自律走行

bag再生、SLAM、手動走行用launchを停止してから実行します。

```bash
ros2 launch experiment_cat navigation.launch.py \
  slam:=false map:=$HOME/ros2_ws/indoor_loop_map.yaml
```

1. RVizの`2D Pose Estimate`で地図上の現在位置と向きを指定します。
2. LiDARと地図の壁が重なることを確認します。
3. Aを押して`NAVIGATION`モードになったことを確認します。
4. 近い位置へ`Nav2 Goal`を指定して走行を確認します。

保存地図での自己位置推定はemcl2です。
`navigation.launch.py`だけを起動した場合は、既定の`slam:=true`でSLAMとNav2が起動します。
手動で地図を作るだけなら`mapping.launch.py`を使用します。

## オドメトリ校正テスト

### 接地状態で距離・角度を測る

各コマンドは1回ずつ実行します。記録準備後にカウントダウンし、自動走行するテストです。
実行ごとに位置を戻し、十分な走行スペースを確保してください。

```bash
ros2 launch experiment_cat odom_tests.launch.py test:=straight
ros2 launch experiment_cat odom_tests.launch.py test:=straight_12m
ros2 launch experiment_cat odom_tests.launch.py test:=left_arc
ros2 launch experiment_cat odom_tests.launch.py test:=right_arc
ros2 launch experiment_cat odom_tests.launch.py test:=spin
```

| テスト | 指令内容 |
|---|---|
| `straight` | 3 m直進、上限0.2 m/s |
| `straight_12m` | 12 m直進、上限0.2 m/s |
| `left_arc` / `right_arc` | 半径1 m、180°の円弧、上限0.2 m/s |
| `spin` | 左へ360°その場旋回、上限0.3 rad/s |

加減速を含む指令の積算値で距離・角度を設定しています。実測値と一致する保証はありません。
並進加減速は0.15 m/s²、その場旋回の角加速度は0.3 rad/s²です。
走行終了後にゼロ指令を送り、bagと各プロセスも自動停止します。
この校正テストは周回テストより多くの診断トピックを記録し、VIOは無効です。
`use_zed:=false`で車輪だけを検証できます。

### 車輪を浮かせて回転数を測る

車体を安定して固定し、タイヤに目印を付けてから実行します。
既定は片輪90 RPMを10秒間、指令上15回転です。

```bash
ros2 launch experiment_cat motor_rpm_calibration.launch.py motor:=right direction:=forward
ros2 launch experiment_cat motor_rpm_calibration.launch.py motor:=right direction:=reverse
ros2 launch experiment_cat motor_rpm_calibration.launch.py motor:=left direction:=forward
ros2 launch experiment_cat motor_rpm_calibration.launch.py motor:=left direction:=reverse
```

`Start counting now`から停止までの回転数を数えます。bagも自動で開始・終了します。
従来の一定時間走行用`odom_calibration.launch.py`もありますが、新しい距離校正には
`odom_tests.launch.py`を使ってください。

## センサ・TF・設定ファイル

### LiDARの方向別近距離フィルタ

`rplidar_s1.launch.py`を使うすべての構成（通常起動・周回記録・地図作成・Nav2）で有効です。
ドライバーの生データは`/scan_raw`、フィルタ後は`/scan`です。
車体前方を0°として±100°の200°範囲では0.1 m未満、残る後方160°では0.4 m未満をNaN（測定不明）にします。
距離はLiDARの測距原点からの距離です。点を間引かず、角度・時刻・座標系を保持します。
除外点は障害物として使わず、空き領域としてクリアするための無限遠値にも変換しません。
後方0.4 m以内の実際の障害物も除外されるため、後退時はその範囲を検出できません。

設定は[scan_filter.yaml](config/scan_filter.yaml)です。`mounting_yaw`はURDFのLiDAR取付方向（現在π rad）と一致させてください。
TFを待たず単体起動でも同じ判定を行うため、取付角を設定値として使用しています。
周回bagにはフィルタ後の`/scan`だけが入り、生データは保存しません。既存bagには後から適用されません。
変更後はlaunchを再起動してください。

左右は車体の前進方向を向いたときの左右です。

| 項目 | 現在の設定 |
|---|---|
| 左モーター | CH4、ID 2、USBシリアル末尾`if06` |
| 右モーター | CH3、ID 1、USBシリアル末尾`if04` |
| 実タイヤ半径 | 50 mm（URDF形状） |
| 制御・odom用の有効半径 | 50.65 mm |
| トレッド幅 | 208 mm |
| LiDAR座標 | `base_link`から前8 mm、左0 mm、上192 mm |
| LiDAR取付方向 | ヨー角180°（π rad）。スキャナの+X方向が車体後方 |
| 車輪フィードバック補正 | 符号付きRPMに`+0.5`、ゼロはゼロのまま |
| EKF周期 | 30 Hz |
| 通常のカメラ設定 | VGA 672×376、15 fps、VIO・深度処理OFF |

通常のEKFは車輪の前進速度・ヨー角速度と、IMUのヨー角速度を融合します。
IMUの姿勢・加速度、VIO、GNSSは融合しません。
IMUが無効・途絶の場合は`wheel_only`、正常なら`wheel_imu`になります。

TFの担当は、EKFが`odom → base_link`、SLAMまたはemcl2が`map → odom`です。
センサ取付位置はrobot_state_publisher、ZED内部のIMU変換はZEDが配信します。
ZED VIOは独自の`zed_odom`座標で出力し、競合するodom TFは配信しません。

| 設定ファイル | 調整内容 |
|---|---|
| [robot_nav2.yaml](config/robot_nav2.yaml) | モーター、車輪寸法、通信・odom周期 |
| [manual_control.yaml](config/manual_control.yaml) | 手動速度、曲率、不感帯、加減速 |
| [ekf.yaml](config/ekf.yaml) | 車輪・IMU融合 |
| [zed_sensors.yaml](config/zed_sensors.yaml) | 通常の軽量カメラ設定 |
| [zed_vio_test.yaml](config/zed_vio_test.yaml) | 周回テスト用VIO設定 |
| [slam_toolbox.yaml](config/slam_toolbox.yaml) | 地図作成 |
| [nav2_params.yaml](config/nav2_params.yaml) | Nav2経路追従、コストマップ、車体外形 |
| [experiment_robot.urdf.xacro](urdf/experiment_robot.urdf.xacro) | 車体・センサの形状と取付位置 |

有効半径などを変更する場合は、`ddsm115_controller_ros2/config/robot.yaml`との整合も確認します。
URDFの実寸と、走行校正用の有効半径は区別してください。
LiDARの取付方向は2026-09-16に0°から180°へ修正しました。
反映にはlaunchを再起動してください。修正前のbagに記録された固定TFは変わりません。

## 困ったとき

### RVizに何も表示されない

地図確認には`bag_mapping.rviz`を指定してください。
空のRVizにはMapやLaserScanが登録されていないことがあります。
bag再生前にRVizを開くと軌跡を最初から残せます。
bag再生終了後は新しいスキャンやodomが届かなくなります。

### 警告・エラー

| ログ | 判断 |
|---|---|
| `minimum laser range setting` | 設定0.05 mとscanの0.050000000745 mという微小差の場合は実害が小さい |
| `extrapolation into the future` | 起動時だけなら様子見。連続する場合はTFと再生時刻を確認 |
| `active samplers ...` | RVizの描画エラー。地図が表示・更新されない場合は対処が必要 |
| `Free/Occupied threshold unspecified` | 保存閾値に既定値を使う通知 |
| `Image format unspecified` | 保存画像形式に既定値を使う通知 |

描画エラーの切り分けにはRVizだけを閉じ、ソフトウェア描画を試せます。

```bash
LIBGL_ALWAYS_SOFTWARE=1 rviz2 \
  -d ~/ros2_ws/src/experiment_cat/rviz/bag_mapping.rviz \
  --ros-args -p use_sim_time:=true
```

### USB再接続・カメラ認識

```bash
lsusb
lsusb -t
ls -l /dev/serial/by-id/
ls -l /dev/rplidar
nvidia-smi
```

ZED MiniはHIDだけでなく、USB 3側の映像デバイスも認識される必要があります。
カメラを使うlaunchとZED Explorerを同時起動しないでください。
OS側にデバイスが出ない場合は、ROSの再起動だけでは解消しません。

モーターは通信断の継続を検知するとドライバーを終了し、対象launchが再起動します。
ZEDもwrapper終了後に再起動を試みます。OSの再認識やSDK初期化時間があるため、即時復帰は保証されません。
復帰時はNav2の目標をキャンセルし、スティックを中立・停止状態にしてから再開します。
ソフトウェアの再接続処理は物理的な緊急停止の代わりではありません。

### 現在の推定状態・TFを確認

```bash
ros2 topic echo /localization/imu_status --once --qos-durability transient_local
ros2 topic echo /wheel/odom --once
ros2 topic echo /odom --once
ros2 run tf2_ros tf2_echo base_link laser
ros2 run tf2_ros tf2_echo base_link zed_imu_link
```

自律走行前には、キャスタを含む車体外形がNav2のfootprint内に収まることと、
LiDAR・TF・実際の車体向きが一致することを確認してください。
