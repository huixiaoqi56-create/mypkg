# ROS 2 稼働時間、ネットワーク監視パッケージ

- ROS 2（Humble）環境で動作する，稼働時間（秒数カウント）とネットワーク接続状態を監視するパッケージです。


## 概要

- ノードの起動からの稼働時間を秒単位でカウントする
- ネットワーク接続状態を定期的に監視する
- 監視結果を ROS 2 のトピック通信で送信する
- Listener 側で受信データを表示し，CSV 形式で自動保存する
- どちらかの状態が異常になった場合にプログラムを終了する



## 使用方法

### インストール方法

- 以下の手順でインストールしてください。

#### 1. リポジトリの取得

```bash
cd ~/ros2_ws/src
git clone https://github.com/huixiaoqi56-create/mypkg1.git
```
####2. ビルド
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
####実行方法
#####モニタノード
- 稼働時間とネットワーク状態を監視し，トピックに送信します。
```bash
source ~/ros2_ws/install/setup.bash
ros2 run mypkg system_health_monitor
```
#####リスナノード
- 受信したデータを表示し，CSV ファイルとして自動保存します。
```bash
source ~/ros2_ws/install/setup.bash
ros2 run mypkg system_health_listener
```

#####launch ファイル
```bash
ros2 launch mypkg system_health.launch.py
```
#####実行例
- モニタ側出力例
```
[INFO] [system_health_monitor]: time=5s network=OK
[INFO] [system_health_monitor]: time=6s network=OK
```
- リスナ側出力例
```
[INFO] [system_health_listener]: logged: 5,OK
[INFO] [system_health_listener]: logged: 6,OK
```
- launchファイル出力例
```

```
###ログの確認方法
- CSV ログの内容を表示
```
cat ~/ros2_ws/log/system_health.csv
```
####稼働時間のみ抽出（パイプ使用例）
```
cat ~/ros2_ws/log/system_health.csv | cut -d, -f1
```
##使用トピック
- `/system_healthstd_msgs/String`稼働時間とネットワーク状態
##必要なソフトウェア

- ROS 2 Humble
- Python 3
  - テスト済みバージョン: Python 3.10

##テスト環境
- Ubuntu 22.04 LTS
- ROS 2 Humble

##ライセンス
- このソフトウェアパッケージは、3条項BSDライセンスの下で再頒布および使用が許可されます。

© 2025 hakozaki teruki
