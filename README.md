# ROS 2 稼働時間、ネットワーク監視パッケージ

## 概要
- ROS 2 上で動作する，稼働時間（秒数）とネットワーク接続状態を監視する。
- ノードの起動からの稼働時間を秒単位でカウントする
- ネットワーク接続状態を定期的に監視する
- 監視結果を ROS 2 のトピック通信で送信する
- Listener 側で受信データを表示し，CSV 形式で自動保存する
- どちらかの状態が異常になった場合にプログラムを終了する
## 構成

### ノード一覧

| ノード名 | 役割 |
|--------|------|
| `/system_health_monitor` | 稼働時間とネットワーク状態を送信 |
| `/system_health_listener` | 情報を受信・ログ保存・オフライン検知 |
## トピック一覧
| トピック名 | 型 | 発行ノード | 内容 |
|-----------|----|-----------|------|
| `/system_health` | `std_msgs/String` |` system_health_monitor` | `"<秒数>,<状態>"` 形式の文字列 |
## 各ノードの機能説明

### system_health_monitor

- 起動時に内部カウンタを 0 に初期化
- 1 秒周期で以下を実行
  - 秒数を +1
  - ネットワーク接続状態を判定（簡易）
  - `/system_health` トピックへ publish
- 標準出力に現在の状態を表示
### system_health_listener

- `/system_health` トピックを subscribe
- 受信したデータを CSV 形式で自動保存
- 一定時間メッセージが来なければ  
  **「monitor offline」と判断してエラー表示後に終了**

## 動作仕様

- 秒数は monitor 起動からの経過時間
- monitor が一定時間 publish しなくなると listener が異常終了
- listener 側でログを自動生成
- Ctrl+C による終了も正常系として扱う

## 使用方法

### インストール

- 以下の手順でインストールしてください。

#### リポジトリの取得

```bash
cd ~/ros2_ws/src
git clone https://github.com/huixiaoqi56-create/mypkg1.git
```
### ビルド
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```
### 実行方法
#### monitor
- 稼働時間とネットワーク状態を監視し，トピックに送信します。
```bash
source ~/ros2_ws/install/setup.bash
ros2 run mypkg system_health_monitor
```
#### listener
- 受信したデータを表示し，CSV ファイルとして自動保存します。
```bash
source ~/ros2_ws/install/setup.bash
ros2 run mypkg system_health_listener
```

#### launch
```bash
ros2 launch mypkg system_health.launch.py
```
#### 実行例
- monitor 側出力例
```
[INFO] [1767056060.597950951] [system_health_monitor]: SystemHealthMonitor started
[INFO] [1767056061.607781104] [system_health_monitor]: time=1s network=OK
[INFO] [1767056062.599538834] [system_health_monitor]: time=2s network=OK
[INFO] [1767056063.601983013] [system_health_monitor]: time=3s network=OK
[INFO] [1767056064.601477438] [system_health_monitor]: time=4s network=OK
[INFO] [1767056065.600307682] [system_health_monitor]: time=5s network=OK
[INFO] [1767056066.600541314] [system_health_monitor]: time=6s network=OK

```
- listener 側出力例
```
[INFO] [1767056064.318518573] [system_health_listener]: SystemHealthListener started, log=/tmp/system_health_log.csv
[INFO] [1767056064.602282863] [system_health_listener]: uptime=4s net=True
[INFO] [1767056065.600720977] [system_health_listener]: uptime=5s net=True
[INFO] [1767056066.601213502] [system_health_listener]: uptime=6s net=True

```
- launch ファイル出力例
```
[INFO] [system_health_monitor-1]: process started with pid [347728]
[INFO] [system_health_listener-2]: process started with pid [347730]
[system_health_listener-2] [INFO] [1767056287.068834439] [system_health_listener]: SystemHealthListener started, log=/tmp/system_health_log.csv
[system_health_monitor-1] [INFO] [1767056287.070842722] [system_health_monitor]: SystemHealthMonitor started
[system_health_monitor-1] [INFO] [1767056289.806589422] [system_health_monitor]: time=2s network=OK
[system_health_listener-2] [INFO] [1767056289.807860252] [system_health_listener]: uptime=2s net=True
[system_health_monitor-1] [INFO] [1767056290.076652404] [system_health_monitor]: time=3s network=OK
[system_health_listener-2] [INFO] [1767056290.077386418] [system_health_listener]: uptime=3s net=True

```
### ログの確認方法
- CSV ログの内容を表示
```
cat ~/ros2_ws/log/system_health.csv
```
#### ログ出力例
```
uptime_sec,network_ok,recv_time
2,True,1767056289.8065047
3,True,1767056290.0766027
4,True,1767056291.075956
```
## 必要なソフトウェア

- ROS 2 Humble
- Python 3
  - テスト済みバージョン: Python 3.8 ~ 3.10

## テスト環境
- Ubuntu 22.04 LTS
- ROS 2 Humble

## ライセンス
- このソフトウェアパッケージは、3条項BSDライセンスの下で再頒布および使用が許可されます。

© 2025 hakozaki teruki
