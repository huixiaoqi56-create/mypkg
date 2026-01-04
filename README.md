# ROS 2 稼働時間、ネットワーク監視パッケージ
[![test](https://github.com/huixiaoqi56-create/mypkg/actions/workflows/test.yaml/badge.svg)](https://github.com/huixiaoqi56-create/mypkg/actions/workflows/test.yaml)
## 概要
- ROS 2 上で動作する，稼働時間（秒数）とネットワーク接続状態を監視する。
- ノードの起動からの稼働時間を秒単位でカウントする
- ネットワーク接続状態を定期的に監視する
- 監視結果を ROS 2 のトピック通信で送信する
- Listener 側で受信データを表示し，CSV 形式で自動保存する
- monitor の状態が異常になった場合にlistener が終了する
## 構成

### ノード一覧

| ノード名 | 役割 |
|--------|------|
| `system_health_monitor` | 稼働時間とネットワーク状態を送信 |
| `system_health_listener` | 情報を受信・ログ保存・オフライン検知 |
## トピック一覧
| トピック名 | 型 | 発行ノード | 内容 |
|-----------|----|-----------|------|
| `system_health` | `std_msgs/String` |` system_health_monitor` | `"<秒数>,<状態>"` 形式の文字列 |
## 各ノードの機能説明

### system_health_monitor

- 起動時に内部カウンタを 0 に初期化
- 1 秒周期で以下を実行
  - 秒数を +1
  - ネットワーク接続状態を判定
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

#### 実行例
- monitor 側出力例
```
$ ros2 run mypkg system_health_monitor
[INFO] [1767538402.852292280] [system_health_monitor]: SystemHealthMonitor started
[INFO] [1767538403.859948202] [system_health_monitor]: time=1s network=OK
[INFO] [1767538404.866195023] [system_health_monitor]: time=2s network=OK
[INFO] [1767538405.877352213] [system_health_monitor]: time=3s network=OK
[INFO] [1767538406.859002809] [system_health_monitor]: time=4s network=OK
[INFO] [1767538407.857260391] [system_health_monitor]: time=5s network=OK
[INFO] [1767538408.861190149] [system_health_monitor]: time=6s network=OK
[INFO] [1767538409.862830035] [system_health_monitor]: time=7s network=OK
[INFO] [1767538410.860987255] [system_health_monitor]: time=8s network=OK
[INFO] [1767538411.859499662] [system_health_monitor]: time=9s network=OK
[INFO] [1767538412.857269771] [system_health_monitor]: time=10s network=OK

```
- listener 側出力例
```
$ ros2 run mypkg system_health_listener
[INFO] [1767538408.327223378] [system_health_listener]: SystemHealthListener started, log=/tmp/system_health_log.csv
[INFO] [1767538408.862261987] [system_health_listener]: uptime=6s net=True
[INFO] [1767538409.863528564] [system_health_listener]: uptime=7s net=True
[INFO] [1767538410.861513704] [system_health_listener]: uptime=8s net=True
[INFO] [1767538411.859626506] [system_health_listener]: uptime=9s net=True
[INFO] [1767538412.857958638] [system_health_listener]: uptime=10s net=True

```
- launch ファイル出力例
```
 ros2 launch mypkg system_health.launch.py
[INFO] [launch]: All log files can be found below /home/hakozaki/.ros/log/2026-01-04-23-59-50-830671-key-354951
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [system_health_monitor-1]: process started with pid [354952]
[INFO] [system_health_listener-2]: process started with pid [354954]
[system_health_listener-2] [INFO] [1767538791.157426250] [system_health_listener]: SystemHealthListener started, log=/tmp/system_health_log.csv
[system_health_monitor-1] [INFO] [1767538791.157777064] [system_health_monitor]: SystemHealthMonitor started
[system_health_monitor-1] [INFO] [1767538792.171211873] [system_health_monitor]: time=1s network=OK
[system_health_listener-2] [INFO] [1767538792.172200762] [system_health_listener]: uptime=1s net=True
[system_health_monitor-1] [INFO] [1767538793.166664104] [system_health_monitor]: time=2s network=OK
[system_health_listener-2] [INFO] [1767538793.166916466] [system_health_listener]: uptime=2s net=True
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
