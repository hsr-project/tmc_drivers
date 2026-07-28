# tmc_computer_monitor

## ノード名  
- tmc_computer_monitor_node : diagnosticsでコンピュータ情報を異常通知するノード  

## 概要  
- HSRのCPUなどの変化を監視してダイアグを発行する  
- CommandMonitorを継承した外部モジュールのダイアグを発行する  
- LsusbCommandMonitorはCommandMonitorを継承し、lsusb linuxコマンドを使用してusbデバイスのダイアグを発行する
- UdevCommandMonitorはCommandMonitorを継承し、udevadm linuxコマンドを使用してデバイスマネージャ情報ダイアグを発行する。


#### YAML名 ([full_diag_list.yaml](./config/full_diag_list.yaml))
[Parameter パラメータ](https://tmc-dev-xr.atlassian.net/wiki/spaces/TMHQibitech/pages/8309047524/tmc_computer_monitor#4.-%E3%83%86%E3%83%BC%E3%83%96%E3%83%AB%E5%AE%9A%E7%BE%A9)
##### FileMonitor
ファイルの内容を確認してダイアグするクラス
##### ProcMemoryMonitor
/proc/{pid}からメモリ使用量を確認してダイアグを生成するクラス
##### ComputerMonitor一覧
何かしらのコマンドを実行してダイアグ監視をするためのベースクラス

* CommandMonitor

* DfonfigCommandMonitor

* IwConfigCommandMonitor

* LsusbCommandMonitor

* MemInfoCommandMonitor

* MpStatCommandMonitor

* NetstatCommandMonitor

* PsCommandMonitor

* SensorCommandMonitor


## ROS Interface
### Published Topic

| トピック名 (Topic Name) | 型 (Type)                             | 内容 (Content)                                 |
|-------------------------|-------------------------------------|-----------------------------------------------|
| `/diagnostics`          | [`diagnostics_msgs/msg/DiagnosticsArray`](https://docs.ros.org/en/humble/p/diagnostic_msgs/interfaces/msg/DiagnosticArray.html) | 異常通知のデータ (Data for abnormality notifications) |

#### 環境構築  
```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select tmc_computer_monitor
```  
CommandMonitorを継承した外部モジュールを利用する際は、ROS2もしくはpythonパッケージとしてインストールすること。  

### 使い方  
#### Launch  
```bash
ros2 launch tmc_computer_monitor tmc_computer_monitor.launch.py
```

#### Launch GUI付き  
```bash
ros2 launch tmc_computer_monitor tmc_computer_monitor_gui.launch.py
```

#### Node単体  
```bash
ros2 run tmc_computer_monitor tmc_computer_monitor_node
```
yamlファイルをロードした後、
```bash
ros2 param set tmc_computer_monitor_node config_files full_diag_list.yaml
```
異常通知の内容が反映される。
```bash
ros2 topic echo /diagnostics
```   
yamlファイルロードは繰り返し再実行可能。

### 参照  
[tmc_computer_monitor](https://tmc-dev-xr.atlassian.net/wiki/spaces/TMHQibitech/pages/8309047524/tmc_computer_monitor)
