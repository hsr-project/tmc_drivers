tmc_imu_diag_updater
=====================

任意のsensor_msgs::Imuトピックを検証し、Diagを発行する


開発関係者
----------

* 松野 喜幸
* 川田 福和


I/Fおよび使用方法について
-------------------------

PKGDOC.rstに記載


設計方針
--------

同一のインターフェースクラスを継承した検証クラスを用いて、  
sensor_msgs::Imuトピックの検証を複数種類行う。

購読イベントと検証イベントは別にし、  
sensor_msgs::Imuトピック発行周期を含めて検証できるようにする。


Diagの振る舞い
--------------

それぞれの検証器が上位へDiagオブジェクトを返す。  
上位は決められた優先度に基づいて、Nodeとして出版するDiagを決定する。  
それはdiagnostic_updater::DiagnosticStatusWrapperの仕様に基づく。

それぞれの検証器から上位に返されるDiagを下表に示す。

- 共通

| Level | Message |
| ---- | ---- |
| OK | "OK" |
| ERROR | "Has no verified data" |

- 個別

| Name of verifier | Level | Message |
| ---- | ---- | ---- | ---- |
| zero_velocity_and acceleration | ERROR | "Velocities and accelerations are zero" |
| contiguous_same_value | ERROR | "Contiguous same value" |
| initial_acceleration_norm | ERROR | "Initial Acceleration Norm is invalid" |
