Overview
++++++++

提供機能
---------------------

キャラクタデバイスを介してつながるSANYOバッテリーパックと通信し、
バッテリーの現在状態を読み取る機能を提供します。

コマンドラインクライアントと、ROSノードの２つの使い方があります。

.. warning:: 現在の実装ではデバイスファイルにロックをかけていません。
   複数のノードを立ち上げたり、ノードとコマンドラインクライアントを併用すると、
   通信が混線し、予期せぬ結果につながる可能性があります。

ROS Interface
++++++++++++++

Published Topics
-----------------

- **battery_state** (:ros:msg:`sensor_msgs/BatteryState`) バッテリー情報

- **diagnostics** (:ros:msg:`diagnostics_msgs/DiagnosticArray`) バッテリーのダイアグ情報

Parameters
-------------

- **~device_name** (``string``: ``"/dev/sanyo-battery"`` ) 接続するデバイスファイルへのパス

- **~baudrate** (``int32``: ``38400``) 通信のボーレート[bps]

- **~publish_rate** (``float64``: ``1.0``) バッテリー情報の更新頻度 [Hz]

Diagnostics
---------------

`REP107`_ に準拠した診断情報を出力しています。

.. _REP107: http://www.ros.org/reps/rep-0107.html

- **summary**

  OK
     問題なし
  WARN
     バッテリー残量が50%以下
  ERROR
     バッテリー残量が20%以下、あるいはバッテリーパック側でエラー発生(原因はメッセージに記録)

- **battery_level**

  バッテリー残量[%]

- **full_charge_capacity**

  バッテリー総容量[Ah]

- **remaining_charge**

  バッテリー残容量[Ah]

- **electric_current**

  放電電流[A] (放電時は＋、充電時はー)

- **voltage**

  電池の全体電圧[V]

- **temperature**

  電池パック内温度[deg C]

- **zero_percent_detected**

  ===== =============
  False 0%検出以外
  True  0%検出
  ===== =============

- **discharge_enabled**

  ===== =============
  False 放電停止
  True  放電許可
  ===== =============

- **charge_enabled**

  ===== =============
  False 充電停止
  True  充電許可
  ===== =============

- **over_discharge**

  ===== =============
  False 過放電以外
  True  過放電
  ===== =============

- **full_charge**

  ===== =============
  False 満充電状態以外
  True  満充電状態
  ===== =============

- **learning_enabled**

  ===== ==========
  False 学習禁止
  True  学習許可
  ===== ==========

- **triple_parallel**

  ``True`` に固定

- **over_charge**

  ===== ==========
  False 過充電以外
  True  過充電
  ===== ==========

How to use
++++++++++

本パッケージにはコマンドラインクライアントとROSノードが含まれています。
以下でそれぞれの使い方を説明します。

.. note::

    コマンドラインクライアントとROSノードはHSRBのみ利用可能です。


ROSノード
---------------------------

下記コマンドでROSノードが起動し、上述のトピックを出版開始します。

.. code-block:: bash

    $ ros2 run tmc_sanyo_battery sanyo_battery_node


各種パラメータはROSの標準的な方法で設定することができます

コマンドラインクライアント
---------------------------

コマンドラインクライアントは直接呼び出すことができます。

.. code-block:: bash

    $ ros2 run tmc_sanyo_battery sanyo_battery_status

デフォルトではバッテリー残量のみを返します。
詳細は付随情報を得たい場合は ``-v`` オプションを使用して下さい。

オプション
~~~~~~~~~

-h, --help
    ヘルプを表示
-d, --device
    デバイスファイルを指定する
-v, --verbose
    詳細情報を表示(Diagnosticsの情報と同じ)


