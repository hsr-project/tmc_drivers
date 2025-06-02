Overview
++++++++

提供機能
--------

IMUの異常検知機能を提供する。

sensor_msgs::Imuのデータを検証し、/diagnosticsを発行する。


ROS Interface
++++++++++++++

Nodes
-----

- **tmc_imu_diag_updater_node**

  sensor_msgs::Imuを検証し/diagnosticsを発行するノード。

Subscribed Topics
^^^^^^^^^^^^^^^^^

- **/imu/data_raw** (:ros:msg:`sensor_msgs/Imu`)

  検証するIMUトピック。

Published Topics
^^^^^^^^^^^^^^^^

- **/diagnostics** (:ros:msg:`diagnostic_msgs/DiagnosticsArray`)

  検証結果。

Parameters
^^^^^^^^^^

共通なパラメータについてはros-kinetic-tmc-diag-updater-commonのPKGDOC.rstを参照

- **~verifiers** (``map``)

  検証器のパラメータマップ。

  IMUの検証機毎に異なるパラメータがある。

  下記のパラメータのリストとなっている。

  具体的な設定例は launch/diag_updater.launch を参照。

  + **type** (``string``)

    検証器のタイプ。

    * zero_velocity_and_acceleration

      入力トピックに含まれる角速度、加速度が有意な値を持つか確認する検証器。

    * contiguous_same_value

      入力トピックの値が固まっていないか確認する検証器。

  + **significant_threshold** (``double``)

    zero_velocity_and_accelerationタイプのパラメータ。角速度・加速度の全成分がこの値を下回るとERROR状態と判断される。

  + **properties_num** (``int``)

    contiguous_same_valueタイプのパラメータ．更新が止まったプロパティの数がこの値以上になるとErrorとなりうる

  + **contiguous_threshold** (``int``)

    contiguous_same_valueタイプのパラメータ．properties_numで指定した数のプロパティの更新が止まった回数（連続）がこの値以上になるとErrorとなる

    * initial_acceleration_norm

      始動時の入力トピックに含まれる加速度が有意な値を持つか確認する検証器。


Services and Actions
^^^^^^^^^^^^^^^^^^^^

なし


How to use
++++++++++

1. 検証したいIMUのトピックが出力されていることを確認する。

.. code-block:: bash

    $ rostopic hz YOUR_IMU_TOPIC

2. ノードを起動する。

.. code-block:: bash

    $ roslaunch tmc_imu_diag_updater diag_updater.launch input_topic_name:=YOUR_IMU_TOPIC

3. 検証結果を確認する

.. code-block:: bash

    $ rostopic echo /diagnostics
