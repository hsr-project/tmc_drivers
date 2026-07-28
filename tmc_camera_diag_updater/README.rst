Overview
++++++++

提供機能
--------

- カメラ系のデバイスのダイアグを発行する


ROS Interface
++++++++++++++

Nodes
-----

- **/camera/camera_nodelet_manager**  ダイアグ発行用ノードレットマネージャ

- **/camera/color_img_diag_updater** Xtionカラー画像用ダイアグ発行ノードレット
- **/camera/depth_img_diag_updater** Xtion深度画像用ダイアグ発行ノードレット
- **/camera/points_diag_updater** Xtion点群データ用ダイアグ発行ノードレット

Subscribed Topics
^^^^^^^^^^^^^^^^^

1つのノードレットで、診断したい1つのトピックを購読する。
トピック名はパラメータで指定する。

- **/camera/rgb/image_rect_color** (:ros:msg:`sensor_msgs.msg/Image`) カラー画像データ
- **/camera/depth_registered/image_raw** (:ros:msg:`sensor_msgs.msg/Image`) 深度画像データ
- **/camera/depth_registered/rectified_points** (:ros:msg:`sensor_msgs.msg/PointCloud2`) 点群データ

Published Topics
^^^^^^^^^^^^^^^^^

- **/diagnostics** (:ros:msg:`diagnostic_msgs.msg/DiagnosticArray`) 診断結果


Parameter
^^^^^^^^^

- **~topic_name** (string)

  診断するトピック名

- **~topic_type** (string: 'image' or 'points')

  診断するトピックの型
  :ros:msg:`sensor_msgs.msg/Image`または:ros:msg:`sensor_msgs.msg/PointCloud2`のみに対応

- **~warning_hz** (double)

  この値より小さいときWARNを発行するトピック購読周期 [Hz]

- **~hardware_id** (string: ~topic_name)

  診断する対象を表す任意のID

- **~unicolor_check** (bool: true)

  Image型のデータが1色で塗りつぶされていないかを調べるかどうかのフラグ

  encodingはrgb8またはbgr8のみに対応

- **~diag_pub_rate** (double: 1.0)

  ダイアグ発行周期 [Hz]

- **~boot_timeout** (double: 60.0)

  1個目のメッセージを購読するまでにかかる時間の許容値 [s]

- **~sub_timeout** (double: 10.0)

  この時間メッセージが購読できなければトピックが止まっていると判断する値 [s]

- **~max_window_size** (uint32_t: 10000)

  周期の計算に使うメッセージ数の最大値

- **~filling_rate** (double: 0.75)

  カラーチェックで異常と判断するときの画面の塗りつぶされた割合

- **~color_check_r** (uint8_t: 0)

  カラーチェックで調べる色のR値

- **~color_check_g** (uint8_t: 154)

  カラーチェックで調べる色のG値

- **~color_check_b** (uint8_t: 0)

  カラーチェックで調べる色のB値

- **~sampling_size_x** (uint32_t: 10)

  カラーチェック時の幅方向のサンプリングサイズ

- **~sampling_size_y** (uint32_t: 10)

  カラーチェック時の高さ方向のサンプリングサイズ


Internal
++++++++

.. ifconfig:: internal

   振る舞い:
     * パラメータで指定したカメラ系のトピックのダイアグを発行する。

+-------+--------------------------------------------------------------+-------------------------------------------------------+
| level | message                                                      | meaning                                               |
+=======+==============================================================+=======================================================+
| STALE |                                                              | ダイアグが発行されていない                            |
+-------+--------------------------------------------------------------+-------------------------------------------------------+
| ERROR | - Boot timeout                                               | - 起動してから一度もメッセージが届かない              |
|       | - Fill image with unicolor                                   | - データが一色で塗りつぶされていないか                |
|       | - No New Messages                                            | - メッセージが一定時間以上届いていない                |
+-------+--------------------------------------------------------------+-------------------------------------------------------+
| WARN  | - Only one message is subscribed. Cannot calculate frequency | - メッセージが1個しか届いておらず、周期計算ができない |
|       | - Subscribing Rate is slow                                   | - データの購読周期が遅い                              |
+-------+--------------------------------------------------------------+-------------------------------------------------------+
| OK    | - Boot Now                                                   | - カメラ起動中                                        |
|       | - OK                                                         | - メッセージ異常無し                                  |
+-------+--------------------------------------------------------------+-------------------------------------------------------+

     * key value 最終購読時刻、実測周期、周期計算に使ったメッセージ数、ワーニングを出さない最小の周期
