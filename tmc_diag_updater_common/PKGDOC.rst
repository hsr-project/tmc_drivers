Overview
++++++++

提供機能
--------

任意のstampedなトピックを検証し、Diagを発行するためのテンプレートクラスを提供する。


ROS Interface
++++++++++++++

Subscribed Topics
^^^^^^^^^^^^^^^^^

パラメータhardware_idで設定したトピックを購読する。
型はテンプレートクラスの利用者依存。

Published Topics
^^^^^^^^^^^^^^^^

- **/diagnostics** (:ros:msg:`diagnostic_msgs/DiagnosticsArray`)

  検証結果。

Parameters
^^^^^^^^^^

- **~hardware_id** (``string`` : ``data``)

  検証結果に載せる自身の名前。

  入力トピック名と同義としている。

- **~sampling_hz** (``double`` : ``100.0``)

  サンプリング周期[hz]。

- **~verifiers** (``map``)

  検証器のパラメータマップ。

  検証機毎に異なるパラメータがある。

  下記のパラメータのリストとなっている。

  + **type** (``string``)

    検証器のタイプ。

    * disconnection

      入力トピックの通信が途切れてないか確認する検証器。

    * unexpected_rate

      期待した周期で入力トピックが得られているか確認する検証器。

    * unexpected_frame_id

      期待したフレームIDの入力トピックが得られているか確認する検証器。

  + **timeout_sec** (``double``)

    disconnectionタイプのパラメータ。通信が途切れたと判断するタイムアウト時間[sec]。

  + **warn_hz** (``double``)

    unexpected_rateタイプのパラメータ。この周期[hz]を下回るとWARN状態と判断される。

  + **error_hz** (``double``)

    unexpected_rateタイプのパラメータ。この周期[hz]を下回るとERROR状態と判断される。

  + **window_size** (``int``)

    unexpected_rateタイプのパラメータ。この周期の計算に用いるキューの個数。

  + **expected_frame_id** (``string``)

    unexpected_frame_idタイプのパラメータ。期待するフレームID。

Services and Actions
^^^^^^^^^^^^^^^^^^^^

なし

