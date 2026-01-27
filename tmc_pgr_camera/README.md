このパッケージについて
================
カメラの画像を取得して、sensor\_msgs/Image型のトピックをPublishするノードです。


開発関係者
================
* 高橋武史


launchファイル種類  
================
* capture.launch : カメラ起動ベースファイル
* single.launch : 単眼カメラ起動用[not nodelet]
* single_camera_nodelet.launch : 単眼カメラ起動用[nodelet]
* stereo.launch : ステレオカメラ起動用[not nodelet]
* stereo_camera_nodelet.launch : ステレオカメラ起動用[nodelet]


カメラノード（camera_nodeのインタフェース）
================
- 購読するトピック
    - なし

- 出版するトピック(単眼カメラ)
    - *~/image_raw*

- 出版するトピック(ステレオカメラ)
    - *~/left/image_raw*
    - *~/right/image_raw*

- 提供するサービス
    - *control_camera*(tmc_vision_msgs/ControlCamera) キャプチャの開始/停止

- パラメータ
    - ~camera_setting_file_path(str, default: "") : カメラの設定ファイルのパス
    - ~frame_id(str, default: "pgr_stereo_camera") : カメラのTFでの座標名
    - ~auto_capture_start(bool, default: true) : カメラ起動時にすぐキャプチャするかのフラグ
    - ~setting(dictionary) : カメラパラメータを設定できる  
    propertyキー配下の配列に下記のような辞書型を追加することで設定可能  
    使用できるタイプは、brightness、auto_exposure、sharpness、white_balance、hue、saturation、gamma、iris、focus、zoom、pan、tilt、shutter、gain、trigger_mode、trigger_delay、frame_rate、temperature  
    設定できる値は、present: bool、absControl: bool、onePush: bool、onOff: bool、autoManualMode: bool、valueA: unsigned int、valueB: unsigned int、absValue: float、reserved: unsigned int[8]
    ```
    $ rosparam set /stereo_camera_node/setting "{ property: [ { type: brightness, absValue: -10.0, onOff: on, autoManualMode: off, onePush: on } ] }"

    ```

カメラ設定ファイルについて
================
カメラのシリアルナンバーやパラメータを設定しています(*.yml)。

* PGRカメラのシリアルナンバーは、flycaptureをインストール後、  
```
$ flycap
```
もしくは、アプリケーション->PointGrayResearch->FlyCap2 <GUI>
で確認することができます。  
ここで、何も表示されない場合は、カメラの接続やドライバを確認してください。

* その他の設定は、カメラに合わせて設定ください。
* format\_modeの解像度は、キャリブレーションファイルと合わせるようにしてください。  
format\_modeのカラーは、RGBを設定してください。
* 2017年10月からシリアルナンバーを除く全てのパラメータをファイルで設定することを廃止しました。
  以下のようなコマンドで不要なパラメータを削除できます。(ワーニングを回避できます。）
  $ rosrun tmc_pgr_camera update_config /home/usr/.ros/tmc/robot/conf.d/stereo_pgr_camera.yml
  sudo権限が必要なフアイルを更新する場合は直接バイナリを実行してください。
  $ sudo /opt/ros/kinetic/lib/tmc_pgr_camera/update_config /etc/opt/tmc/robot/conf.d/stereo_pgr_camera.yml

flycapture2のOS別バージョン
===================================
Indigo@trusty 2.7.3.13
Kinetic@xenial 2.11.3.121


その他
===================================
カメラの起動がうまくできない場合は、以下をご確認ください。

* セッティングファイル記載のカメラのシリアルナンバーが合っているか
* セッティングファイルで、カメラに合わないパラメータを設定していないか
* 前回、異常終了していないか
　（異常終了している場合は、カメラの抜き差しをお願いします）
* NVIDIAのドライバが動作していないか
* パラメータで設定したファイルが存在するか
* $ sudo udevadm trigger を実行
