# サーボ関連エラーメッセージ一覧

## 目的

エラーメッセージを英語で出力する際に、統一的で分かりやすくなるようにしたい。

## 方針

方針を以下のように定め、メッセージ内容はこのドキュメントで管理するようにする。

1. 使用する単語は以下の通りに統一する。

   * failure: 故障。 例) sensor failure: センサ故障。
   * error: 異常。期待されているものと異なる状態。 例) communication error: 通信異常。
   * over～: 制限値を越えた。 例) over current: 過電流, over temperature: 温度高過。
   * reference: 指令 例) velocity reference: 速度指令

2. A is Bの場合、isを省略する。

   * 例) Error is too large -> Error too large

3. 簡単な英語を使う。

   * できるだけ平易な英単語を使う。例: deviation -> error

4. あいまいな表現はできれば避ける

   * Abnoraml(異常) -> Out of ragnge(範囲外)、Over (過多)
   * No ～ for a while(◯◯がしばらくこなかった)　-> ～ interrupted(中断された)

## メッセージ一覧

詳細については、exxx-firmwareの「ALARMについて」を参照。

`exxx-firmware/include/exxx_firmware/alarm_status.h`

### アラーム

| 定数 | bit | 日本語 | 英語 |
| ---- | --- | --- | --- |
| TEMPERATURE             | 0  | 基板温度過                  | Board over temperature |
| VOLTAGE                 | 1  | 電源電圧異常                | Supplied voltage out of range |
| OVERPOSITION            | 2  | 過位置                      | Abnormal position |
| CURRENT                 | 3  | モータ電流過大              | Over current |
| CORETEMP                | 4  | 電子サーマル                | Motor over temperature |
| DEVIATION_VELOCITY_IERR | 5  | 制御偏差過大                | Velocity integration error too large |
| DEVIATION_POSITION      | 6  | 指令値と現在値の偏差が過大  | Position error too large |
| CURRENT_SENSOR          | 7  | 電流センサ異常              | Current sensor failure |
| TOOTHJUMP               | 8  | 歯とび異常                  | Timing belt jumped |
| HALLSENSOR              | 9  | ホールセンサ異常            | Hall sensor failure |
| EEPROM                  | 10 | EEPROM書き込み失敗          | EEPROM failure |
| LOGIC                   | 11 | ロジックエラー              | Abnormal state in motor driver |
| COMMUNICATION           | 12 | 通信異常                    | Invalid packet size |
| ENCODERFAIL             | 13 | エンコーダ故障              | Encoder failure |
| JOINT                   | 14 | AVAGO異常                   | Avago sensor failure |
| FUNCTIONALSAFETY        | 15 | 機能安全アラーム(HSR-Cから) | Functional safety error |

### 機能安全アラーム(HSR-Cから)

| 定数 | bit | 日本語 | 英語 |
| ---- | --- | --- | --- |
| ----              | 0  | 3相電圧モニタ異常(現在は未使用) | --- |
| INIT_MOSFET_SHORT | 1  | 電源MOSFETショート異常          | Short error |
| INIT_MOSFET_OPEN  | 2  | 電源MOSFETオープン異常          | Open error |
| ----              | 3  | PBMショート異常(現在は未使用)   | --- |
| INIT_WD_SLOW      | 4  | ウォッチドッグslow監視機能異常  | Slow monitoring error |
| INIT_WD_FAST      | 5  | ウォッチドッグfast監視機能異常  | Fast monitoring error |
| COM_ROM           | 6  | ROMチェック異常(BLDC、DC)       | Check error |
| COM_SEQ           | 7  | シーケンスモニタ異常(BLDC、DC)  | Sequence error |
| COM_PBM           | 8  | インバータ電源異常2(BLDC、DC)   | PBM supply error |
| COM_12V           | 9  | インバータ電源異常1(BLDC、DC)   | 12V Power supply error |
| BLDC_ADD_VOLT     | 10 | 3相電圧加算異常                 | Three phase voltage addition error |
| BLDC_REV_OFFSET   | 11 | 回転角センサオフセット異常      | [BLDC]Unexpected sensor value |
| BLDC_POT_SENS     | 12 | 位置センサ異常                  | [BLDC]Two sensor value difference |
| BLDC_REV_SENS     | 13 | 回転角センサ範囲外異常          | Out of range |
| BLDC_ADD_CUR      | 14 | 3相電流加算異常                 | Three phase current addition error |
| DC_ADD_VOLT       | 15 | ２相電圧加算異常(DC)            | Two phase voltage addition error |
| DC_REV_MODEL      | 16 | 回転角センサモデル異常(DC)      | [DC]Unexpected sensor value |
| DC_POT_SENS       | 17 | 位置センサ異常                  | [DC]Two sensor value difference |
| MOTOR_SHORT       | 18 | モータ短絡異常                  | Short circuit error |

### 警告

| 定数 | bit | 日本語 | 英語 |
| ---- | --- | --- | --- |
| STATUS                        | 0  | alarm_statusが0以外のとき1になる            | Error occurred |
| VELOCITY_DOWN                 | 1  | 速度指令モードで速度指令値が途絶            | Velocity command interrupted |
| VELOCITY_RESTRICTION          | 2  | 速度指令モードで速度指令値が範囲外          | Velocity reference out of range |
| CURRENT_DOWN                  | 3  | 電流指令モードで電流指令値が途絶            | Current command interrupted |
| CURRENT_RESTRICTION           | 4  | 電流指令モードで電流指令値がclamp値を超えた | Current reference out of range |
| DUTY_DOWN                     | 5  | 電圧指令モードで電圧指令値が途絶            | Voltage command interrupted |
| DUTY_RESTRICTION              | 6  | 電圧指令モードで電圧指令値がclamp値を超えた | Voltage reference out of range |
| POSITION_RESTRICTION          | 7  | 位置指令モードで位置指令値がclamp値を超えた | Position reference out of range |
| ELECTRICAL_OVERLOAD           | 8  | 電流値が定格の100%を超えた                  | Electrical overload |
| POSITION_VELOCITY_RESTRICTION | 9  | 位置指令モードで速度指令値がclamp値を超えた | Velocity reference out of range |
| CHECKSUM                      | 10 | チェックサムエラー                          | Checksum error |
| INST                          | 11 | 未定義のインストラクション                  | Invalid instruction |
| ADDRESS_RANGE                 | 12 | アドレス範囲外                              | Invalid address on control table |
| JOINT_TO_MOTOR                | 13 | kInstAvago2Mr失敗                           | Motor position update failure |
| HAND_RESTRICTION              | 14 | ハンドのモードで指令値がclamp値を超えた     | Hand effort reference out of range |
| OTHER_DRIVE_MODE              | 15 | 指定していないモードが設定された            | Invalid drive mode |