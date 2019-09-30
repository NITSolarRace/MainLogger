/*更新情報
 * 2017BWSCに使用したプログラムがベース、以下にその変更履歴を示す

 2019/05/17
Zのプログラムを参考にパルス追加する変更箇所: 73・112・137・144・161・198・518-519・692・757-759行目
　　##タイヤ直径は73行目の計算で調整
XbeeのgpsSpeedをmotorSpeed_pulseに変更608行目（Xbee :: gps速度→パルス速度)
lcd表示のmotorSpeedをmotorSpeed_pulseに変更624行目（lcd :: can速度→パルス速度）
 *
 * 2019/06/19
 * CAN通信対応に伴う変更
 * LogData構造体 dataAryMITSUBA, dataArySolarLog, bmsVoltage　削除
 *               MITSUBA, SOLARLOGGER, BMSに各変数を追加
 * CAN通信 1秒間隔でSendCANMessageを実行、受信割り込みでReceiveCANMessageを実行
 * USBメモリ保存データ追加 MITSUBA, SOLARLOGGER, BMSデータ ファイル別保存
 *2019/8/10
 *const char USB_HEADER_STRING[] のBSMの電力、積算電力Ah、平均電流、平均電力、残量％、パルス速度の項を削りました
 *ファンなんて強弱変えないので削除
 *パルス速度は使わない方針で考えています
 */
 /*2019_9_15
 
 貫通型電流センサーのINA２２６のプログラム修正
 グローバル変数vAl[]に補正値（LCDに表示する値）をいれたがdata->realtimeata[][]には反映されていない模様
 LCD更新の時間が長い事象が発生なう*
 */
/*
* 2019/09/21
* CAN通信 BMS バッテリエラー表示対応
* BMSからバッテリ異常情報・合計電圧、最小電圧、最大電圧のフレーム１秒間隔で送信
* 受信割り込みで処理 DisplayBMSError()
* SOLARLOGGER保存用プログラムコメントアウト
*/

#include "mbed.h"
#include "INA226.hpp"
#include "TextOLED.h"
#include "MSCFileSystem.h"
#include "RTC.h"
#include "MBed_Adafruit_GPS.h"

// INA226用I2Cアドレス I2Cアドレスはmbedの場合1ビット左シフトしたアドレスになる
const int BATTERY_MONITOR_ADDRESS = 0x80;         // 0b10000000 (GG)
const int BATTERY_MON2_ADDRESS = 0x94;             // 0b10010100 (DD) 
const int PANEL_MONITOR_ADDRESS = 0x82;           // 0b10000010 (G1)
const int MOTOR_MONITOR_ADDRESS = 0x9E;           // 0b10011110 (CC)
const unsigned short INA226_CONFIG = 0x4897;    // 平均化処理やタイミング

/*------------------------------設定変更厳禁------------------------------*/
// const double VOLTAGE_CALIB = 10 * 1.0016;         // 分圧比(10倍に補正)
const double MITSUBA_VOLTAGE_CALIB = 0.93396;     // ミツバCANlogger 電圧補正 誤差0.7%程度へ
const unsigned short INA226_CALIB = 0xF00;      // シャント電圧補正係数
const unsigned short INA226_CALIB_PANEL = 0x3555; // シャント電圧補正係数（パネル）
// const double currentCalibration[] = {2.00, 0.375, 2.00};
// [0]: バッテリ [1]: パネル [2]: モータ
/*------------------------------設定変更厳禁------------------------------*/
const double BATTERY_CAPACITY = 3.6 * 3.45 * 26 * 16;       // 公称電圧*電流容量*直列数*並列数
// const double MOTOR_OVER_TEMP = 150;  // コイル温度150℃で過温度

// const int OFFSET_TIME = 60 * 60 * 9;     // 9時間(標準時との時差) = 日本時間
const int OFFSET_TIME = 60 * 60 * 9.5;     // ダーウィン時間
const int STORAGE_TIME = 1;             // 1秒間隔Xbee送信

const double NOTS_CONVERT = 1.852;    // 1ノット = 1.852 km/h
const double RPM_CONVERT = 0.104399;  // rpm -> km/h 変換係数
const double RPM_CONVERT_MS = 0.028955;     // rpm -> m/s 変換係数
//const double pulseConvert = 0.1303;     // 車速パルス -> 速度変換係数  1[pulse/sec]÷48[pulse/回転数]×直径0.553[m]×3.14159265×3.6[sec・km/m]

/*各配列要素数*/
const int RealtimeData_i = 3;  // [0]: バッテリ [1]: パネル [2]: モータ
const int RealtimeData_j = 4;  //532行～535行でいくつかの項目を削除したため6から４へ
const int timeStr_i = 3;

// バッファーサイズ
const int SERIAL_MAX = 250;
const int TIME_MAX = 32;
const int FILE_NAME_MAX = 32;

// ヘッダー
const char USB_HEADER_STRING[] = "Time,timeNoon[s],B[V],B[A],S[A],M[A],B[Wh],S[Wh],M[Wh],Remain[Wh],Mspeed[km/h],GPSspeed[km/h],gpsAngle,distance[km],MotorTemp";
const char HEADER_CAN_MITSUBA[] = "Time,timeNoon[s],voltage,current,FET,RPM,duty,angle";
// const char HEADER_CAN_SOLAR[] = "Time,timeNoon[s],MPPT_V1,MPPT_V2,MPPT_V3,MPPT_V4,MPPT_V5,MPPT_V6,MPPT_V7,MPPT_V8,MPPT_V9,MPPT_V10,MPPT_V11,MPPT_V12,MPPT_I1,MPPT_I2,MPPT_I3,MPPT_I4,MPPT_I5,MPPT_I6,MPPT_I7,MPPT_I8,MPPT_I9,MPPT_I10,MPPT_I11,MPPT_I12,MPPT_P1,MPPT_P2,MPPT_P3,MPPT_P4,MPPT_P5,MPPT_P6,MPPT_P7,MPPT_P8,MPPT_P9,MPPT_P10,MPPT_P11,MPPT_P12, T1,T2,T3,T4,T5,T6,T7,T8,T9,T10,T11,T12,T13,T14,T15,T16";
// const char HEADER_CAN_BMS[] = "Time,timeNoon[s],sum,c1,c2,c3,c4,c5,c6,c7,c8,c9,c10,c11,c12,c13,c14,c15,c16,c17,c18,c19,c20,c21,c22,c23,c24,c25,c26,c27,c28";
const char HEADER_CAN_BMS[] = "Time,timeNoon[s],sum,min,max";

Serial debug(USBTX, USBRX);
Serial Xbee(p13, p14);
Serial gpsSerial(p28, p27);
Adafruit_GPS myGPS(&gpsSerial);

CAN canBusSystem(p30, p29);

// 引数は(rs,e,d4,d5,d6,d7) 接続しないピンはGNDに落としておくと安定する
TextOLED lcd(p15, p16, p17, p24, p25, p26, TextOLED::LCD20x4);

I2C i2c(p9, p10);  // sda, scl
INA226 BatteryMonitor(i2c, BATTERY_MONITOR_ADDRESS, 10000);
INA226 PanelMonitor(i2c, PANEL_MONITOR_ADDRESS, 10000);
INA226 MotorMonitor(i2c, MOTOR_MONITOR_ADDRESS, 10000);
INA226 BatteryMonitor2(i2c, BATTERY_MON2_ADDRESS, 10000);

MSCFileSystem msc("usb");  // USBメモリには/usb/...でアクセス

AnalogIn motorTempAnalogIn(p19);  // モータコイル温度
//I nterruptIn motorPulse(p12);      // モーターパルス(ピン設定)


struct LogData {
  char timeStr[timeStr_i][TIME_MAX];
  /* [0]:タイムスタンプ    20160801121356
     [1]:現在時刻          12:13:56
     [2]:UNIXTIME          1900年1月1日からの経過時間
   */
  unsigned int totalTime;
  int totalTimeNoon;            // 正午を0とした経過時間
  double RealtimeData[RealtimeData_i][RealtimeData_j];  // 電流、電力、平均、積算
  double batteryVoltage;        // 電圧
  double remainBatteryWh;       // 残量[Wh]
  //double remainBatteryParcent;  // 残量[%]
  double motorTemp;             // モーターコイル温度
  double motorSpeed;            // モーター速度[km/h]
  double gpsSpeed;              // GPS対地速度[km/h]
  double gpsAngle;              // 進行方向
  double distanceMotorSpeed;    // モーター速度から距離[km]
  double latitude;              // 緯度
  double longitude;             // 経度
  //double motorSpeed_pulse;      // モーター速度[km/h] byパルス

  // CAN MITSUBA
  double motorVoltage;  // MITSUBA CAN 電圧
  double motorCurrent;  // MITSUBA CAN 電流
  double tempFET;       // MITSUBA CAN FET温度
  double motorRPM;      // MITSUBA CAN 回転数
  double duty;          // MITSUBA CAN duty比
  double angle;         // MITSUBA CAN 進角

  // CAN SOLARLOGGER
  double SOLARLOGGERVoltage[12];  // MPPT入力電圧1-12ch
  double SOLARLOGGERCurrent[12];  // MPPT入力電流1-12ch
  double SOLARLOGGERPower[12];    // MPPT入力電力1-12ch（電圧×電流の演算）
  double SOLARLOGGERTemp[16];     // アレイ温度16ch

  // CAN BMS
  double sumBatteryVoltage;   // BMS合計バッテリセル電圧
  double minCellVoltage;      // 最小セル電圧
  double maxCellVoltage;      // 最大セル電圧
  // double cellVoltage[28];     // 全セル電圧
};

bool secTimer = false;
struct LogData logdata = {0};
time_t startGPSTime = 0;
unsigned int storageCount = 0;
// unsigned int pulseCount = 0;     // 車速パルスカウント変数
bool BMSConnectFlag = false;        // BMS通信成功フラグ
bool BMSErrorFlag = false;                // BMSエラーフラグ
bool BMSErrorFlagOverDischarge = false;   // 過放電フラグ
bool BMSErrorFlagOverCharge = false;  // 過充電フラグ
bool BMSErrorFlagOverTemp = false;    // 過温度フラグ
bool BMSErrorFlagLowTemp = false;     // 過低温フラグ

/*プロトタイプ宣言*/
void print(const char *str, int row, int col);
int SetupINA226(void);
void SetupTime(void);
int SetupUSB(struct LogData *data, char name[][FILE_NAME_MAX]);
void MainFuncTimer(void);
int ReadBackup(struct LogData *data);
int FetchGPSdata(const char name[][FILE_NAME_MAX]);
void CalculateTimeSet(struct LogData *data);
void CalculateSet_CAN(struct LogData *data);
void CalculateSet(struct LogData *data, bool run);
void LCD(const struct LogData *data);
void XbeeSerial(const struct LogData *data);
int USBSaveData(const struct LogData *data, const char name[][FILE_NAME_MAX]);
int USBSaveCANData(const struct LogData *data, const char name[][FILE_NAME_MAX]);
int SaveBackup(const struct LogData *data);
//void MotorPulseCount(void);  //車速パルスカウント関数
void SendCANMessage(void);
void ReceiveCANMessage(void);  // CAN受信割り込みハンドラ
void DisplayBMSError(void);
double MoD0(double vaLue);
double MoD1(double vaLue);
double MoD2(double vaLue);
double MoD3(double vaLue);

int main(){
  char fileName[5][FILE_NAME_MAX] = {0};   // USBファイル名  [0]:LOG, [1]:GPS,[2]CAN_MITSUBA, [3]:CAN_SOLAR, [4]:CAN_BMS

  debug.baud(9600);
  Xbee.baud(57600);  // 安心と信頼()の57600bps
  myGPS.begin(9600);
  myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  myGPS.sendCommand(PGCMD_ANTENNA);

  lcd.cls();

  if(SetupINA226()) {
    NVIC_SystemReset();
    return -1;
  }

  // GPS補足するまでループする
  SetupTime();
  startGPSTime = time(NULL) + OFFSET_TIME;
  strftime(logdata.timeStr[1], TIME_MAX, "%T", localtime(&startGPSTime));
  lcd.locate(0, 2);
  lcd.printf("Time %s", logdata.timeStr[1]);

  if(SetupUSB(&logdata, fileName)) {
    NVIC_SystemReset();
    return -1;
  }

  ReadBackup(&logdata);
  wait(0.5);

  //motorPulse.fall(&MotorPulseCount);  //ピン立ち下がり割り込みで、パルスカウント関数の実行
  //パルスで速度を見ることを考えていないなら問題ない↑

  lcd.cls();
  canBusSystem.frequency(250000);
  canBusSystem.attach(&ReceiveCANMessage, CAN::RxIrq);  // CAN受信割り込みの実行
  RTC::attach(&MainFuncTimer, RTC::Second);             // MainFuncTimerを1秒ごとに実行

  // メインループ
  while(1) {
    FetchGPSdata(fileName);
    // 1秒ごとに実行
    if(secTimer) {
      secTimer = false;
      if(USBSaveData(&logdata, fileName) == -1) {
        print("USB  ERROR!", 5, 1);
        wait(0.5);
        break;
      }
      if(USBSaveCANData(&logdata, fileName) == -1) {
        print("USB  ERROR!", 5, 1);
        wait(0.5);
        break;
      }
      if(SaveBackup(&logdata) == -1) {
        print("USB  ERROR!", 5, 1);
        wait(0.5);
        break;
      }
    }
    // STORAGE_TIMEごとに実行
    if(storageCount >= STORAGE_TIME) {
      storageCount = 0;
      XbeeSerial(&logdata);
    }
    // BMS警告時
    if(BMSErrorFlag){
      DisplayBMSError();
    }
  }
  NVIC_SystemReset();
}

void MainFuncTimer() {
  secTimer = true;
  SendCANMessage();
  CalculateTimeSet(&logdata);
  CalculateSet(&logdata, true);

  // BMS警告でないなら通常ディスプレイ表示
  if(!BMSErrorFlag){
    LCD(&logdata);
  }
  
  storageCount++;
}

/** 全てのストリーム系に文字列を出力
 * @param const char *str 送信文字列
 * @param int row 列
 * @param int col 行
 */
void print(const char *str, int row, int col){
  debug.printf("%s", str);
  Xbee.printf("%s", str);
  lcd.locate(row, col);
  lcd.printf("%s", str);
}

/** INA226 レジスタ書き込み等
 * @return 0:成功     -1:失敗
 */
int SetupINA226(){
  unsigned short val_1 = 0;
  unsigned short val_2 = 0;
  unsigned short val_3 = 0;
  
  // INA226の存在を確認
  if(!BatteryMonitor.isExist()) {
    print("INA226B not found!\n", 0, 0);
    wait(0.5);
  }
  if(!PanelMonitor.isExist()) {
    print("INA226P not found!\n", 0, 1);
     wait(0.5);
  }
  if(!MotorMonitor.isExist()) {
    print("INA226M not found!\n", 0, 2);
     wait(0.5);
    return -1;
  }
 if(!BatteryMonitor2.isExist()) {
    print("INA226B not found!\n", 0, 0);
    wait(0.5);
  }
  // INA226のレジスタの値を正しく読めるか確認
  if(BatteryMonitor.rawRead(0x00, &val_1) != 0) {
    print("INA226B read Error!\n", 0, 0);
    wait(0.5);
    return -1;
  }
  if(PanelMonitor.rawRead(0x00, &val_2) != 0) {
    print("INA226P read Error!\n", 0, 1);
    wait(0.5);
    return -1;
  }
  if(MotorMonitor.rawRead(0x00, &val_3) != 0) {
    print("INA226M read Error!\n", 0, 2);
    wait(0.5);
    return -1;
  }
  if(BatteryMonitor2.rawRead(0x00, &val_1) != 0) {
    print("INA226B2 read Error!\n", 0, 0);
    wait(0.5);
    return -1;
}
  // 各INA226にレジスタ値を書き込む
  BatteryMonitor.setConfiguration(INA226_CONFIG);
  PanelMonitor.setConfiguration(INA226_CONFIG);
  MotorMonitor.setConfiguration(INA226_CONFIG);
  BatteryMonitor2.setConfiguration(INA226_CONFIG);
  
  BatteryMonitor.setCurrentCalibration(INA226_CALIB);
  PanelMonitor.setCurrentCalibration(INA226_CALIB_PANEL);
  MotorMonitor.setCurrentCalibration(INA226_CALIB);
  BatteryMonitor2.setCurrentCalibration(INA226_CALIB);

  print("INA226 OK!\n", 0, 0);
  return 0;
}

void SetupTime(){
  int count = 0;
  // print("GPS fixing now...\n", 0, 1);

  // char c;
  do {
    // c = myGPS.read();
    myGPS.read();
    // if (c) debug.printf("%c", c);
    if(myGPS.newNMEAreceived()) {
      if(!myGPS.parse(myGPS.lastNMEA())) {
        count++;
        debug.printf("%d\n", count);
        lcd.locate(0, 1);
        lcd.printf("GPS calibrating :%d", count);
        continue;
      }
    }
  } while(((!myGPS.fix) && (count < 500)));

  struct tm t;
  t.tm_sec = myGPS.seconds;     // 0-59
  t.tm_min = myGPS.minute;      // 0-59
  t.tm_hour = myGPS.hour;     // 0-23
  t.tm_mday = myGPS.day;     // 1-31
  t.tm_mon = myGPS.month - 1;       // 0-11
  t.tm_year = myGPS.year + 100;    // year since 1900

  time_t seconds = mktime(&t);
  set_time(seconds);
}

/** USBメモリ書き込み初期化 ファイル名設定
 * @param struct LogData LogData 構造体
 * @param char name ファイル名が書き込まれる配列
 *
 * @return 0: 成功 -1: 失敗
 */
int SetupUSB(struct LogData *data, char name[][FILE_NAME_MAX]){
  time_t createdTime = time(NULL) + OFFSET_TIME;
  // 日付がファイル名
  strftime(name[0], FILE_NAME_MAX, "/usb/%y%m%d_log.csv", localtime(&createdTime));
  strftime(name[1], FILE_NAME_MAX, "/usb/%y%m%d_GPSNMEA.log", localtime(&createdTime));
  strftime(name[2], FILE_NAME_MAX, "/usb/%y%m%d_MITSUBA_CAN.csv", localtime(&createdTime));
  // strftime(name[3], FILE_NAME_MAX, "/usb/%y%m%d_SOLAR_CAN.csv", localtime(&createdTime));
  strftime(name[4], FILE_NAME_MAX, "/usb/%y%m%d_BMS_CAN.csv", localtime(&createdTime));

  FILE *fp = fopen(name[0], "a");
  if(fp == NULL) {
    print("USB  ERORR\n", 0, 3);
    return -1;
  }
  else{
    strftime(data->timeStr[0], TIME_MAX, "%Y%m%d%H%M%S", localtime(&createdTime));
    fprintf(fp, "%s\n", data->timeStr[0]);    // 試しに現在時刻を書き込み
    fprintf(fp, "%s\n", USB_HEADER_STRING);   // ヘッダー書き込み
  }
  fclose(fp);

  fp = fopen(name[2], "a");
  if(fp == NULL) {
    print("USB  ERORR\n", 0, 3);
    wait(0.5);
    return -1;
  }
  else{
    strftime(data->timeStr[0], TIME_MAX, "%Y%m%d%H%M%S", localtime(&createdTime));
    fprintf(fp, "%s\n", data->timeStr[0]);    // 試しに現在時刻を書き込み
    fprintf(fp, "%s\n", HEADER_CAN_MITSUBA);   // ヘッダー書き込み
  }
  fclose(fp);

  // fp = fopen(name[3], "a");
  // if(fp == NULL) {
  //   print("USB  ERORR\n", 0, 3);
  //    wait(0.5);
  //   return -1;
  // }
  // else{
  //   strftime(data->timeStr[0], TIME_MAX, "%Y%m%d%H%M%S", localtime(&createdTime));
  //   fprintf(fp, "%s\n", data->timeStr[0]);    // 試しに現在時刻を書き込み
  //   fprintf(fp, "%s\n", HEADER_CAN_SOLAR);   // ヘッダー書き込み
  // }
  // fclose(fp);

  fp = fopen(name[4], "a");
  if(fp == NULL) {
    print("USB  ERORR\n", 0, 3);
     wait(0.5);
    return -1;
  }
  else{
    strftime(data->timeStr[0], TIME_MAX, "%Y%m%d%H%M%S", localtime(&createdTime));
    fprintf(fp, "%s\n", data->timeStr[0]);    // 試しに現在時刻を書き込み
    fprintf(fp, "%s\n", HEADER_CAN_BMS);   // ヘッダー書き込み
  }
  fclose(fp);

  print("USB OK!\n", 0, 3);
  
  return 0;
}

/** バックアップデータを読み込み，LogData構造体に格納
 * @param struct LogData バックアップからLogData 構造体に書き込む
 *
 * @return 0:成功     1:バックアップデータが存在していない
 */
int ReadBackup(struct LogData *data){
  struct tm *t_now;
  struct tm *t_backup;
  time_t backupTime;
  time_t nowTime = time(NULL) + OFFSET_TIME;
  t_now = localtime(&nowTime);

  FILE *fp_back = fopen("/usb/backup.csv", "r");

  if(fp_back == NULL) {
    debug.printf("Backup File No Exist\n");
    return 1;  // 初回起動時 backup.csvはSaveBackup関数が実行された時に作成される
  }
  else{
    fscanf(fp_back, "%d", &backupTime);
    t_backup = localtime(&backupTime);
    if(t_now->tm_mday == t_backup->tm_mday) {  // 同一日付のみ
      fscanf(fp_back, "%d,%d", &(data->totalTime), &startGPSTime);
    }
    for(int i = 0; i < RealtimeData_i; i++) {
      fscanf(fp_back, "%lf,", &(data->RealtimeData[i][3]));  // 積算（積算電流の項目消しちゃった）
    }
    fscanf(fp_back, "%lf", &(data->distanceMotorSpeed));
  }
  fclose(fp_back);
  return 0;
}

/** シリアルGPSデータを解析，保存
 * @param const char name ファイル名が保存されている配列
 *
 * @return 0:成功    -1:失敗
 */
int FetchGPSdata(const char name[][FILE_NAME_MAX]){
  // char c;
  while(1) {
    myGPS.read();                            // ただ読むだけでなくて，GPSセンテンスを文字列化している
    // if (c) debug.printf("%c", c);
    if (myGPS.newNMEAreceived()) {           // センテンスが一文取得できたかどうか
      if (!myGPS.parse(myGPS.lastNMEA())) {  // 解析が完了したかどうか していない場合はもう一度ループ(breakスキップ)
        continue;
      }
      break;                                 // 解析終了した場合はループを抜ける
    }
  }
  // 直前のGPSセンテンスをUSBに保存
  FILE *fp = fopen(name[1], "a");
  if(fp == NULL) {
    lcd.cls();
    print("USB  ERORR!\n", 5, 1);
    wait(0.5);
    return -1;
  }
  else{
    fprintf(fp, "%s", myGPS.lastNMEA());
  }
  fclose(fp);
  return 0;
}

/** 時刻関連のデータを計算，格納
 * @param struct LogData
 */
void CalculateTimeSet(struct LogData *data) {
  time_t nowTime;        // 現在時刻
  time_t totalTime_tmp;  // 経過時間
  int hour = 0;
  int minute = 0;
  int second = 0;

  nowTime = time(NULL) + OFFSET_TIME;

  totalTime_tmp = nowTime - startGPSTime;
  data->totalTime = (unsigned int) totalTime_tmp;

  strftime(data->timeStr[0], TIME_MAX, "%Y%m%d%H%M%S", localtime(&nowTime));   // PHP,USB用
  strftime(data->timeStr[1], TIME_MAX, "%T", localtime(&nowTime));             // LCDの現在時刻用
  snprintf(data->timeStr[2], TIME_MAX, "%u", (unsigned int) nowTime);

  sscanf(data->timeStr[1], "%d:%d:%d", &hour, &minute, &second);
  data->totalTimeNoon = (hour * 3600 + minute * 60 + second) - 3600 * 12;  // 正午を基準とした経過時間
}

/** 1秒毎に電圧・電流測定，計算後のデータを構造体に格納
 * @param struct LogData
 * @param  bool run : true 積算する false 積算しない
 */
void CalculateSet(struct LogData *data, bool run){
  bool retry = false;
  int count = 0;

  double voltage_tmp = 0;      // 仮に電圧,電流データを保存しておく
  double current_tmp[RealtimeData_i] = {0};
  double vAl[4] = {0};

  // 異常な値の場合は，再度データを取り直す
  do {
    retry = false;
    
    BatteryMonitor2.getVoltage(&vAl[0]);     // バッテリー電圧
    BatteryMonitor.getVoltage(&vAl[1]); // バッテリー電流
    PanelMonitor.getVoltage(&vAl[2]);   // パネル電流
    MotorMonitor.getVoltage(&vAl[3]);   // モーター電流
    
    current_tmp[0] = MoD1(vAl[1]);    // バッテリー電流変換後の値をcurrent_tmp[0]に代入fx0かかないと
    current_tmp[1] = MoD2(vAl[2]);    //同上
    current_tmp[2] = MoD3(vAl[3]);    //同上
    voltage_tmp = MoD0(vAl[0]);       //同上
    
   //data->motorSpeed_pulse = pulseCount * pulseConvert;     // 1秒間でカウントしたパルスを速度に変換
   // pulseCount = 0;//パルスのリセット
   //↑パルスは使わないとしている

    if(voltage_tmp > 150) retry = true;         // 電圧が150V以上は異常
    for(int i = 0; i < RealtimeData_i; i++) {
      if(current_tmp[i] > 60) retry = true;    // 電流が60A以上は異常
    }
    count++;
  } while(retry && count < 5);  // 5回とっても異常な場合はさすがにループを抜ける

  // 0割の有無を確認した後，配列に格納する
  if(data->totalTime != 0) {
    data->batteryVoltage = voltage_tmp; // 分圧電圧を補正
    for(int i = 0; i < RealtimeData_i; i++) {
      data->RealtimeData[i][0] = current_tmp[i];  // 電流
      data->RealtimeData[i][1] = data->batteryVoltage * data->RealtimeData[i][0];   // 電力
      if(run) {
        //data->RealtimeData[i][2] += data->RealtimeData[i][0] / 3600;            // 積算電流
        data->RealtimeData[i][3] += data->RealtimeData[i][1] / 3600;            // 積算電力
       // data->RealtimeData[i][4] = data->RealtimeData[i][2] * 3600 / data->totalTime; // 平均電流
        //data->RealtimeData[i][5] = data->RealtimeData[i][3] * 3600 / data->totalTime; // 平均電力
      }
    }
    data->remainBatteryWh = BATTERY_CAPACITY - data->RealtimeData[0][3];     // バッテリー残量[Wh]
    // data->remainBatteryParcent = (data->remainBatteryWh / BATTERY_CAPACITY) * 100;    // バッテリー残量[%]
    data->gpsSpeed = myGPS.speed * NOTS_CONVERT;  // GPS速度
    data->gpsAngle = myGPS.angle;                 //進行方向
    data->latitude = myGPS.latitude;              // GPS緯度
    data->longitude = myGPS.longitude;            // GPS経度

    data->motorSpeed = data->motorRPM * RPM_CONVERT;  //モータ速度
    data->distanceMotorSpeed += data->motorRPM * RPM_CONVERT_MS / 1000;  //走行距離

    // モーターコイル温度センサ（きっとLM35） 値取得(移動平均処理)
    data->motorTemp = 0;
    const int TEMP_AVE_CNT = 10;
    double motorTemp_tmp[TEMP_AVE_CNT] = {0};

    for(int i = 0; i < TEMP_AVE_CNT; i++) {
      count = 0;
      do {
        motorTemp_tmp[i] = motorTempAnalogIn.read() * 3.3 * 100;
        count++;
      } while(motorTemp_tmp[i] > 250 && count < 5);
      // debug.printf("tempData %d: %.2f\n", i, motorTemp_tmp[i]);
      data->motorTemp += motorTemp_tmp[i] / TEMP_AVE_CNT;  // 10回平均
    }
    // debug.printf("\ntemp %d: %.2f\n", i, data->motorTemp);
  }
}

/*
 * データ送信サンプル
 *
 */
void XbeeSerial(const struct LogData *data){
  char SerialSendStr[SERIAL_MAX] = {0};

  snprintf(SerialSendStr, SERIAL_MAX, "%s,%d,%.2f", data->timeStr[1], data->totalTimeNoon, data->batteryVoltage);

  // 電流
  for(int i = 0; i < RealtimeData_i; i++) {
    snprintf(SerialSendStr, SERIAL_MAX, "%s,%.2f", SerialSendStr, data->RealtimeData[i][0]);
  }

  // 電力
  for(int i = 0; i < RealtimeData_i; i++) {
    snprintf(SerialSendStr, SERIAL_MAX, "%s,%.2f", SerialSendStr, data->RealtimeData[i][1]);
  }

  // 積算電力
  for(int i = 0; i < RealtimeData_i; i++) {
    snprintf(SerialSendStr, SERIAL_MAX, "%s,%.2f", SerialSendStr, data->RealtimeData[i][3]);
  }

  snprintf(SerialSendStr, SERIAL_MAX, "%s,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f", SerialSendStr,
           data->motorSpeed,
           data->motorTemp,
           data->remainBatteryWh,
           data->sumBatteryVoltage,
           data->minCellVoltage,
           data->maxCellVoltage);

  // 平均電力
  /* for(int i = 0; i < RealtimeData_i; i++) {
    snprintf(SerialSendStr, SERIAL_MAX, "%s,%.2f", SerialSendStr,  data->RealtimeData[i][5]);
  }*/

  snprintf(SerialSendStr, SERIAL_MAX, "%s,%.1f,%.1f", SerialSendStr,
            data->distanceMotorSpeed,
            data->gpsSpeed);//「gpsSpeed」→「motorSpeed_pulse」→「gpsSpeed」パルスは使わない方針で考える

  Xbee.printf("%s\r\n", SerialSendStr);  // 改行コード変更 \n → \r\n
}

/*LCDへのデータ表示を行う locate関数は0行0列からはじまり，引数は(列，行)*/
void LCD(const struct LogData *data){
  lcd.locate(0, 0);
  lcd.printf("%s", data->timeStr[1]);
  lcd.locate(11, 0);
  lcd.printf("%3.0f", data->motorSpeed);  //「motorSpeed」→「motorSpeed_pulse」→「motorSpeed」パルスは使わない方針で考える
  lcd.locate(15, 0);
  lcd.printf("km/h");

  lcd.locate(0, 1);
  lcd.printf("%6.1f", data->remainBatteryWh);  //％表記からWhへ、Whでみんな分かるはず
  lcd.locate(6, 1);
  lcd.printf("Wh");
  lcd.locate(10, 1);
  lcd.printf("B");
  lcd.locate(12, 1);
  lcd.printf("%5.2f", data->RealtimeData[0][0]);
  lcd.locate(18, 1);
  lcd.printf("A");

  lcd.locate(1, 2);
  lcd.printf("%5.1f", data->batteryVoltage);
  lcd.locate(7, 2);
  lcd.printf("V");
  lcd.locate(10, 2);
  lcd.printf("P");
  lcd.locate(12, 2);
  lcd.printf("%5.2f", data->RealtimeData[1][0]);
  lcd.locate(18, 2);
  lcd.printf("A");

  lcd.locate(0, 3);
  lcd.printf("%6.1f", data->distanceMotorSpeed);
  lcd.locate(6, 3);
  lcd.printf("km");
  lcd.locate(10, 3);
  lcd.printf("M");
  lcd.locate(12, 3);
  lcd.printf("%5.2f", data->RealtimeData[2][0]);
  lcd.locate(18, 3);
  lcd.printf("A");

  // BMS 通信成功インジケータ
  if(BMSConnectFlag){
    BMSConnectFlag = false;
    lcd.locate(19, 3);
    lcd.printf("*");  // 「*」を表示
  }
  else{
    lcd.locate(19, 3);
    lcd.printf(" ");  // 「*」消去
  }
}

void DisplayBMSError(){
  if(BMSErrorFlagOverCharge){
    BMSErrorFlagOverCharge = false;
    lcd.cls();
    lcd.locate(3, 1);
    lcd.printf("BATTERY");
    lcd.locate(11, 1);
    lcd.printf("ERORR!");
    lcd.locate(4, 2);
    lcd.printf("OVER");
    lcd.locate(10, 2);
    lcd.printf("CHARGE");
    wait(3.0);
    lcd.cls();
    BMSErrorFlag = false;
  }
  if(BMSErrorFlagOverDischarge){
    BMSErrorFlagOverCharge = false;
    lcd.cls();
    lcd.locate(3, 1);
    lcd.printf("BATTERY");
    lcd.locate(11, 1);
    lcd.printf("ERORR!");
    lcd.locate(3, 2);
    lcd.printf("OVER");
    lcd.locate(8, 2);
    lcd.printf("DISCHARGE");
    wait(3.0);
    lcd.cls();
    BMSErrorFlag = false;
  }
  if(BMSErrorFlagOverTemp){
    BMSErrorFlagOverTemp = false;
    lcd.cls();
    lcd.locate(3, 1);
    lcd.printf("BATTERY");
    lcd.locate(11, 1);
    lcd.printf("ERORR!");
    lcd.locate(4, 2);
    lcd.printf("OVER");
    lcd.locate(11, 2);
    lcd.printf("TEMP");
    wait(3.0);
    lcd.cls();
    BMSErrorFlag = false;
  }
  if(BMSErrorFlagLowTemp){
    BMSErrorFlagLowTemp = false;
    lcd.cls();
    lcd.locate(3, 1);
    lcd.printf("BATTERY");
    lcd.locate(11, 1);
    lcd.printf("ERORR!");
    lcd.locate(4, 2);
    lcd.printf("LOW");
    lcd.locate(11, 2);
    lcd.printf("TEMP");
    wait(3.0);
    lcd.cls();
    BMSErrorFlag = false;
  }
}

/*構造体のデータをUSBメモリへデータを保存(csvファイル)*/
int USBSaveData(const struct LogData *data, const char name[][FILE_NAME_MAX]){
  FILE *fp = fopen(name[0], "a");

  if(fp == NULL) {
    fclose(fp);
    return -1;
  }
  else{
    fprintf(fp, "%s,%d,%.2f", data->timeStr[1], data->totalTimeNoon, data->batteryVoltage);
    for(int j = 0; j < RealtimeData_j; j++) {
      for(int i = 0; i < RealtimeData_i; i++) {
        fprintf(fp, ",%.2f", data->RealtimeData[i][j]);  // 全データ
      }
    }
    fprintf(fp, ",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f",
            //data->remainBatteryParcent,
            data->remainBatteryWh,
            data->motorSpeed,
            data->gpsSpeed,
            data->gpsAngle,
            data->distanceMotorSpeed,
            data->motorTemp
            //data->motorSpeed_pulse
            );
    fprintf(fp, "\n");
    fclose(fp);
  }
  return 0;
}

int USBSaveCANData(const struct LogData *data, const char name[][FILE_NAME_MAX]){

  FILE *fp = fopen(name[2], "a");
  if(fp == NULL){
    fclose(fp);
    return -1;
  }
  else{
    fprintf(fp, "%s,%d", data->timeStr[1], data->totalTimeNoon);
    fprintf(fp, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n", data->motorVoltage, data->motorCurrent, data->tempFET, data->motorRPM, data->duty, data->angle);
    fclose(fp);
  }

  // SOLARLOGGER
  // fp = fopen(name[3], "a");
  // if(fp == NULL){
  //   fclose(fp);
  //   return -1;
  // }
  // else{
  //   fprintf(fp, "%s,%d", data->timeStr[1], data->totalTimeNoon);
  //   for(int i = 0; i < 12; i++){
  //     fprintf(fp, ",%.2f", data->SOLARLOGGERVoltage[i]);
  //   }
  //   for(int i = 0; i < 12; i++){
  //     fprintf(fp, ",%.2f", data->SOLARLOGGERCurrent[i]);
  //   }
  //   for(int i = 0; i < 12; i++){
  //     fprintf(fp, ",%.2f", data->SOLARLOGGERPower[i]);
  //   }
  //   for(int i = 0; i < 16; i++){
  //     fprintf(fp, ",%.1f", data->SOLARLOGGERTemp[i]);
  //   }
  //   fprintf(fp, "\n");
  //   fclose(fp);
  // }

  // BMS
  fp = fopen(name[4], "a");
  if(fp == NULL){
    fclose(fp);
    return -1;
  }
  else{
    fprintf(fp, "%s,%d", data->timeStr[1], data->totalTimeNoon);
    fprintf(fp, ",%.4f,%.4f,%.4f", data->sumBatteryVoltage, data->minCellVoltage, data->maxCellVoltage);
    // for(int i = 0; i < 28; i++){
    //   fprintf(fp, ",%.4f", data->cellVoltage[i]);
    // }
    fprintf(fp, "\n");
    fclose(fp);
  }

  return 0;
}

/** 蓄積されているデータをcsvファイルにバックアップ
 * @param const strct LogData
 * @param const double predata
 * @param time_t preTime
 */
int SaveBackup(const struct LogData *data){

  FILE *fp_back = fopen("/usb/backup.csv", "w");
  if(fp_back == NULL) {
    fclose(fp_back);
    return -1;
  }
  else{
    fprintf(fp_back, "%s\r\n", data->timeStr[2]);  // UnixTime
    fprintf(fp_back, "%d,%d\r\n", data->totalTime, (unsigned int)startGPSTime);  //  時間
    for(int i = 0; i < RealtimeData_i; i++) {
      //for(int j = 0; j < 2; j++){
      fprintf(fp_back, "%f,", data->RealtimeData[i][3]);  // 積算電力
      //}
      fprintf(fp_back, "\r\n");
    }
    fprintf(fp_back, "%f\r\n", data->distanceMotorSpeed);  // 走行距離
    fclose(fp_back);
  }
  return 0;
}

/* void MotorPulseCount() {
    pulseCount++;    // ストリーム等でデバックすると遅延で正確にパルス数を測定できない
}*/
//パルスは使わない方針で考えるのでバン

void SendCANMessage(){
  const uint32_t REQUEST_ID_LIST = 0x08F89540;
  const uint8_t REQUEST_DATA_LIST = 0x01;

  CANMessage sendMsg;
  sendMsg.id = REQUEST_ID_LIST;
  sendMsg.data[0] = REQUEST_DATA_LIST;
  sendMsg.len = 8;
  sendMsg.type = CANData;
  sendMsg.format = CANExtended;
  
  canBusSystem.write(sendMsg);
  // debug.printf("send_id:%x, send_data:%x\n", sendMsg.id, sendMsg.data[1]);
}

void ReceiveCANMessage(){
  CalculateSet_CAN(&logdata);
}

void CalculateSet_CAN(struct LogData *data){
  // const double SOL_VOLTAGE_LSB = 0.01953125;
  // const double SOL_CURRENT_LSB = 0.0048828125;
  // const double SOL_TEMP_LSB = 0.48828125;
  const double BMS_VOLTAGE_LSB = 0.0001;  // BMS セル電圧の1LSB

  const uint32_t RECEIVE_ID_MITSUBA[] = {0x08850225, 0x08950225, 0x08A50225};  // FRAME0, FRAME1, FRAME2
  // const uint32_t RECEIVE_ID_SOLAR_1[] = {0x10, 0x20, 0x30};
  // const uint32_t RECEIVE_ID_SOLAR_2[] = {0x11, 0x21, 0x31};
  const uint32_t RECEIVE_ID_BMS[] = {0x40, 0x41, 0x42, 0x43, 0x44, 0x45, 0x46, 0x47};

  const uint32_t RECEIVE_ID_BMS_ERROER = 0x00;
  const uint8_t FLAG_OVER_DISCHARGE = 0x01; // 0001
  const uint8_t FLAG_OVER_CHARGE = 0x02;    // 0010
  const uint8_t FLAG_OVER_TEMP = 0x04;      // 0100
  const uint8_t FLAG_LOW_TEMP = 0x08;       // 1000

  double motorVoltage_temp = 0;

  CANMessage msg;
  if(canBusSystem.read(msg)){
    // debug.printf("\nreceive_id:%x receive_data:%x\n", msg.id, msg.data[0]);
    // MITSUBA FRAME0
    if(msg.id == RECEIVE_ID_MITSUBA[0]){
      motorVoltage_temp = (double)((((((uint16_t)msg.data[1]) & 0x0003) << 8) | (((uint16_t)msg.data[0]) & 0x00ff))) * 0.5;
      data->motorVoltage = motorVoltage_temp * MITSUBA_VOLTAGE_CALIB;
      data->motorCurrent = (double)(((((uint16_t)msg.data[2]) & 0x0007) << 6) | ((((uint16_t)msg.data[1]) & 0x00fc) >> 2));
      data->tempFET = (double)(((((uint16_t)msg.data[4]) & 0x0007) << 2) | ((((uint16_t)msg.data[3]) & 0x00c0) >> 6)) * 5;
      data->motorRPM = (double)(((((uint16_t)msg.data[5]) & 0x007f) << 5) | ((((uint16_t)msg.data[4]) & 0x00f8) >> 3));
      data->duty = (double)(((((uint16_t)msg.data[5]) & 0x0080) >> 7) | ((((uint16_t)msg.data[6]) & 0x00ff) << 1) | ((((uint16_t)msg.data[7]) & 0x0001) << 9)) * 0.5;
      data->angle = (double)((((uint16_t)msg.data[7]) & 0x00fe) >> 1) * 0.5;
    }
    
    /*
    // SOLARLOGGER 1 Voltage
    if(msg.id == RECEIVE_ID_SOLAR_1[0]){
      data->SOLARLOGGERVoltage[0] = (double)(((uint16_t)msg.data[0]) | ((uint16_t)msg.data[1] & 0x03) << 8) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[1] = (double)(((uint16_t)msg.data[1] & 0xFC) >> 2 | ((uint16_t)msg.data[2] & 0x0F) << 6) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[2] = (double)(((uint16_t)msg.data[2] & 0xF0) >> 4 | ((uint16_t)msg.data[3] & 0x3F) << 4) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[3] = (double)(((uint16_t)msg.data[3] & 0xC0) >> 6 | ((uint16_t)msg.data[4]) << 2) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[4] = (double)(((uint16_t)msg.data[5]) | ((uint16_t)msg.data[6] & 0x03) << 8) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[5] = (double)(((uint16_t)msg.data[6] & 0xFC) >> 2 | ((uint16_t)msg.data[7] & 0x0F) << 6) * SOL_VOLTAGE_LSB;
    }

    // SOLARLOGGER 2 Voltage
    if(msg.id == RECEIVE_ID_SOLAR_2[0]){
      data->SOLARLOGGERVoltage[6] = (double)(((uint16_t)msg.data[0]) | ((uint16_t)msg.data[1] & 0x03) << 8) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[7] = (double)(((uint16_t)msg.data[1] & 0xFC) >> 2 | ((uint16_t)msg.data[2] & 0x0F) << 6) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[8] = (double)(((uint16_t)msg.data[2] & 0xF0) >> 4 | ((uint16_t)msg.data[3] & 0x3F) << 4) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[9] = (double)(((uint16_t)msg.data[3] & 0xC0) >> 6 | ((uint16_t)msg.data[4]) << 2) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[10] = (double)(((uint16_t)msg.data[5]) | ((uint16_t)msg.data[6] & 0x03) << 8) * SOL_VOLTAGE_LSB;
      data->SOLARLOGGERVoltage[11] = (double)(((uint16_t)msg.data[6] & 0xFC) >> 2 | ((uint16_t)msg.data[7] & 0x0F) << 6) * SOL_VOLTAGE_LSB;
    }

    // SOLARLOGGER 1 Current
    if(msg.id == RECEIVE_ID_SOLAR_1[1]){
      data->SOLARLOGGERCurrent[0] = (double)(((uint16_t)msg.data[0]) | ((uint16_t)msg.data[1] & 0x03) << 8) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[1] = (double)(((uint16_t)msg.data[1] & 0xFC) >> 2 | ((uint16_t)msg.data[2] & 0x0F) << 6) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[2] = (double)(((uint16_t)msg.data[2] & 0xF0) >> 4 | ((uint16_t)msg.data[3] & 0x3F) << 4) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[3] = (double)(((uint16_t)msg.data[3] & 0xC0) >> 6 | ((uint16_t)msg.data[4]) << 2) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[4] = (double)(((uint16_t)msg.data[5]) | ((uint16_t)msg.data[6] & 0x03) << 8) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[5] = (double)(((uint16_t)msg.data[6] & 0xFC) >> 2 | ((uint16_t)msg.data[7] & 0x0F) << 6) * SOL_CURRENT_LSB;
    }

    // SOLARLOGGER 2 Current
    if(msg.id == RECEIVE_ID_SOLAR_2[1]){
      data->SOLARLOGGERCurrent[6] = (double)(((uint16_t)msg.data[0]) | ((uint16_t)msg.data[1] & 0x03) << 8) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[7] = (double)(((uint16_t)msg.data[1] & 0xFC) >> 2 | ((uint16_t)msg.data[2] & 0x0F) << 6) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[8] = (double)(((uint16_t)msg.data[2] & 0xF0) >> 4 | ((uint16_t)msg.data[3] & 0x3F) << 4) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[9] = (double)(((uint16_t)msg.data[3] & 0xC0) >> 6 | ((uint16_t)msg.data[4]) << 2) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[10] = (double)(((uint16_t)msg.data[5]) | ((uint16_t)msg.data[6] & 0x03) << 8) * SOL_CURRENT_LSB;
      data->SOLARLOGGERCurrent[11] = (double)(((uint16_t)msg.data[6] & 0xFC) >> 2 | ((uint16_t)msg.data[7] & 0x0F) << 6) * SOL_CURRENT_LSB;
    }

    // SOLARLOGGER POWER
    for(int i = 0; i < 12; i++){
      data->SOLARLOGGERPower[i] = data->SOLARLOGGERVoltage[i] * data->SOLARLOGGERCurrent[i];
    }

    // SOLARLOGGER 1 Temp
    if(msg.id == RECEIVE_ID_SOLAR_1[2]){
      for (int i = 0; i < 8; i++){
        data->SOLARLOGGERTemp[i] = (double)(msg.data[i] * SOL_TEMP_LSB);
      }
    }

    // SOLARLOGGER 2 Temp
    if(msg.id == RECEIVE_ID_SOLAR_2[2]){
      for (int i = 8; i < 16; i++){
        data->SOLARLOGGERTemp[i] = (double)(msg.data[i] * SOL_TEMP_LSB);
      }
    }
    */

    // BMS 最小・最大・合計バッテリセル電圧
    if(msg.id == RECEIVE_ID_BMS[0]){
      BMSConnectFlag = true;
      data->minCellVoltage = (double)(((uint16_t)msg.data[0]) | ((uint16_t)msg.data[1]) << 8) * BMS_VOLTAGE_LSB;
      data->maxCellVoltage = (double)(((uint16_t)msg.data[2]) | ((uint16_t)msg.data[3]) << 8) * BMS_VOLTAGE_LSB;
      data->sumBatteryVoltage = (double)(((uint16_t)msg.data[4]) | ((uint16_t)msg.data[5] << 8) | ((uint16_t)msg.data[6] << 16)) * BMS_VOLTAGE_LSB;
    }

    // // BMS 各セル電圧
    // for(int i = 1; i < 8; i++){
    //   if(msg.id == RECEIVE_ID_BMS[i]){
    //     for(int j = 4 * (i - 1); j < 4 * i; j++){
    //       data->cellVoltage[j] = (double)(((uint16_t)msg.data[2 * (j % 4)]) | ((uint16_t)msg.data[2 * (j % 4) + 1] << 8)) * BMS_VOLTAGE_LSB;
    //     }
    //   }
    // }

    // Battery Error用
    if (msg.id == RECEIVE_ID_BMS_ERROER){
      BMSConnectFlag = true;  // BMS 通信成功フラグ
      if (msg.data[0] & FLAG_OVER_DISCHARGE){
        BMSErrorFlag = true;  // BMS エラー表示フラグ
        BMSErrorFlagOverDischarge = true;  // 過充電
      }
      if (msg.data[0] & FLAG_OVER_CHARGE){
        BMSErrorFlag = true; // BMS エラー表示フラグ
        BMSErrorFlagOverCharge = true;  // 過放電
      }
      if (msg.data[0] & FLAG_OVER_TEMP){
        BMSErrorFlag = true; // BMS エラー表示フラグ
        BMSErrorFlagOverTemp = true;  // 過温度
      }
      if (msg.data[0] & FLAG_LOW_TEMP){
        BMSErrorFlag = true; // BMS エラー表示フラグ
        BMSErrorFlagLowTemp = true;  // 過低温
      }
    }
  }
}

double MoD0(double vaLue)//バッテリー電圧
{
    
  return (vaLue*10);    
    
}     

double MoD1(double vaLue)//バッテリー電流
{
    
  return ((vaLue-2.510)*40);    
    
}     

double MoD2(double vaLue)//パネル電流
{
    
  return ((vaLue-2.507)*40);   ;    
    
}     

double MoD3(double vaLue)//モーター電流
{
    
  return ((vaLue-2.5025)*40);   ;    
    
}     
