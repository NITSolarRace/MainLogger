/*�X�V���
 * 2017BWSC�Ɏg�p�����v���O�������x�[�X�A�ȉ��ɂ��̕ύX����������

 2019/05/17
Z�̃v���O�������Q�l�Ƀp���X�ǉ�����ύX�ӏ�: 73�E112�E137�E144�E161�E198�E518-519�E692�E757-759�s��
�@�@##�^�C�����a��73�s�ڂ̌v�Z�Œ���
Xbee��gpsSpeed��motorSpeed_pulse�ɕύX608�s�ځiXbee :: gps���x���p���X���x)
lcd�\����motorSpeed��motorSpeed_pulse�ɕύX624�s�ځilcd :: can���x���p���X���x�j
 *
 * 2019/06/19
 * CAN�ʐM�Ή��ɔ����ύX
 * LogData�\���� dataAryMITSUBA, dataArySolarLog, bmsVoltage�@�폜
 *               MITSUBA, SOLARLOGGER, BMS�Ɋe�ϐ���ǉ�
 * CAN�ʐM 1�b�Ԋu��SendCANMessage�����s�A��M���荞�݂�ReceiveCANMessage�����s
 * USB�������ۑ��f�[�^�ǉ� MITSUBA, SOLARLOGGER, BMS�f�[�^ �t�@�C���ʕۑ�
 *2019/8/10
 *const char USB_HEADER_STRING[] ��BSM�̓d�́A�ώZ�d��Ah�A���ϓd���A���ϓd�́A�c�ʁ��A�p���X���x�̍������܂���
 *�t�@���Ȃ�ċ���ς��Ȃ��̂ō폜
 *�p���X���x�͎g��Ȃ����j�ōl���Ă��܂�
 */
 /*2019_9_15
 
 �ђʌ^�d���Z���T�[��INA�Q�Q�U�̃v���O�����C��
 �O���[�o���ϐ�vAl[]�ɕ␳�l�iLCD�ɕ\������l�j�����ꂽ��data->realtimeata[][]�ɂ͔��f����Ă��Ȃ��͗l
 LCD�X�V�̎��Ԃ��������ۂ������Ȃ�*
 */
/*
* 2019/09/21
* CAN�ʐM BMS �o�b�e���G���[�\���Ή�
* BMS����o�b�e���ُ���E���v�d���A�ŏ��d���A�ő�d���̃t���[���P�b�Ԋu�ő��M
* ��M���荞�݂ŏ��� DisplayBMSError()
* SOLARLOGGER�ۑ��p�v���O�����R�����g�A�E�g
*/

#include "mbed.h"
#include "INA226.hpp"
#include "TextOLED.h"
#include "MSCFileSystem.h"
#include "RTC.h"
#include "MBed_Adafruit_GPS.h"

// INA226�pI2C�A�h���X I2C�A�h���X��mbed�̏ꍇ1�r�b�g���V�t�g�����A�h���X�ɂȂ�
const int BATTERY_MONITOR_ADDRESS = 0x80;         // 0b10000000 (GG)
const int BATTERY_MON2_ADDRESS = 0x94;             // 0b10010100 (DD) 
const int PANEL_MONITOR_ADDRESS = 0x82;           // 0b10000010 (G1)
const int MOTOR_MONITOR_ADDRESS = 0x9E;           // 0b10011110 (CC)
const unsigned short INA226_CONFIG = 0x4897;    // ���ω�������^�C�~���O

/*------------------------------�ݒ�ύX����------------------------------*/
// const double VOLTAGE_CALIB = 10 * 1.0016;         // ������(10�{�ɕ␳)
const double MITSUBA_VOLTAGE_CALIB = 0.93396;     // �~�c�oCANlogger �d���␳ �덷0.7%���x��
const unsigned short INA226_CALIB = 0xF00;      // �V�����g�d���␳�W��
const unsigned short INA226_CALIB_PANEL = 0x3555; // �V�����g�d���␳�W���i�p�l���j
// const double currentCalibration[] = {2.00, 0.375, 2.00};
// [0]: �o�b�e�� [1]: �p�l�� [2]: ���[�^
/*------------------------------�ݒ�ύX����------------------------------*/
const double BATTERY_CAPACITY = 3.6 * 3.45 * 26 * 16;       // ���̓d��*�d���e��*����*����
// const double MOTOR_OVER_TEMP = 150;  // �R�C�����x150���ŉ߉��x

// const int OFFSET_TIME = 60 * 60 * 9;     // 9����(�W�����Ƃ̎���) = ���{����
const int OFFSET_TIME = 60 * 60 * 9.5;     // �_�[�E�B������
const int STORAGE_TIME = 1;             // 1�b�ԊuXbee���M

const double NOTS_CONVERT = 1.852;    // 1�m�b�g = 1.852 km/h
const double RPM_CONVERT = 0.104399;  // rpm -> km/h �ϊ��W��
const double RPM_CONVERT_MS = 0.028955;     // rpm -> m/s �ϊ��W��
//const double pulseConvert = 0.1303;     // �ԑ��p���X -> ���x�ϊ��W��  1[pulse/sec]��48[pulse/��]��]�~���a0.553[m]�~3.14159265�~3.6[sec�Ekm/m]

/*�e�z��v�f��*/
const int RealtimeData_i = 3;  // [0]: �o�b�e�� [1]: �p�l�� [2]: ���[�^
const int RealtimeData_j = 4;  //532�s�`535�s�ł������̍��ڂ��폜��������6����S��
const int timeStr_i = 3;

// �o�b�t�@�[�T�C�Y
const int SERIAL_MAX = 250;
const int TIME_MAX = 32;
const int FILE_NAME_MAX = 32;

// �w�b�_�[
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

// ������(rs,e,d4,d5,d6,d7) �ڑ����Ȃ��s����GND�ɗ��Ƃ��Ă����ƈ��肷��
TextOLED lcd(p15, p16, p17, p24, p25, p26, TextOLED::LCD20x4);

I2C i2c(p9, p10);  // sda, scl
INA226 BatteryMonitor(i2c, BATTERY_MONITOR_ADDRESS, 10000);
INA226 PanelMonitor(i2c, PANEL_MONITOR_ADDRESS, 10000);
INA226 MotorMonitor(i2c, MOTOR_MONITOR_ADDRESS, 10000);
INA226 BatteryMonitor2(i2c, BATTERY_MON2_ADDRESS, 10000);

MSCFileSystem msc("usb");  // USB�������ɂ�/usb/...�ŃA�N�Z�X

AnalogIn motorTempAnalogIn(p19);  // ���[�^�R�C�����x
//I nterruptIn motorPulse(p12);      // ���[�^�[�p���X(�s���ݒ�)


struct LogData {
  char timeStr[timeStr_i][TIME_MAX];
  /* [0]:�^�C���X�^���v    20160801121356
     [1]:���ݎ���          12:13:56
     [2]:UNIXTIME          1900�N1��1������̌o�ߎ���
   */
  unsigned int totalTime;
  int totalTimeNoon;            // ���߂�0�Ƃ����o�ߎ���
  double RealtimeData[RealtimeData_i][RealtimeData_j];  // �d���A�d�́A���ρA�ώZ
  double batteryVoltage;        // �d��
  double remainBatteryWh;       // �c��[Wh]
  //double remainBatteryParcent;  // �c��[%]
  double motorTemp;             // ���[�^�[�R�C�����x
  double motorSpeed;            // ���[�^�[���x[km/h]
  double gpsSpeed;              // GPS�Βn���x[km/h]
  double gpsAngle;              // �i�s����
  double distanceMotorSpeed;    // ���[�^�[���x���狗��[km]
  double latitude;              // �ܓx
  double longitude;             // �o�x
  //double motorSpeed_pulse;      // ���[�^�[���x[km/h] by�p���X

  // CAN MITSUBA
  double motorVoltage;  // MITSUBA CAN �d��
  double motorCurrent;  // MITSUBA CAN �d��
  double tempFET;       // MITSUBA CAN FET���x
  double motorRPM;      // MITSUBA CAN ��]��
  double duty;          // MITSUBA CAN duty��
  double angle;         // MITSUBA CAN �i�p

  // CAN SOLARLOGGER
  double SOLARLOGGERVoltage[12];  // MPPT���͓d��1-12ch
  double SOLARLOGGERCurrent[12];  // MPPT���͓d��1-12ch
  double SOLARLOGGERPower[12];    // MPPT���͓d��1-12ch�i�d���~�d���̉��Z�j
  double SOLARLOGGERTemp[16];     // �A���C���x16ch

  // CAN BMS
  double sumBatteryVoltage;   // BMS���v�o�b�e���Z���d��
  double minCellVoltage;      // �ŏ��Z���d��
  double maxCellVoltage;      // �ő�Z���d��
  // double cellVoltage[28];     // �S�Z���d��
};

bool secTimer = false;
struct LogData logdata = {0};
time_t startGPSTime = 0;
unsigned int storageCount = 0;
// unsigned int pulseCount = 0;     // �ԑ��p���X�J�E���g�ϐ�
bool BMSConnectFlag = false;        // BMS�ʐM�����t���O
bool BMSErrorFlag = false;                // BMS�G���[�t���O
bool BMSErrorFlagOverDischarge = false;   // �ߕ��d�t���O
bool BMSErrorFlagOverCharge = false;  // �ߏ[�d�t���O
bool BMSErrorFlagOverTemp = false;    // �߉��x�t���O
bool BMSErrorFlagLowTemp = false;     // �ߒቷ�t���O

/*�v���g�^�C�v�錾*/
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
//void MotorPulseCount(void);  //�ԑ��p���X�J�E���g�֐�
void SendCANMessage(void);
void ReceiveCANMessage(void);  // CAN��M���荞�݃n���h��
void DisplayBMSError(void);
double MoD0(double vaLue);
double MoD1(double vaLue);
double MoD2(double vaLue);
double MoD3(double vaLue);

int main(){
  char fileName[5][FILE_NAME_MAX] = {0};   // USB�t�@�C����  [0]:LOG, [1]:GPS,[2]CAN_MITSUBA, [3]:CAN_SOLAR, [4]:CAN_BMS

  debug.baud(9600);
  Xbee.baud(57600);  // ���S�ƐM��()��57600bps
  myGPS.begin(9600);
  myGPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  myGPS.sendCommand(PMTK_SET_NMEA_UPDATE_5HZ);
  myGPS.sendCommand(PGCMD_ANTENNA);

  lcd.cls();

  if(SetupINA226()) {
    NVIC_SystemReset();
    return -1;
  }

  // GPS�⑫����܂Ń��[�v����
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

  //motorPulse.fall(&MotorPulseCount);  //�s�����������芄�荞�݂ŁA�p���X�J�E���g�֐��̎��s
  //�p���X�ő��x�����邱�Ƃ��l���Ă��Ȃ��Ȃ���Ȃ���

  lcd.cls();
  canBusSystem.frequency(250000);
  canBusSystem.attach(&ReceiveCANMessage, CAN::RxIrq);  // CAN��M���荞�݂̎��s
  RTC::attach(&MainFuncTimer, RTC::Second);             // MainFuncTimer��1�b���ƂɎ��s

  // ���C�����[�v
  while(1) {
    FetchGPSdata(fileName);
    // 1�b���ƂɎ��s
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
    // STORAGE_TIME���ƂɎ��s
    if(storageCount >= STORAGE_TIME) {
      storageCount = 0;
      XbeeSerial(&logdata);
    }
    // BMS�x����
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

  // BMS�x���łȂ��Ȃ�ʏ�f�B�X�v���C�\��
  if(!BMSErrorFlag){
    LCD(&logdata);
  }
  
  storageCount++;
}

/** �S�ẴX�g���[���n�ɕ�������o��
 * @param const char *str ���M������
 * @param int row ��
 * @param int col �s
 */
void print(const char *str, int row, int col){
  debug.printf("%s", str);
  Xbee.printf("%s", str);
  lcd.locate(row, col);
  lcd.printf("%s", str);
}

/** INA226 ���W�X�^�������ݓ�
 * @return 0:����     -1:���s
 */
int SetupINA226(){
  unsigned short val_1 = 0;
  unsigned short val_2 = 0;
  unsigned short val_3 = 0;
  
  // INA226�̑��݂��m�F
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
  // INA226�̃��W�X�^�̒l�𐳂����ǂ߂邩�m�F
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
  // �eINA226�Ƀ��W�X�^�l����������
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

/** USB�������������ݏ����� �t�@�C�����ݒ�
 * @param struct LogData LogData �\����
 * @param char name �t�@�C�������������܂��z��
 *
 * @return 0: ���� -1: ���s
 */
int SetupUSB(struct LogData *data, char name[][FILE_NAME_MAX]){
  time_t createdTime = time(NULL) + OFFSET_TIME;
  // ���t���t�@�C����
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
    fprintf(fp, "%s\n", data->timeStr[0]);    // �����Ɍ��ݎ�������������
    fprintf(fp, "%s\n", USB_HEADER_STRING);   // �w�b�_�[��������
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
    fprintf(fp, "%s\n", data->timeStr[0]);    // �����Ɍ��ݎ�������������
    fprintf(fp, "%s\n", HEADER_CAN_MITSUBA);   // �w�b�_�[��������
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
  //   fprintf(fp, "%s\n", data->timeStr[0]);    // �����Ɍ��ݎ�������������
  //   fprintf(fp, "%s\n", HEADER_CAN_SOLAR);   // �w�b�_�[��������
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
    fprintf(fp, "%s\n", data->timeStr[0]);    // �����Ɍ��ݎ�������������
    fprintf(fp, "%s\n", HEADER_CAN_BMS);   // �w�b�_�[��������
  }
  fclose(fp);

  print("USB OK!\n", 0, 3);
  
  return 0;
}

/** �o�b�N�A�b�v�f�[�^��ǂݍ��݁CLogData�\���̂Ɋi�[
 * @param struct LogData �o�b�N�A�b�v����LogData �\���̂ɏ�������
 *
 * @return 0:����     1:�o�b�N�A�b�v�f�[�^�����݂��Ă��Ȃ�
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
    return 1;  // ����N���� backup.csv��SaveBackup�֐������s���ꂽ���ɍ쐬�����
  }
  else{
    fscanf(fp_back, "%d", &backupTime);
    t_backup = localtime(&backupTime);
    if(t_now->tm_mday == t_backup->tm_mday) {  // ������t�̂�
      fscanf(fp_back, "%d,%d", &(data->totalTime), &startGPSTime);
    }
    for(int i = 0; i < RealtimeData_i; i++) {
      fscanf(fp_back, "%lf,", &(data->RealtimeData[i][3]));  // �ώZ�i�ώZ�d���̍��ڏ�����������j
    }
    fscanf(fp_back, "%lf", &(data->distanceMotorSpeed));
  }
  fclose(fp_back);
  return 0;
}

/** �V���A��GPS�f�[�^����́C�ۑ�
 * @param const char name �t�@�C�������ۑ�����Ă���z��
 *
 * @return 0:����    -1:���s
 */
int FetchGPSdata(const char name[][FILE_NAME_MAX]){
  // char c;
  while(1) {
    myGPS.read();                            // �����ǂނ����łȂ��āCGPS�Z���e���X�𕶎��񉻂��Ă���
    // if (c) debug.printf("%c", c);
    if (myGPS.newNMEAreceived()) {           // �Z���e���X���ꕶ�擾�ł������ǂ���
      if (!myGPS.parse(myGPS.lastNMEA())) {  // ��͂������������ǂ��� ���Ă��Ȃ��ꍇ�͂�����x���[�v(break�X�L�b�v)
        continue;
      }
      break;                                 // ��͏I�������ꍇ�̓��[�v�𔲂���
    }
  }
  // ���O��GPS�Z���e���X��USB�ɕۑ�
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

/** �����֘A�̃f�[�^���v�Z�C�i�[
 * @param struct LogData
 */
void CalculateTimeSet(struct LogData *data) {
  time_t nowTime;        // ���ݎ���
  time_t totalTime_tmp;  // �o�ߎ���
  int hour = 0;
  int minute = 0;
  int second = 0;

  nowTime = time(NULL) + OFFSET_TIME;

  totalTime_tmp = nowTime - startGPSTime;
  data->totalTime = (unsigned int) totalTime_tmp;

  strftime(data->timeStr[0], TIME_MAX, "%Y%m%d%H%M%S", localtime(&nowTime));   // PHP,USB�p
  strftime(data->timeStr[1], TIME_MAX, "%T", localtime(&nowTime));             // LCD�̌��ݎ����p
  snprintf(data->timeStr[2], TIME_MAX, "%u", (unsigned int) nowTime);

  sscanf(data->timeStr[1], "%d:%d:%d", &hour, &minute, &second);
  data->totalTimeNoon = (hour * 3600 + minute * 60 + second) - 3600 * 12;  // ���߂���Ƃ����o�ߎ���
}

/** 1�b���ɓd���E�d������C�v�Z��̃f�[�^���\���̂Ɋi�[
 * @param struct LogData
 * @param  bool run : true �ώZ���� false �ώZ���Ȃ�
 */
void CalculateSet(struct LogData *data, bool run){
  bool retry = false;
  int count = 0;

  double voltage_tmp = 0;      // ���ɓd��,�d���f�[�^��ۑ����Ă���
  double current_tmp[RealtimeData_i] = {0};
  double vAl[4] = {0};

  // �ُ�Ȓl�̏ꍇ�́C�ēx�f�[�^����蒼��
  do {
    retry = false;
    
    BatteryMonitor2.getVoltage(&vAl[0]);     // �o�b�e���[�d��
    BatteryMonitor.getVoltage(&vAl[1]); // �o�b�e���[�d��
    PanelMonitor.getVoltage(&vAl[2]);   // �p�l���d��
    MotorMonitor.getVoltage(&vAl[3]);   // ���[�^�[�d��
    
    current_tmp[0] = MoD1(vAl[1]);    // �o�b�e���[�d���ϊ���̒l��current_tmp[0]�ɑ��fx0�����Ȃ���
    current_tmp[1] = MoD2(vAl[2]);    //����
    current_tmp[2] = MoD3(vAl[3]);    //����
    voltage_tmp = MoD0(vAl[0]);       //����
    
   //data->motorSpeed_pulse = pulseCount * pulseConvert;     // 1�b�ԂŃJ�E���g�����p���X�𑬓x�ɕϊ�
   // pulseCount = 0;//�p���X�̃��Z�b�g
   //���p���X�͎g��Ȃ��Ƃ��Ă���

    if(voltage_tmp > 150) retry = true;         // �d����150V�ȏ�ُ͈�
    for(int i = 0; i < RealtimeData_i; i++) {
      if(current_tmp[i] > 60) retry = true;    // �d����60A�ȏ�ُ͈�
    }
    count++;
  } while(retry && count < 5);  // 5��Ƃ��Ă��ُ�ȏꍇ�͂������Ƀ��[�v�𔲂���

  // 0���̗L�����m�F������C�z��Ɋi�[����
  if(data->totalTime != 0) {
    data->batteryVoltage = voltage_tmp; // �����d����␳
    for(int i = 0; i < RealtimeData_i; i++) {
      data->RealtimeData[i][0] = current_tmp[i];  // �d��
      data->RealtimeData[i][1] = data->batteryVoltage * data->RealtimeData[i][0];   // �d��
      if(run) {
        //data->RealtimeData[i][2] += data->RealtimeData[i][0] / 3600;            // �ώZ�d��
        data->RealtimeData[i][3] += data->RealtimeData[i][1] / 3600;            // �ώZ�d��
       // data->RealtimeData[i][4] = data->RealtimeData[i][2] * 3600 / data->totalTime; // ���ϓd��
        //data->RealtimeData[i][5] = data->RealtimeData[i][3] * 3600 / data->totalTime; // ���ϓd��
      }
    }
    data->remainBatteryWh = BATTERY_CAPACITY - data->RealtimeData[0][3];     // �o�b�e���[�c��[Wh]
    // data->remainBatteryParcent = (data->remainBatteryWh / BATTERY_CAPACITY) * 100;    // �o�b�e���[�c��[%]
    data->gpsSpeed = myGPS.speed * NOTS_CONVERT;  // GPS���x
    data->gpsAngle = myGPS.angle;                 //�i�s����
    data->latitude = myGPS.latitude;              // GPS�ܓx
    data->longitude = myGPS.longitude;            // GPS�o�x

    data->motorSpeed = data->motorRPM * RPM_CONVERT;  //���[�^���x
    data->distanceMotorSpeed += data->motorRPM * RPM_CONVERT_MS / 1000;  //���s����

    // ���[�^�[�R�C�����x�Z���T�i������LM35�j �l�擾(�ړ����Ϗ���)
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
      data->motorTemp += motorTemp_tmp[i] / TEMP_AVE_CNT;  // 10�񕽋�
    }
    // debug.printf("\ntemp %d: %.2f\n", i, data->motorTemp);
  }
}

/*
 * �f�[�^���M�T���v��
 *
 */
void XbeeSerial(const struct LogData *data){
  char SerialSendStr[SERIAL_MAX] = {0};

  snprintf(SerialSendStr, SERIAL_MAX, "%s,%d,%.2f", data->timeStr[1], data->totalTimeNoon, data->batteryVoltage);

  // �d��
  for(int i = 0; i < RealtimeData_i; i++) {
    snprintf(SerialSendStr, SERIAL_MAX, "%s,%.2f", SerialSendStr, data->RealtimeData[i][0]);
  }

  // �d��
  for(int i = 0; i < RealtimeData_i; i++) {
    snprintf(SerialSendStr, SERIAL_MAX, "%s,%.2f", SerialSendStr, data->RealtimeData[i][1]);
  }

  // �ώZ�d��
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

  // ���ϓd��
  /* for(int i = 0; i < RealtimeData_i; i++) {
    snprintf(SerialSendStr, SERIAL_MAX, "%s,%.2f", SerialSendStr,  data->RealtimeData[i][5]);
  }*/

  snprintf(SerialSendStr, SERIAL_MAX, "%s,%.1f,%.1f", SerialSendStr,
            data->distanceMotorSpeed,
            data->gpsSpeed);//�ugpsSpeed�v���umotorSpeed_pulse�v���ugpsSpeed�v�p���X�͎g��Ȃ����j�ōl����

  Xbee.printf("%s\r\n", SerialSendStr);  // ���s�R�[�h�ύX \n �� \r\n
}

/*LCD�ւ̃f�[�^�\�����s�� locate�֐���0�s0�񂩂�͂��܂�C������(��C�s)*/
void LCD(const struct LogData *data){
  lcd.locate(0, 0);
  lcd.printf("%s", data->timeStr[1]);
  lcd.locate(11, 0);
  lcd.printf("%3.0f", data->motorSpeed);  //�umotorSpeed�v���umotorSpeed_pulse�v���umotorSpeed�v�p���X�͎g��Ȃ����j�ōl����
  lcd.locate(15, 0);
  lcd.printf("km/h");

  lcd.locate(0, 1);
  lcd.printf("%6.1f", data->remainBatteryWh);  //���\�L����Wh�ցAWh�ł݂�ȕ�����͂�
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

  // BMS �ʐM�����C���W�P�[�^
  if(BMSConnectFlag){
    BMSConnectFlag = false;
    lcd.locate(19, 3);
    lcd.printf("*");  // �u*�v��\��
  }
  else{
    lcd.locate(19, 3);
    lcd.printf(" ");  // �u*�v����
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

/*�\���̂̃f�[�^��USB�������փf�[�^��ۑ�(csv�t�@�C��)*/
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
        fprintf(fp, ",%.2f", data->RealtimeData[i][j]);  // �S�f�[�^
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

/** �~�ς���Ă���f�[�^��csv�t�@�C���Ƀo�b�N�A�b�v
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
    fprintf(fp_back, "%d,%d\r\n", data->totalTime, (unsigned int)startGPSTime);  //  ����
    for(int i = 0; i < RealtimeData_i; i++) {
      //for(int j = 0; j < 2; j++){
      fprintf(fp_back, "%f,", data->RealtimeData[i][3]);  // �ώZ�d��
      //}
      fprintf(fp_back, "\r\n");
    }
    fprintf(fp_back, "%f\r\n", data->distanceMotorSpeed);  // ���s����
    fclose(fp_back);
  }
  return 0;
}

/* void MotorPulseCount() {
    pulseCount++;    // �X�g���[�����Ńf�o�b�N����ƒx���Ő��m�Ƀp���X���𑪒�ł��Ȃ�
}*/
//�p���X�͎g��Ȃ����j�ōl����̂Ńo��

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
  const double BMS_VOLTAGE_LSB = 0.0001;  // BMS �Z���d����1LSB

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

    // BMS �ŏ��E�ő�E���v�o�b�e���Z���d��
    if(msg.id == RECEIVE_ID_BMS[0]){
      BMSConnectFlag = true;
      data->minCellVoltage = (double)(((uint16_t)msg.data[0]) | ((uint16_t)msg.data[1]) << 8) * BMS_VOLTAGE_LSB;
      data->maxCellVoltage = (double)(((uint16_t)msg.data[2]) | ((uint16_t)msg.data[3]) << 8) * BMS_VOLTAGE_LSB;
      data->sumBatteryVoltage = (double)(((uint16_t)msg.data[4]) | ((uint16_t)msg.data[5] << 8) | ((uint16_t)msg.data[6] << 16)) * BMS_VOLTAGE_LSB;
    }

    // // BMS �e�Z���d��
    // for(int i = 1; i < 8; i++){
    //   if(msg.id == RECEIVE_ID_BMS[i]){
    //     for(int j = 4 * (i - 1); j < 4 * i; j++){
    //       data->cellVoltage[j] = (double)(((uint16_t)msg.data[2 * (j % 4)]) | ((uint16_t)msg.data[2 * (j % 4) + 1] << 8)) * BMS_VOLTAGE_LSB;
    //     }
    //   }
    // }

    // Battery Error�p
    if (msg.id == RECEIVE_ID_BMS_ERROER){
      BMSConnectFlag = true;  // BMS �ʐM�����t���O
      if (msg.data[0] & FLAG_OVER_DISCHARGE){
        BMSErrorFlag = true;  // BMS �G���[�\���t���O
        BMSErrorFlagOverDischarge = true;  // �ߏ[�d
      }
      if (msg.data[0] & FLAG_OVER_CHARGE){
        BMSErrorFlag = true; // BMS �G���[�\���t���O
        BMSErrorFlagOverCharge = true;  // �ߕ��d
      }
      if (msg.data[0] & FLAG_OVER_TEMP){
        BMSErrorFlag = true; // BMS �G���[�\���t���O
        BMSErrorFlagOverTemp = true;  // �߉��x
      }
      if (msg.data[0] & FLAG_LOW_TEMP){
        BMSErrorFlag = true; // BMS �G���[�\���t���O
        BMSErrorFlagLowTemp = true;  // �ߒቷ
      }
    }
  }
}

double MoD0(double vaLue)//�o�b�e���[�d��
{
    
  return (vaLue*10);    
    
}     

double MoD1(double vaLue)//�o�b�e���[�d��
{
    
  return ((vaLue-2.510)*40);    
    
}     

double MoD2(double vaLue)//�p�l���d��
{
    
  return ((vaLue-2.507)*40);   ;    
    
}     

double MoD3(double vaLue)//���[�^�[�d��
{
    
  return ((vaLue-2.5025)*40);   ;    
    
}     
