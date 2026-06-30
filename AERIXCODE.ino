#include <Wire.h>
#include <WiFi.h>
#include <FirebaseESP32.h>
#include <Adafruit_ADS1X15.h>
#include <Adafruit_INA219.h>
#define WIFI_SSID "AERIX"
#define WIFI_PASSWORD "Katasandi"

#define API_KEY "AIzaSyCjfUpR2CUcNEgD7su3SbBww81JHx5WJPs"
#define DATABASE_URL "https://aerix-2425-01-104-default-rtdb.asia-southeast1.firebasedatabase.app"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

#define SDA_I2C 21
#define SCL_I2C 22

Adafruit_ADS1115 ads;
Adafruit_INA219 ina219;

#define ZVS_GATE_PIN 19
#define RESET_BTN_PIN 18
#define HW_SWITCH_PIN 17
#define PM_LED_PIN 5

#define CH_CO 0
#define CH_PM25 1

#define ADS_LSB_GAIN_SIXTEEN     0.0000078125   // ±0.256 V
#define ADS_LSB_GAIN_TWOTHIRDS  0.0001875      // ±6.144 V

#define EMA_ALPHA 0.2
#define CO_AVG_N 10
#define PM_AVG_N 10

float co_ema = 0;
bool co_ema_init = false;
float co_buf[CO_AVG_N];
int co_idx = 0;
bool co_full = false;

float pm_ema = 0;
bool pm_ema_init = false;
float pm_buf[PM_AVG_N];
int pm_idx = 0;
bool pm_full = false;

float co_ppm = 0;
float pm25_ug = 0;

float pm_raw = 0;

#define INA_AVG 10

float vin_v = 0;
float current_a = 0;
float power_w = 0;

#define CURRENT_LIMIT 0.8
#define CURRENT_RESET_LEVEL 0.5

bool zvs_lock = false;
bool zvs_allow = false;
bool zvs_active = false;
bool app_zvs_allow = false;

unsigned long zvs_start_time = 0;

unsigned long t_sensor = 0;
unsigned long t_control = 0;
unsigned long t_firebase = 0;

#define SENSOR_INTERVAL 200
#define CONTROL_INTERVAL 800
#define FIREBASE_INTERVAL 800


float convertCO(float v)
{
  float ppm = (v - 0.0411) / 0.000402;

  if(ppm < 0)
    ppm = 0;

  return ppm;
}

float convertPM(float v)
{
  float pm = (v - 0.1114) / 0.00883;

  if(pm < 0)
    pm = 0;

  return pm;
}
float readCO_voltage()
{
  ads.setGain(GAIN_SIXTEEN);
  delay(2);

  int16_t adc = ads.readADC_SingleEnded(CH_CO);

  float v_raw = adc * ADS_LSB_GAIN_SIXTEEN;

  if(!co_ema_init)
  {
    co_ema = v_raw;
    co_ema_init = true;
  }
  else
  {
    co_ema = EMA_ALPHA * v_raw + (1 - EMA_ALPHA) * co_ema;
  }

  co_buf[co_idx++] = co_ema;

  if(co_idx >= CO_AVG_N)
  {
    co_idx = 0;
    co_full = true;
  }

  int count = co_full ? CO_AVG_N : co_idx;

  float sum = 0;

  for(int i = 0; i < count; i++)
  {
    sum += co_buf[i];
  }

  return sum / count;
}
float readPM_voltage()
{
  ads.setGain(GAIN_TWOTHIRDS);
  delay(2);

  digitalWrite(PM_LED_PIN, HIGH);

  delayMicroseconds(280);

  int16_t adc = ads.readADC_SingleEnded(CH_PM25);

  digitalWrite(PM_LED_PIN, LOW);

  float v_raw = adc * ADS_LSB_GAIN_TWOTHIRDS;

  if(!pm_ema_init)
  {
    pm_ema = v_raw;
    pm_ema_init = true;
  }
  else
  {
    pm_ema = EMA_ALPHA * v_raw + (1 - EMA_ALPHA) * pm_ema;
  }
  pm_buf[pm_idx++] = pm_ema;

  if(pm_idx >= PM_AVG_N)
  {
    pm_idx = 0;
    pm_full = true;
  }
  int count = pm_full ? PM_AVG_N : pm_idx;
  float sum = 0;
  for(int i = 0; i < count; i++)
  {
    sum += pm_buf[i];
  }

  return sum / count;
}
void readINA()
{
  float bv = 0;
  float ia = 0;
  float pw = 0;

  float current_raw = ina219.getCurrent_mA() / 1000.0;

  if(!zvs_lock &&
     millis() - zvs_start_time > 1000 &&
     current_raw >= CURRENT_LIMIT)
  {
    zvs_lock = true;

    digitalWrite(ZVS_GATE_PIN, LOW);

    Serial.print("TRIP! RAW CURRENT: ");
    Serial.println(current_raw);
  }
  for(int i = 0; i < INA_AVG; i++)
  {
    bv += ina219.getBusVoltage_V();
    ia += ina219.getCurrent_mA();
    pw += ina219.getPower_mW();
  }

  vin_v = bv / INA_AVG;
  current_a = (ia / INA_AVG) / 1000.0;
  power_w = (pw / INA_AVG) / 1000.0;
}
void tampilserial()
{
  Serial.println("===== AERIX =====");

  Serial.print("CO : ");
  Serial.println(co_ppm);

  Serial.print("PM25 : ");
  Serial.println(pm25_ug);

  Serial.print("VIN : ");
  Serial.print(vin_v);
  Serial.println(" V");

  Serial.print("CURRENT : ");
  Serial.print(current_a);
  Serial.println(" A");

  Serial.print("POWER : ");
  Serial.print(power_w);
  Serial.println(" W");

  Serial.print("ZVS ACTIVE : ");
  Serial.println(zvs_active);

  Serial.println();
}

void setup()
{
  Serial.begin(115200);

  Wire.begin(SDA_I2C, SCL_I2C);

  pinMode(ZVS_GATE_PIN, OUTPUT);
  pinMode(RESET_BTN_PIN, INPUT_PULLUP);
  pinMode(HW_SWITCH_PIN, INPUT_PULLUP);
  pinMode(PM_LED_PIN, OUTPUT);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  while(WiFi.status() != WL_CONNECTED)
  {
    delay(300);
  }

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;

  Firebase.signUp(&config, &auth, "", "");
  Firebase.begin(&config, &auth);

  Firebase.reconnectWiFi(true);

  ads.begin(0x48);
  ads.setGain(GAIN_SIXTEEN);

  ina219.begin();
  ina219.setCalibration_32V_2A();

  Serial.print("CURRENT LIMIT: ");
  Serial.println(CURRENT_LIMIT);

  Serial.println("WARMING UP SENSOR...");

  delay(60000);
}

void loop()
{
  unsigned long now = millis();

  bool hw_on = (digitalRead(HW_SWITCH_PIN) == LOW);

  zvs_allow = hw_on && app_zvs_allow && !zvs_lock;

  digitalWrite(ZVS_GATE_PIN, zvs_allow);

  static bool last_zvs = false;

  if(zvs_allow && !last_zvs)
  {
    zvs_start_time = millis();
  }

  last_zvs = zvs_allow;

  /* ===== SENSOR READ ===== */

  if(now - t_sensor >= SENSOR_INTERVAL)
  {
    t_sensor = now;

    readINA();

    float v_co = readCO_voltage();
    co_ppm = convertCO(v_co);
    float v_pm = readPM_voltage();
    pm_raw = convertPM(v_pm);
    pm25_ug = pm_raw;
 
    zvs_active = zvs_allow && current_a > 0.1;

     tampilserial();
  }

  if(Firebase.ready() && now - t_control >= CONTROL_INTERVAL)
  {
    t_control = now;

    if(Firebase.getBool(fbdo, "/CONTROL/ZVS_ALLOW"))
    {
      app_zvs_allow = fbdo.boolData();
    }
  }
  if(zvs_lock &&
     digitalRead(RESET_BTN_PIN) == LOW &&
     current_a < CURRENT_RESET_LEVEL)
  {
    zvs_lock = false;
  }

  if(Firebase.ready() && now - t_firebase >= FIREBASE_INTERVAL)
  {
    t_firebase = now;

    Firebase.setFloat(fbdo, "/SENSOR/CO", co_ppm);
    Firebase.setFloat(fbdo, "/SENSOR/PM25", pm25_ug);

    Firebase.setFloat(fbdo, "/INA219/VIN", vin_v);
    Firebase.setFloat(fbdo, "/INA219/CURRENT", current_a);
    Firebase.setFloat(fbdo, "/INA219/POWER", power_w);

    Firebase.setBool(fbdo, "/STATUS/ZVS_ACTIVE", zvs_active);
    Firebase.setBool(fbdo, "/STATUS/LOCK", zvs_lock);
  }
}