#include <string>
#include <Wire.h>
#include <ESP32Time.h>
#include "ota.h"
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "hal/gpio_ll.h"

//all timer must milisecond
#define BLE_advertising_duration 20000
#define reaction_time 30000
#define slow_tasks_tick_period 50
#define menu_time_out 30000
#define battery_control_period 10000 * 1000
#define low_battery_percentage 3
#define wake_up_timer_max_value 0x7FFFFFFFFFFFFFFF
#define sleeping_delay_after_parking_time_is_exceed 1200000
#define ota_connection_accepting_duration 1200000

int duration = 15;
unsigned long current_milisecond;
unsigned long timer_last_motion, timer_last_motionless_state, timer_tick, timer_last_entry_in_menu;
bool flag = 0;
int mode = 3;
int hour_park = 23;
int minute_park = 59;
int manuel_parked = 0;

#include "display_static_LCD.h"
#define display_pin_count 32
#define delta 50
/*
uint8_t pin_map[display_pin_count] = {
  A2, A3, 59, 58, A0, A1, 60, SCL,
  6, 5, 63, 62, 61, 9, 64,
  A5, SCK, 52, 51, 50, A4, 53,
  MISO, RX, 56, 55, 54, MOSI, 57,
  65, TX
};*/
uint8_t pin_map[display_pin_count] = {
  52, 53, 74, 75, 76, 51, 50,  //1
  56, 57, 71, 72, 73, 55, 54,  //2
  61, 62, 83, 84, 85, 60, 59,  //3
  65, 78, 79, 80, 81, 64, 63,  //4
  58, 70, 77                   //col,dp2,bp
};
int seperator = 1;
Display_static_LCD LCD{ 4, 7, 3, 0, 32, &seperator };

#include "ble.h"
Ble_service ble;
unsigned long timer_ble_on;

ESP32Time rtc(3600);
int sec = 0, minute = 4, hour = 0, day = 25, month = 3, year = 2024;

#include <MPU9250_WE.h>
#define MPU9250_ADDR 0x68
MPU9250_WE mpu = MPU9250_WE(MPU9250_ADDR);
const int intPin = 3;
volatile bool motion = false;

//#include "Adafruit_MAX1704X.h"
//Adafruit_MAX17048 maxlipo;

#define pin_a 13
#define pin_b 14
#define pin_plus 1
#define pin_minus 2
#define ext_power 10
#define adc_battery A8

void TaskMainAttirubutes();  //void* pvParameters);
void TaskDisplay();          //void* pvParameters);
//TaskHandle_t task_handle;

void next_stop(int& hour, int& minute);
unsigned int differance(unsigned int value1, unsigned int value2);
void deep_sleep_on(unsigned long long wakeup_timer = wake_up_timer_max_value, int ext_wake_up1 = 0);
int battery_status();

void setup() {
  Serial.begin(115200);



  Wire.begin(18, 8, 400000);
  pinMode(ext_power, OUTPUT);
  digitalWrite(ext_power, 1);
  LCD.init(pin_map);

  analogSetAttenuation(ADC_0db);
  if (battery_status() < 3000) {
    Serial.printf("Battey low! Vbatt:%d", battery_status());
    deep_sleep_on(wake_up_timer_max_value, 0);
  }

  //ota_setup();

  std::sscanf(__TIME__, "%d:%d:%d", &hour, &minute, &sec);

  //rtc.setTime(sec + 15, minute, hour - 1, day, month, year);

  //while (!maxlipo.begin() && trying_count--) {
  //  delay(200);
  //}
  //trying_count = max_trying_count;
  //delay(1000);

  //float percentage = constrain(maxlipo.cellPercent(), 0, 100);
  //Serial.printf("battery percentage:%f   :%d\n", percentage, percentage);
  //if (percentage < low_battery_percentage) {

  //  seperator = 1;
  //  LCD.print(0xCA00 + to_hex(percentage), 5000);
  //  deep_sleep_on(battery_control_period);
  //}


  //Serial.println("Failed to find MPU6050 chip");
  while (!mpu.init() && trying_count--) {
    Serial.println("Failed to find MPU6050 chip");
    delay(10);
  }
  mpu.setSampleRateDivider(5);
  mpu.setAccRange(MPU9250_ACC_RANGE_2G);
  mpu.enableAccDLPF(true);
  mpu.setAccDLPF(MPU9250_DLPF_6);
  mpu.setIntPinPolarity(MPU9250_ACT_HIGH);
  mpu.enableIntLatch(0);
  mpu.enableClearIntByAnyRead(false);
  mpu.enableInterrupt(MPU9250_WOM_INT);
  mpu.setWakeOnMotionThreshold(2);
  mpu.enableWakeOnMotion(MPU9250_WOM_ENABLE, MPU9250_WOM_COMP_ENABLE);
  attachInterrupt(digitalPinToInterrupt(intPin), motionISR, RISING);
  /*
  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setHighPassFilter(MPU6050_HIGHPASS_UNUSED);
  mpu.setInterruptPinLatch(true);  // Keep it latched.  Will turn off when reinitialized.
  mpu.setInterruptPinPolarity(true);
  mpu.setMotionInterrupt(true);
  mpu.setMotionDetectionThreshold(1);
  mpu.setMotionDetectionDuration(1);
  mpu.setCycleRate(MPU6050_CYCLE_40_HZ);*/
  //mpu.enableCycle(true);
  //mpu.setMotionDetectionDuration(1000);

  pinMode(pin_a, INPUT_PULLUP);
  pinMode(pin_b, INPUT_PULLUP);
  pinMode(pin_plus, INPUT_PULLUP);
  pinMode(pin_minus, INPUT_PULLUP);




  timer_last_motion = millis();
  //ble.init();
  /*

  xTaskCreatePinnedToCore(
    TaskMainAttirubutes, "TaskMainAttirubutes", 2048  // Stack size
    ,
    NULL  // When no parameter is used, simply pass NULL
    ,
    2  // Priority
    ,
    &task_handle  // With task handle we will be able to manipulate with this task.
    ,
    0  // Core on which the task will run
  );
  xTaskCreatePinnedToCore(
    TaskDisplay, "Analog Read", 2048  // Stack size
    ,
    NULL  // When no parameter is used, simply pass NULL
    ,
    3  // Priority
    ,
    &task_handle  // With task handle we will be able to manipulate with this task.
    ,
    1  // Core on which the task will run
  );*/
}

void loop() {
  current_milisecond = millis();
  TaskMainAttirubutes();
  TaskDisplay();
  if (current_milisecond < ota_connection_accepting_duration) {
    ota_loop();
  }
}

void next_stop(int& hour, int& minute) {  //<<sonraki saat-

  for (int i = 1; i <= 60 / duration; i++) {
    if (minute < i * duration) {
      minute = i * duration;
      if (minute > 59) {
        minute = minute % 60;
        hour++;
      }
      break;
    }
    Serial.printf("%d:%d setted\n", hour, minute);
  }
}

void TaskMainAttirubutes()  //void* pvParameters)
{
  //while (1) {

  tm tstruct = rtc.getTimeStruct();
  minute = tstruct.tm_min;
  hour = tstruct.tm_hour;
  sec = tstruct.tm_sec;

  static uint8_t parking_time_exceed;
  static unsigned long time_of_parking_time_is_exceed;
  static unsigned long timer_after_parking_time_is_exceed;

  if (!digitalRead(pin_a)) {
    mode = 2;
    timer_last_entry_in_menu = current_milisecond;
  }  //Setting mode

  int status = mpu.readAndClearInterrupts();
  if (mpu.checkInterrupt(status, MPU9250_WOM_INT)) {  //Motion dedector
    timer_last_motion = current_milisecond;
    //mpu.readAndClearInterrupts();
    //Serial.printf("motion:%d\n", status);
  }
  //Serial.printf("motion:%d\n",status);

  if (differance(current_milisecond, timer_last_motion) > 2000) {  //timer_last_motion reset during short term motion or motionless state
    timer_last_motionless_state = current_milisecond;
    if (mode == 3) {
      deep_sleep_on(wake_up_timer_max_value, 1);
    }
  }


  if (differance(current_milisecond, timer_tick) > slow_tasks_tick_period) {  //slow tasks
    timer_tick = current_milisecond;

    if ((hour >= hour_park) && (minute >= minute_park)) {
      manuel_parked = 0;
      parking_time_exceed = 1;
      timer_after_parking_time_is_exceed = current_milisecond;
    }

    if ((differance(current_milisecond, timer_last_motion) > reaction_time) && (mode == 0)) {  //Parking mode is activated
      next_stop(hour, minute);
      hour_park = hour;
      minute_park = minute;

      ble.init();
      ble.advertise_start();
      timer_ble_on = current_milisecond;
      ble.ble_data(hour_park, minute_park);

      parking_time_exceed = 0;

      mode = 1;
    }

    if (differance(current_milisecond, timer_last_motionless_state) > reaction_time) {  //Returns to normal mode as motion is detected
      if (!manuel_parked) {                                                             //In case of manuel parking, mode change is prevented due to motion detection
        mode = 0;
      }
      if (mode == 3) {
        mode = 0;
        timer_last_motion = current_milisecond;
      }
    }



    if (parking_time_exceed && (differance(current_milisecond, time_of_parking_time_is_exceed) > sleeping_delay_after_parking_time_is_exceed)) {
      deep_sleep_on(wake_up_timer_max_value, 1);
    }

    if (differance(current_milisecond, timer_ble_on) > BLE_advertising_duration) {
      ble.deinit();
    }  // Bluetooth on time duration control

    //Serial.printf(battery_status(adc_battery));
    //  if (maxlipo.cellPercent() < low_battery_percentage) {
    //    seperator = 1;
    //    LCD.print(0xCA00 + to_hex(constrain(maxlipo.cellPercent(), 0, 100)), 5000);


    //    deep_sleep_on(battery_control_period);

    //  }  // MCU go to sleep if battery is low(<%5)

  }  //Tasks tick rate control for slow tasks

  //}
}

void TaskDisplay()  //void* pvParameters)
{
  //while (1) {
  switch (mode) {
    case 0:  //Clock/normal mode
      LCD.print((to_hex(hour) << 8) + to_hex(minute));
      seperator = (current_milisecond % 1000 > 500);

      break;

    case 1:  //Parking mode
      LCD.print((to_hex(hour_park) << 8) + to_hex(minute_park));
      seperator = 1;

      break;

    case 2:  //Setting
      LCD.print(menu(), 200);

      break;

    case 3:  //After wake up(as clock mode)
      LCD.print((to_hex(hour) << 8) + to_hex(minute));
      seperator = (current_milisecond % 1000 > 500);

      break;
  }
  //}
}

unsigned long differance(unsigned long value1, unsigned long value2) {
  if (value1 >= value2) {
    return value1 - value2;
  } else {
    return value1 + UINT_MAX - value2 + 1;
  }
}

uint16_t to_hex(int value) {

  return value % 10 + ((value / 10 % 10) << 4) + ((value / 100 % 10) << 8) + ((value / 1000 % 10) << 12);
}

int menu() {

  static unsigned long timer_button_delay;
  static int jump_next;
  static int jump_prev;
  static int state;
  static int clock_setting_flag;
  static uint8_t hour_set;
  static uint8_t minute_set;
  static uint8_t duration_set;

  int value = 0;
  tm tstruct = rtc.getTimeStruct();

  uint32_t pin_state = ~GPIO.in;  // + ~((uint64_t)GPIO.in1<<32);
  uint32_t mask = (1 << pin_a) + (1 << pin_b) + (1 << pin_plus) + (1 << pin_minus);
  //pin_state &= 0b10111111111111;
  pin_state = pin_state & mask;


  //0b0000 0000 0000 0011 1100 0000 0000 0000
  if (differance(current_milisecond, timer_button_delay) > 200) {
    timer_last_motion = current_milisecond;
    timer_last_motionless_state = current_milisecond;

    if (pin_state) {
      timer_button_delay = current_milisecond;

      if (mode == 3) {
        mode = 0;
      }
    }
    if (pin_state & (1 << pin_plus)) {
      value = 1;

    } else if (pin_state & (1 << pin_minus)) {
      value = -1;

    } else if (pin_state & (1 << pin_a)) {
      state = jump_next;

    } else if (pin_state & (1 << pin_b)) {
      state = jump_prev;

    } else if (differance(current_milisecond, timer_last_entry_in_menu) > menu_time_out) {
      if (state != 41) {
        state = jump_prev;
      }
    }

    // Serial.printf("state:%d t:%d t1:%d Pin_state:%x mask:%x\n", state, differance(current_milisecond, timer_last_entry_in_menu), differance(current_milisecond, timer_button_delay), pin_state, mask);
  }

  switch (state) {
    case 0:
      state = 1;

      return 0xABA1;
      break;

    case 1:  //clock setting

      jump_next = 11;
      jump_prev = 59;
      state += value;
      seperator = 0;

      hour_set = hour;
      minute_set = minute;
      clock_setting_flag = 1;

      return 0xABA1;
      break;

    case 2:  //park duration
      jump_next = 21;
      jump_prev = 59;
      state += value;
      seperator = 0;

      duration_set = duration;

      return 0xABA2;
      break;

    case 3:  //return time to vehicle
      jump_next = 11;
      jump_prev = 59;
      state += value;
      seperator = 0;

      hour_set = hour;
      minute_set = minute;
      clock_setting_flag = 0;

      return 0xABA3;
      break;

    case 4:  //battery status

      jump_next = 41;
      jump_prev = 59;
      state += value;
      seperator = 0;

      return 0xABA4;
      break;

    case 5:
      jump_next = 55;
      jump_prev = 59;
      state += value;
      seperator = 0;

      return 0xABA5;
      break;

    case 6:
      if (!ota_started) { 
        ota_setup(); 
        delay(500);
        }
      ArduinoOTA.handle();
      return 0xABA6;
      break;

    case 7:
      state = 6;
      return 0xABA6;
      break;

    case 10:
      state = 11;
      return 0x11;
      break;

    case 11:

      hour_set += value;
      seperator = 1;

      if (hour_set >= 24) {
        hour_set = 0;
      }
      if (hour_set < 0) {
        hour_set = 23;
      }


      jump_next = 12;
      if (clock_setting_flag) {
        jump_prev = 51;
      } else {
        jump_prev = 53;
      }


      if (current_milisecond % 1000 > 500) {
        return (to_hex(hour_set) << 8) + to_hex(minute_set);
      } else {
        return 0xAA00 + to_hex(minute_set);
      }
      break;

    case 12:

      minute_set += value;
      seperator = 1;

      if (minute_set >= 60) {
        minute_set = 0;
      }
      if (minute_set < 0) {
        minute_set = 59;
      }

      jump_next = 11;
      if (clock_setting_flag) {
        jump_prev = 51;
      } else {
        jump_prev = 53;
      }

      if (current_milisecond % 1000 > 500) {
        return (to_hex(hour_set) << 8) + to_hex(minute_set);
      } else {
        return 0xAA + (to_hex(hour_set) << 8);
      }
      break;

    case 13:
      state = 12;
      return 0x13;
      break;

    case 21:

      jump_next = 21;
      jump_prev = 52;
      seperator = 0;

      duration_set += value;

      return to_hex(duration_set);
      break;

    case 41:

      jump_next = 41;
      jump_prev = 1;
      seperator = 0;

      //return 0xC000 + to_hex(constrain(maxlipo.cellPercent(), 0, 100));
      return to_hex(battery_status());
      break;

    case 51:
      jump_next = 1;
      jump_prev = 1;
      state = 1;
      mode = 0;

      tstruct.tm_min = minute_set;
      tstruct.tm_hour = hour_set - 1;
      tstruct.tm_sec = 0;
      rtc.setTimeStruct(tstruct);

      return 0x51;
      break;

    case 52:

      jump_next = 1;
      jump_prev = 1;
      state = 1;
      mode = 0;

      duration = duration_set;

      return 0x52;
      break;


    case 53:
      jump_next = 1;
      jump_prev = 1;
      state = 1;
      mode = 1;

      minute_park = minute_set;
      hour_park = hour_set;
      manuel_parked = 1;
      ble.init();
      ble.advertise_start();
      ble.ble_data(hour_park, minute_park);
      timer_ble_on = current_milisecond;

      return 0x53;
      break;

    case 55:

      jump_next = 1;
      jump_prev = 1;
      state = 1;
      mode = 0;

      LCD.print(0xAAAA);
      deep_sleep_on(wake_up_timer_max_value, 0);


      return 0x55;
      break;

    case 59:

      jump_next = 1;
      jump_prev = 1;
      state = 1;
      mode = 0;

      return 0x59;
      break;
  }
}


void deep_sleep_on(unsigned long long wakeup_timer, int ext_wake_up1) {
  LCD.print(0xAAAA);
  esp_sleep_config_gpio_isolate();
  esp_sleep_pd_config(ESP_PD_DOMAIN_XTAL, ESP_PD_OPTION_ON);

  esp_sleep_enable_timer_wakeup(wakeup_timer);

  if (ext_wake_up1) {
    //esp_sleep_enable_ext1_wakeup(0x80000003C00,ESP_EXT1_WAKEUP_ANY_LOW);
    //gpio_set_pull_mode(GPIO_NUM_13, GPIO_PULLUP_ONLY);

    gpio_deep_sleep_hold_en();
    gpio_hold_en((gpio_num_t)ext_power);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)intPin, 1);
    LCD.print(0xA1AA, 3000);
    //mpu.enableCycle(true);
  } else {
    //gpio_set_pull_mode(GPIO_NUM_3, GPIO_PULLUP_ONLY);
    gpio_hold_dis((gpio_num_t)ext_power);
    gpio_deep_sleep_hold_dis();
    esp_sleep_enable_ext0_wakeup((gpio_num_t)pin_a, 0);
    LCD.print(0xA0AA, 3000);
    mpu.setMagOpMode(AK8963_PWR_DOWN);
    // mpu.enableCycle(true);
    //mpu.enableSleep(true);
  }
  seperator = 0;
  LCD.print(0xAAAA);
  esp_deep_sleep_start();
}

void motionISR() {
  motion = true;
}

int battery_status() {
  int value = 0;
  for (int i = 0; i < 100; i++) {
    value += analogRead(adc_battery);
  }
  value /= 100;
  value = 78 + (value * 1.1224);  //85+(value*1.1257);//78+(value*1.1224);//88+(value*1.15);//4,2-3,1
  return value;                   //analogRead(pin);
}