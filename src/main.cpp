#include <Arduino.h>


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Adafruit_ST7789.h>
#include <Arduino_GFX.h>
#include <ESP32Servo.h>
#include <SPI.h>



#define CONTROL_BASE_PIN GPIO_NUM_32
#define SERVO_BASE_PIN GPIO_NUM_13
#define CONTROL_ELEVATION_PIN GPIO_NUM_33
#define SERVO_ELEVATION_PIN GPIO_NUM_12
#define CONTROL_SHOULDER_PIN GPIO_NUM_35
#define SERVO_SHOULDER_PIN GPIO_NUM_14
#define CONTROL_ELBOW_PIN GPIO_NUM_39
#define SERVO_ELBOW_PIN GPIO_NUM_27
#define CONTROL_WRIST_PIN GPIO_NUM_34
#define SERVO_WRIST_PIN GPIO_NUM_26
#define CONTROL_HAND_PIN GPIO_NUM_36
#define SERVO_HAND_PIN GPIO_NUM_25

#define SERVO_MAX_SIGNAL 2500
#define SERVO_MIN_SIGNAL 500
#define CONTROL_READ_DELAY 70
#define SERVO_SET_DELAY 5
#define SERVO_VELOCITY 1

#define RESET_STATE_PIN GPIO_NUM_5
#define TFT_SCL GPIO_NUM_18
#define TFT_SDA GPIO_NUM_23
#define TFT_RST GPIO_NUM_4
#define TFT_DC GPIO_NUM_2
#define TFT_CS GPIO_NUM_15


Servo servo_base;
Servo servo_elevation;
Servo servo_shoulder;
Servo servo_elbow;
Servo servo_wrist;
Servo servo_hand;

uint8_t servo_base_angle = 90;
uint8_t servo_elevation_angle = 60;
uint8_t servo_shoulder_angle = 90;
uint8_t servo_elbow_angle = 90;
uint8_t servo_wrist_angle = 90;
uint8_t servo_hand_angle = 90;

Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);



void read_control_base(void *parameters);
void set_servo_base(void *parameters);
void read_control_elevation(void *parameters);
void set_servo_elevation(void *parameters);
void read_control_shoulder(void *parameters);
void set_servo_shoulder(void *parameters);
void read_control_elbow(void *parameters);
void set_servo_elbow(void *parameters);
void read_control_wrist(void *parameters);
void set_servo_wrist(void *parameters);
void read_control_hand(void *parameters);
void set_servo_hand(void *parameters);
void check_reset_state(void *parameters);
void update_display(void *parameters);



void setup() {
  Serial.begin(115200);


  pinMode(CONTROL_BASE_PIN, INPUT);
  pinMode(CONTROL_ELEVATION_PIN, INPUT);
  pinMode(CONTROL_SHOULDER_PIN, INPUT);
  pinMode(CONTROL_ELBOW_PIN, INPUT);
  pinMode(CONTROL_WRIST_PIN, INPUT);
  pinMode(CONTROL_HAND_PIN, INPUT);
  pinMode(RESET_STATE_PIN, INPUT);


  display.init(135, 240, SPI_MODE0);
  display.setRotation(3);
  display.fillScreen(ST77XX_BLACK);


  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);


	servo_base.setPeriodHertz(50);
	servo_base.attach(SERVO_BASE_PIN, SERVO_MIN_SIGNAL, SERVO_MAX_SIGNAL);

  servo_elevation.setPeriodHertz(50);
  servo_elevation.attach(SERVO_ELEVATION_PIN, SERVO_MIN_SIGNAL, SERVO_MAX_SIGNAL);

  servo_shoulder.setPeriodHertz(50);
  servo_shoulder.attach(SERVO_SHOULDER_PIN, SERVO_MIN_SIGNAL, SERVO_MAX_SIGNAL);

  servo_elbow.setPeriodHertz(50);
  servo_elbow.attach(SERVO_ELBOW_PIN, SERVO_MIN_SIGNAL, SERVO_MAX_SIGNAL);

  servo_wrist.setPeriodHertz(50);
  servo_wrist.attach(SERVO_WRIST_PIN, SERVO_MIN_SIGNAL, SERVO_MAX_SIGNAL);

  servo_hand.setPeriodHertz(50);
  servo_hand.attach(SERVO_HAND_PIN, SERVO_MIN_SIGNAL, SERVO_MAX_SIGNAL);


  BaseType_t task_read_control_base = xTaskCreate(read_control_base, "read_control_base", (configMINIMAL_STACK_SIZE + 256), NULL, 2, NULL);
  if (task_read_control_base == pdFAIL) { Serial.println("Task read_control_base failed to create"); }

  BaseType_t task_set_servo_base = xTaskCreate(set_servo_base, "set_servo_base", (configMINIMAL_STACK_SIZE), NULL, 1, NULL);
  if (task_set_servo_base == pdFAIL) { Serial.println("Task set_servo_base failed to create"); }

  BaseType_t task_read_control_elevation = xTaskCreate(read_control_elevation, "read_control_elevation", (configMINIMAL_STACK_SIZE + 256), NULL, 2, NULL);
  if (task_read_control_elevation == pdFAIL) { Serial.println("Task read_control_elevation failed to create"); }

  BaseType_t task_set_servo_elevation = xTaskCreate(set_servo_elevation, "set_servo_elevation", (configMINIMAL_STACK_SIZE), NULL, 1, NULL);
  if (task_set_servo_elevation == pdFAIL) { Serial.println("Task set_servo_elevation failed to create"); }

  BaseType_t task_read_control_shoulder = xTaskCreate(read_control_shoulder, "read_control_shoulder", (configMINIMAL_STACK_SIZE + 256), NULL, 2, NULL);
  if (task_read_control_shoulder == pdFAIL) { Serial.println("Task read_control_shoulder failed to create"); }

  BaseType_t task_set_servo_shoulder = xTaskCreate(set_servo_shoulder, "set_servo_shoulder", (configMINIMAL_STACK_SIZE), NULL, 1, NULL);
  if (task_set_servo_shoulder == pdFAIL) { Serial.println("Task set_servo_shoulder failed to create"); }

  BaseType_t task_read_control_elbow = xTaskCreate(read_control_elbow, "read_control_elbow", (configMINIMAL_STACK_SIZE + 256), NULL, 2, NULL);
  if (task_read_control_elbow == pdFAIL) { Serial.println("Task read_control_elbow failed to create"); }

  BaseType_t task_set_servo_elbow = xTaskCreate(set_servo_elbow, "set_servo_elbow", (configMINIMAL_STACK_SIZE), NULL, 1, NULL);
  if (task_set_servo_elbow == pdFAIL) { Serial.println("Task set_servo_elbow failed to create"); }

  BaseType_t task_read_control_wrist = xTaskCreate(read_control_wrist, "read_control_wrist", (configMINIMAL_STACK_SIZE + 256), NULL, 2, NULL);
  if (task_read_control_wrist == pdFAIL) { Serial.println("Task read_control_wrist failed to create"); }

  BaseType_t task_set_servo_wrist = xTaskCreate(set_servo_wrist, "set_servo_wrist", (configMINIMAL_STACK_SIZE), NULL, 1, NULL);
  if (task_set_servo_wrist == pdFAIL) { Serial.println("Task set_servo_wrist failed to create"); }

  BaseType_t task_read_control_hand = xTaskCreate(read_control_hand, "read_control_hand", (configMINIMAL_STACK_SIZE + 256), NULL, 2, NULL);
  if (task_read_control_hand == pdFAIL) { Serial.println("Task read_control_hand failed to create"); }

  BaseType_t task_set_servo_hand = xTaskCreate(set_servo_hand, "set_servo_hand", (configMINIMAL_STACK_SIZE), NULL, 1, NULL);
  if (task_set_servo_hand == pdFAIL) { Serial.println("Task set_servo_hand failed to create"); }

  BaseType_t task_check_reset_state = xTaskCreate(check_reset_state, "check_reset_state", (configMINIMAL_STACK_SIZE + 1024), NULL, 1, NULL);
  if (task_check_reset_state == pdFAIL) { Serial.println("Task check_reset_state failed to create"); }

  BaseType_t task_update_display = xTaskCreate(update_display, "update_display", (configMINIMAL_STACK_SIZE + 1024), NULL, 1, NULL);
  if (task_update_display == pdFAIL) { Serial.println("Task update_display failed to create"); }
}



void loop() {
  vTaskDelay(10000);
}



void read_control_base(void *parameters) {
  while (true) {
    uint16_t control_base = analogRead(CONTROL_BASE_PIN);
    int16_t new_angle = servo_base_angle;

    if (control_base <= 1000) {
      new_angle = servo_base_angle - SERVO_VELOCITY;
    }

    if (control_base >= 3000) {
      new_angle = servo_base_angle + SERVO_VELOCITY;
    }
  
    if (new_angle < 0) { new_angle = 0; }
    if (new_angle > 180) { new_angle = 180; }
    servo_base_angle = new_angle;

    vTaskDelay(CONTROL_READ_DELAY);
  }
}



void set_servo_base(void *parameters) {
  while (true) {
    servo_base.write(servo_base_angle);
    vTaskDelay(SERVO_SET_DELAY);
  }
}



void read_control_elevation(void *parameters) {
  while (true) {
    uint16_t control_elevation = analogRead(CONTROL_ELEVATION_PIN);
    int16_t new_angle = servo_elevation_angle;

    if (control_elevation <= 1000) {
      new_angle = servo_elevation_angle - SERVO_VELOCITY;
    }

    if (control_elevation >= 3000) {
      new_angle = servo_elevation_angle + SERVO_VELOCITY;
    }
  
    if (new_angle < 0) { new_angle = 0; }
    if (new_angle > 180) { new_angle = 180; }
    servo_elevation_angle = new_angle;

    vTaskDelay(CONTROL_READ_DELAY);
  }
}



void set_servo_elevation(void *parameters) {
  while (true) {
    servo_elevation.write(servo_elevation_angle);
    vTaskDelay(SERVO_SET_DELAY);
  }
}



void read_control_shoulder(void *parameters) {
  while (true) {
    uint16_t control_shoulder = analogRead(CONTROL_SHOULDER_PIN);
    int16_t new_angle = servo_shoulder_angle;

    if (control_shoulder <= 1000) {
      new_angle = servo_shoulder_angle - SERVO_VELOCITY;
    }

    if (control_shoulder >= 3000) {
      new_angle = servo_shoulder_angle + SERVO_VELOCITY;
    }
  
    if (new_angle < 0) { new_angle = 0; }
    if (new_angle > 180) { new_angle = 180; }
    servo_shoulder_angle = new_angle;

    vTaskDelay(CONTROL_READ_DELAY);
  }
}



void set_servo_shoulder(void *parameters) {
  while (true) {
    servo_shoulder.write(servo_shoulder_angle);
    vTaskDelay(SERVO_SET_DELAY);
  }
}



void read_control_elbow(void *parameters) {
  while (true) {
    uint16_t control_elbow = analogRead(CONTROL_ELBOW_PIN);
    int16_t new_angle = servo_elbow_angle;

    if (control_elbow <= 1000) {
      new_angle = servo_elbow_angle - SERVO_VELOCITY;
    }

    if (control_elbow >= 3000) {
      new_angle = servo_elbow_angle + SERVO_VELOCITY;
    }
  
    if (new_angle < 0) { new_angle = 0; }
    if (new_angle > 180) { new_angle = 180; }
    servo_elbow_angle = new_angle;

    vTaskDelay(CONTROL_READ_DELAY);
  }
}



void set_servo_elbow(void *parameters) {
  while (true) {
    servo_elbow.write(servo_elbow_angle);
    vTaskDelay(SERVO_SET_DELAY);
  }
}



void read_control_wrist(void *parameters) {
  while (true) {
    uint16_t control_wrist = analogRead(CONTROL_WRIST_PIN);
    int16_t new_angle = servo_wrist_angle;

    if (control_wrist <= 1000) {
      new_angle = servo_wrist_angle - SERVO_VELOCITY;
    }

    if (control_wrist >= 3000) {
      new_angle = servo_wrist_angle + SERVO_VELOCITY;
    }
  
    if (new_angle < 0) { new_angle = 0; }
    if (new_angle > 180) { new_angle = 180; }
    servo_wrist_angle = new_angle;

    vTaskDelay(CONTROL_READ_DELAY);
  }
}



void set_servo_wrist(void *parameters) {
  while (true) {
    servo_wrist.write(servo_wrist_angle);
    vTaskDelay(SERVO_SET_DELAY);
  }
}



void read_control_hand(void *parameters) {
  while (true) {
    uint16_t control_hand = analogRead(CONTROL_HAND_PIN);
    int16_t new_angle = servo_hand_angle;

    if (control_hand <= 1000) {
      new_angle = servo_hand_angle - SERVO_VELOCITY;
    }

    if (control_hand >= 3000) {
      new_angle = servo_hand_angle + SERVO_VELOCITY;
    }
  
    if (new_angle < 0) { new_angle = 0; }
    if (new_angle > 180) { new_angle = 180; }
    servo_hand_angle = new_angle;

    vTaskDelay(CONTROL_READ_DELAY);
  }
}



void set_servo_hand(void *parameters) {
  while (true) {
    servo_hand.write(servo_hand_angle);
    vTaskDelay(SERVO_SET_DELAY);
  }
}



void check_reset_state(void *parameters) {
  while (true) {
    bool reset_state = digitalRead(RESET_STATE_PIN);
    if (reset_state == LOW) {
      servo_base_angle = 90;
      servo_elevation_angle = 60;
      servo_shoulder_angle = 90;
      servo_elbow_angle = 90;
      servo_wrist_angle = 90;
      servo_hand_angle = 90;
    }
    vTaskDelay(20);
  }
}



void update_display(void *parameters) {
  while (true) {
    display.fillScreen(ST77XX_BLACK);
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(ST77XX_WHITE);
    display.println("Robotic Arm\n");

    display.print("Base:           ");
    display.setTextColor(ST77XX_RED);
    display.println(servo_base_angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Elevation:      ");
    display.setTextColor(ST77XX_GREEN);
    display.println(servo_elevation_angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Shoulder:       ");
    display.setTextColor(ST77XX_BLUE);
    display.println(servo_shoulder_angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Elbow:          ");
    display.setTextColor(ST77XX_YELLOW);
    display.println(servo_elbow_angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Wrist:          ");
    display.setTextColor(ST77XX_CYAN);
    display.println(servo_wrist_angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Hand:           ");
    display.setTextColor(ST77XX_MAGENTA);
    display.println(servo_hand_angle);

    vTaskDelay(50);
  }
}
