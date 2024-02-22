#include <Arduino.h>


#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <Adafruit_ST7789.h>
#include <Arduino_GFX.h>
#include <ESP32Servo.h>
#include <SPI.h>



#define CONTROL_Y_1 GPIO_NUM_33
#define CONTROL_X_1 GPIO_NUM_32
#define CONTROL_Y_2 GPIO_NUM_35
#define CONTROL_X_2 GPIO_NUM_34
#define CONTROL_Y_3 GPIO_NUM_39
#define CONTROL_X_3 GPIO_NUM_36

#define SERVO_BASE_PIN GPIO_NUM_13
#define SERVO_SHOULDER_PIN GPIO_NUM_12
#define SERVO_ELBOW_PIN GPIO_NUM_14
#define SERVO_WRIST_PIN GPIO_NUM_27
#define SERVO_HAND_PIN GPIO_NUM_26
#define SERVO_CLAW_PIN GPIO_NUM_25

#define RESET_BUTTON_PIN GPIO_NUM_5
#define MORE_SPEED_BUTTON_PIN GPIO_NUM_17
#define LESS_SPEED_BUTTON_PIN GPIO_NUM_19

#define TFT_SCL GPIO_NUM_18
#define TFT_SDA GPIO_NUM_23
#define TFT_RST GPIO_NUM_4
#define TFT_DC GPIO_NUM_2
#define TFT_CS GPIO_NUM_15

#define SERVO_MAX_SIGNAL 2500
#define SERVO_MIN_SIGNAL 500
#define READ_CONTROL_DELAY 70
#define SET_SERVO_DELAY 5



typedef struct {
  Servo servo;
  uint8_t angle;
  uint16_t control_pin;
  uint16_t servo_pin;
  uint8_t max_angle;
  uint8_t min_angle;
  bool inverted;
  uint8_t velocity;


  void setup() {
    pinMode(control_pin, INPUT);

    servo.setPeriodHertz(50);
    servo.attach(servo_pin, SERVO_MIN_SIGNAL, SERVO_MAX_SIGNAL);
    servo.write(angle);
  }

  void updateServo() {
    servo.write(angle);
  }
} ServoParameters;



ServoParameters base = {Servo(), 90, CONTROL_X_1, SERVO_BASE_PIN, 180, 0, false, 1};
ServoParameters shoulder = {Servo(), 40, CONTROL_Y_1, SERVO_SHOULDER_PIN, 180, 0, true, 1};
ServoParameters elbow = {Servo(), 110, CONTROL_Y_3, SERVO_ELBOW_PIN, 180, 0, true, 1};
ServoParameters wrist = {Servo(), 135, CONTROL_Y_2, SERVO_WRIST_PIN, 180, 0, false, 1};
ServoParameters hand = {Servo(), 90, CONTROL_X_2, SERVO_HAND_PIN, 180, 0, false, 1};
ServoParameters claw = {Servo(), 90, CONTROL_X_3, SERVO_CLAW_PIN, 160, 70, true, 5};
Adafruit_ST7789 display = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);



void read_control(void *pvParameters);
void set_servo(void *pvParameters);
void check_reset_button(void *pvParameters);
void check_more_speed_button(void *pvParameters);
void check_less_speed_button(void *pvParameters);
void update_display(void *pvParameters);
void clear_display(void *pvParameters);



void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial2.end();


  display.init(135, 240, SPI_MODE0);
  display.setRotation(3);
  display.fillScreen(ST77XX_BLACK);


  ESP32PWM::allocateTimer(0);
	ESP32PWM::allocateTimer(1);
	ESP32PWM::allocateTimer(2);
	ESP32PWM::allocateTimer(3);


  base.setup();
  shoulder.setup();
  elbow.setup();
  wrist.setup();
  hand.setup();
  claw.setup();


  BaseType_t task_read_control_base = xTaskCreate(read_control, "read_control_base", (configMINIMAL_STACK_SIZE + 256), &base, 2, NULL);
  BaseType_t task_read_control_shoulder = xTaskCreate(read_control, "read_control_shoulder", (configMINIMAL_STACK_SIZE + 256), &shoulder, 2, NULL);
  BaseType_t task_read_control_wrist = xTaskCreate(read_control, "read_control_wrist", (configMINIMAL_STACK_SIZE + 256), &wrist, 2, NULL);
  BaseType_t task_read_control_elbow = xTaskCreate(read_control, "read_control_elbow", (configMINIMAL_STACK_SIZE + 256), &elbow, 2, NULL);
  BaseType_t task_read_control_hand = xTaskCreate(read_control, "read_control_hand", (configMINIMAL_STACK_SIZE + 256), &hand, 2, NULL);
  BaseType_t task_read_control_claw = xTaskCreate(read_control, "read_control_claw", (configMINIMAL_STACK_SIZE + 256), &claw, 2, NULL);
  if (task_read_control_base == pdFAIL) { Serial.println("Task read_control_base failed to create"); }
  if (task_read_control_shoulder == pdFAIL) { Serial.println("Task read_control_shoulder failed to create"); }
  if (task_read_control_elbow == pdFAIL) { Serial.println("Task read_control_elbow failed to create"); }
  if (task_read_control_wrist == pdFAIL) { Serial.println("Task read_control_wrist failed to create"); }
  if (task_read_control_hand == pdFAIL) { Serial.println("Task read_control_hand failed to create"); }
  if (task_read_control_claw == pdFAIL) { Serial.println("Task read_control_hand failed to create"); }


  BaseType_t task_set_servo_base = xTaskCreate(set_servo, "set_servo_base", (configMINIMAL_STACK_SIZE), &base, 1, NULL);
  BaseType_t task_set_servo_shoulder = xTaskCreate(set_servo, "set_servo_shoulder", (configMINIMAL_STACK_SIZE), &shoulder, 1, NULL);
  BaseType_t task_set_servo_elbow = xTaskCreate(set_servo, "set_servo_elbow", (configMINIMAL_STACK_SIZE), &elbow, 1, NULL);
  BaseType_t task_set_servo_wrist = xTaskCreate(set_servo, "set_servo_wrist", (configMINIMAL_STACK_SIZE), &wrist, 1, NULL);
  BaseType_t task_set_servo_hand = xTaskCreate(set_servo, "set_servo_hand", (configMINIMAL_STACK_SIZE), &hand, 1, NULL);
  BaseType_t task_set_servo_claw = xTaskCreate(set_servo, "set_servo_claw", (configMINIMAL_STACK_SIZE), &claw, 1, NULL);
  if (task_set_servo_base == pdFAIL) { Serial.println("Task set_servo_base failed to create"); }
  if (task_set_servo_shoulder == pdFAIL) { Serial.println("Task set_servo_shoulder failed to create"); }
  if (task_set_servo_elbow == pdFAIL) { Serial.println("Task set_servo_elbow failed to create"); }
  if (task_set_servo_wrist == pdFAIL) { Serial.println("Task set_servo_wrist failed to create"); }
  if (task_set_servo_hand == pdFAIL) { Serial.println("Task set_servo_hand failed to create"); }
  if (task_set_servo_claw == pdFAIL) { Serial.println("Task set_servo_claw failed to create"); }


  BaseType_t task_check_reset_button = xTaskCreate(check_reset_button, "check_reset_button", (configMINIMAL_STACK_SIZE), NULL, 3, NULL);
  BaseType_t task_check_more_speed_button = xTaskCreate(check_more_speed_button, "check_more_speed_button", (configMINIMAL_STACK_SIZE), NULL, 3, NULL);
  BaseType_t task_check_less_speed_button = xTaskCreate(check_less_speed_button, "check_less_speed_button", (configMINIMAL_STACK_SIZE), NULL, 3, NULL);
  if (task_check_reset_button == pdFAIL) { Serial.println("Task check_reset_button failed to create"); }
  if (task_check_more_speed_button == pdFAIL) { Serial.println("Task check_more_speed_button failed to create"); }
  if (task_check_less_speed_button == pdFAIL) { Serial.println("Task check_less_speed_button failed to create"); }


  BaseType_t task_update_display = xTaskCreate(update_display, "update_display", (configMINIMAL_STACK_SIZE + 1024), NULL, 4, NULL);
  BaseType_t task_clear_display = xTaskCreate(clear_display, "clear_display", (configMINIMAL_STACK_SIZE), NULL, 4, NULL);
  if (task_update_display == pdFAIL) { Serial.println("Task update_display failed to create"); }
  if (task_clear_display == pdFAIL) { Serial.println("Task clear_display failed to create"); }
}



void loop() {
  vTaskDelay(10000);
}



void read_control(void *pvParameters) {
  ServoParameters *parameters = (ServoParameters *)pvParameters;

  while (true) {
    uint16_t control_value = analogRead(parameters->control_pin);
    int16_t new_angle = parameters->angle;

    if (control_value <= 1000) {
      new_angle -= parameters->inverted ? parameters->velocity : -parameters->velocity;
    }

    if (control_value >= 3000) {
      new_angle += parameters->inverted ? parameters->velocity : -parameters->velocity;
    }
  
    if (new_angle < parameters->min_angle) { new_angle = parameters->min_angle; }
    if (new_angle > parameters->max_angle) { new_angle = parameters->max_angle; }
    
    parameters->angle = new_angle;

    vTaskDelay(READ_CONTROL_DELAY);
  }
}



void set_servo(void *pvParameters) {
  ServoParameters *parameters = (ServoParameters *)pvParameters;

  while (true) {
    parameters->updateServo();
    vTaskDelay(SET_SERVO_DELAY);
  }
}



void check_reset_button(void *pvParameters) {
  while (true) {
    bool reset_state = digitalRead(RESET_BUTTON_PIN);
    if (reset_state == LOW) {
      Serial.println("Resettting");
      base.angle = 90;
      shoulder.angle = 40;
      elbow.angle = 110;
      wrist.angle = 135;
      hand.angle = 90;
      claw.angle = 90;
    }
    vTaskDelay(20);
  }
}



void check_more_speed_button(void *pvParameters) {
  while (true) {
    bool more_speed_state = digitalRead(MORE_SPEED_BUTTON_PIN);
    if (more_speed_state == HIGH) {
      Serial.println("More speed");
      shoulder.velocity = 5;
      elbow.velocity = 5;
      wrist.velocity = 5;
      hand.velocity = 5;
    }
    vTaskDelay(20);
  }
}



void check_less_speed_button(void *pvParameters) {
  while (true) {
    bool less_speed_state = digitalRead(LESS_SPEED_BUTTON_PIN);
    if (less_speed_state == HIGH) {
      Serial.println("Less speed");
      shoulder.velocity = 1;
      elbow.velocity = 1;
      wrist.velocity = 1;
      hand.velocity = 1;
    }
    vTaskDelay(20);
  }
}



void update_display(void *pvParameters) {
  while (true) {
    display.setCursor(0, 0);
    display.setTextSize(2);
    display.setTextColor(ST77XX_WHITE);
    display.println("Robotic Arm\n");

    display.print("Base:           ");
    display.setTextColor(ST77XX_RED);
    display.println(base.angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Shoulder:       ");
    display.setTextColor(ST77XX_GREEN);
    display.println(shoulder.angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Elbow:          ");
    display.setTextColor(ST77XX_BLUE);
    display.println(elbow.angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Wrist:          ");
    display.setTextColor(ST77XX_CYAN);
    display.println(wrist.angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Hand:           ");
    display.setTextColor(ST77XX_YELLOW);
    display.println(hand.angle);

    display.setTextColor(ST77XX_WHITE);
    display.print("Claw:           ");
    display.setTextColor(ST77XX_MAGENTA);
    display.println(claw.angle);

    vTaskDelay(10);
  }
}



void clear_display(void *pvParameters) {
  while (true) {
    display.fillScreen(ST77XX_BLACK);
    vTaskDelay(20);
  }
}