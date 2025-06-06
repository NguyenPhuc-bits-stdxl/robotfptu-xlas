// Đấu nối:
// Channel 2: Servo 1
// Channel 3: Servo 2
// Channel 1: Motor 30RPM bàn nâng
// Channel 8 & 9: Motor trái
// Channel 10 & 11: Motor phải

#include <Arduino.h>
#include <Wire.h>
#include <PS2X_lib.h>
#include <Adafruit_PWMServoDriver.h>

#define PS2_DAT 12  // MISO
#define PS2_CMD 13  // MOSI
#define PS2_SEL 15  // SS
#define PS2_CLK 14  // SLK

#define SER1 2
#define SER2 3
#define SER3 4
#define SER4 5
#define SER5 6
#define SER6 7
#define MOL1 8
#define MOL2 9
#define MOR1 10
#define MOR2 11
#define SLIFT1 12
#define SLIFT2 13

// Các % tốc độ
#define HI_SPEED 100
#define MD_SPEED 50
#define LO_SPEED 30

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;

// Đừng đụng vào mấy cái const này!
const double PWM_servo_const = 2.275555555555556;   // =2/(20/4096)/180, tính PWM 1 độ
const double PWM_motor_const = 40.95;               // =4095/100, để lấy 1%
const double PS2_convert_perc = 0.0078125;          // =1/128 (KHÔNG *100 vì phải lấy %, vd. 0.25) Tính intensity của magnitude abs 128, để chạy smooth start

uint8_t MOTOR_SPEED = MD_SPEED;
uint8_t LIFT_SPEED = HI_SPEED;
bool DRIVE_ENABLED = false;

int x, y, y_chk;
double y_mag_percent, x_mag_percent;
int8_t y_neg, x_neg, curr_l, curr_r, last_l = 0, last_r = 0;

uint8_t X_JOY_CALIB = 127;
uint8_t Y_JOY_CALIB = 128;

void PS2_Connect() {
  Serial.println("Connecting");

  int error = -1;
  // (clock, command, attention, data, pressure, rumble)
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  Serial.print(".");

  switch (error) {
    case 0:
      Serial.println(" Success");
      break;
    case 1:
      Serial.println(" E Not Found");
      break;
    case 2:
      Serial.println(" E Can't send cmd");
      break;
    case 3:
      Serial.println(" E Can't enter Pressure");
      break;
  }
}

void Quay_Servo_180(uint8_t channel, uint8_t angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;

  double pwm_send = (angle * PWM_servo_const);
  pwm.setPWM(channel, 0, pwm_send);
  
  Serial.print(" >> Sending PWM to channel, with value: ");
  Serial.print(channel);
  Serial.print(", ");
  Serial.println(pwm_send);
}

// > 0 la thuan, < 0 la nguoc
void Quay_Motor(uint8_t channel1, uint8_t channel2, int8_t power) {
  if (!DRIVE_ENABLED) {
    Serial.println("Driving is disableed. Press Select to enable.");
    return;
  }

  if (power > 100) power = 100;
  if (power < -100) power = -100;

  bool thuan = (power > 0);
  uint8_t power_send = abs(power);
  pwm.setPin(channel1, power_send * PWM_motor_const *   thuan);
  pwm.setPin(channel2, power_send * PWM_motor_const * (!thuan));

  Serial.print(" >> Spinning motor at channel, % power: ");
  Serial.print(channel1);
  Serial.print(", ");
  Serial.println(power);
}

void Safe_Motor() {
  // Đơ 450ms nữa để đảm bảo an toàn khi dò motor ngược chiều
  // Nghĩa là last đang tiến (+) mà nhận curr lệnh lùi (-) thì sẽ ra 1 số < 0, do đó nhân lại để dò đơ
  if ((last_l * curr_l < 0) || (last_r * curr_r < 0)) {
    Quay_Motor(MOL1, MOL2, 0);
    Quay_Motor(MOR1, MOR2, 0);
    delay(450);
  }
}

void Change_Speed(uint8_t speed) {
  MOTOR_SPEED = speed;

  Serial.print("Changed speed to :");
  Serial.println(speed);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Press Start to connect");
  Serial.println("Then, press Select to toggle Drive");
  Serial.println("In need of calibrating joysticks, press Cross, default X=127 and Y=128");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.begin();
  Wire.setClock(400000);

  Quay_Servo_180(SER1, 0);
  Quay_Servo_180(SER2, 0);
}

void loop() {
  ps2x.read_gamepad(false, false);
  y = Y_JOY_CALIB - ps2x.Analog(PSS_LY);
  x = X_JOY_CALIB - ps2x.Analog(PSS_RX);
  y_mag_percent = abs(y) * PS2_convert_perc;
  x_mag_percent = abs(x) * PS2_convert_perc;

  y_chk = Y_JOY_CALIB - ps2x.Analog(PSS_RY);

  // Báo mất kết nối
  if(x == -1 && y_chk == 0) {
    Quay_Motor(MOL1, MOL2, 0);
    Quay_Motor(MOR1, MOR2, 0);
    
    Serial.println("Disconnected");
  }

  // Ấn Start để kết nối
  if (ps2x.Button(PSB_START)) PS2_Connect();
  
  // Ấn Select để bật/tắt Drive
  if (ps2x.Button(PSB_SELECT)) {
    DRIVE_ENABLED = !DRIVE_ENABLED;
    Serial.print("Toggled drive: ");
    if (DRIVE_ENABLED) Serial.println("ON");
    else Serial.println("OFF");
    delay(2000);
  }

  // Servo 1: PSB_L1, PSB_L2
  if (ps2x.Button(PSB_L2)) Quay_Servo_180(SER1, 0);   // càng đóng
  if (ps2x.Button(PSB_L1)) Quay_Servo_180(SER1, 75);  // càng mở
  // Servo 2: PSB_R1, PSB_R2
  if (ps2x.Button(PSB_R2)) Quay_Servo_180(SER2, 0);   // càng đóng
  if (ps2x.Button(PSB_R1)) Quay_Servo_180(SER2, 75);  // càng mở

  // Chỉnh tốc độ
  if (ps2x.Button(PSB_TRIANGLE)) Change_Speed(LO_SPEED);
  if (ps2x.Button(PSB_SQUARE))   Change_Speed(MD_SPEED);
  if (ps2x.Button(PSB_CIRCLE))   Change_Speed(HI_SPEED);

  // Calibrate lại joystick
  // Bỏ tay ra khỏi joystick trong khi calibrate
  if (ps2x.Button(PSB_CROSS)) {
    X_JOY_CALIB = ps2x.Analog(PSS_RX);
    Y_JOY_CALIB = ps2x.Analog(PSS_LY);
    delay(2000);

    Serial.print("Calibrated to x = ");
    Serial.print(X_JOY_CALIB);
    Serial.print(" and y = ");
    Serial.println(Y_JOY_CALIB);
  }
  
  // Nâng hạ
  if (ps2x.Button(PSB_PAD_UP) || ps2x.Button(PSB_PAD_RIGHT)) {
    Quay_Motor(SLIFT1, SLIFT2, LIFT_SPEED);
    delay(150);
  }
  if (ps2x.Button(PSB_PAD_DOWN) || ps2x.Button(PSB_PAD_LEFT)) {
    Quay_Motor(SLIFT1, SLIFT2, -LIFT_SPEED);
    delay(150);
  }

  // Di chuyển: Ưu tiên trục Oy
  // Chú ý: x_neg = -1 nghĩa là quẹo trái, 1 là quẹo phải
  y_neg = 1 - 2*(y < 0);
  x_neg = 1 - 2*(x < 0);

  // Không cần lấy y_mag và x_mag vì, chỉ cần xét thằng nào mạnh hơn thôi, dùng % cũng được
  if (y_mag_percent > x_mag_percent) {
    // ML PWM = yneg; MR PWM = yneg
    curr_l = y_neg; curr_r = y_neg;
    Safe_Motor();

    Quay_Motor(MOL1, MOL2, MOTOR_SPEED * y_neg * y_mag_percent);
    Quay_Motor(MOR1, MOR2, MOTOR_SPEED * y_neg * y_mag_percent);
  }
  else if (y_mag_percent < x_mag_percent) {
    // Quay trái (x_neg = -1): phải thuận (+), trái nghịch (-)
    // Quay phải (x_neg = +1): trái thuận (+), phải nghịch (-)

    // ML PWM = xneg; MR PWM = -xneg 
    curr_l = x_neg; curr_r = -x_neg;
    Safe_Motor();

    Quay_Motor(MOL1, MOL2, MOTOR_SPEED *   x_neg  * x_mag_percent);
    Quay_Motor(MOR1, MOR2, MOTOR_SPEED * -(x_neg) * x_mag_percent);
  }
  else {
    Quay_Motor(MOL1, MOL2, 0);
    Quay_Motor(MOR1, MOR2, 0);
  }

  last_l = curr_l;
  last_r = curr_r;
  
  delay(50);
}
