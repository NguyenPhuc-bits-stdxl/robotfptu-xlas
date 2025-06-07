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
#define HI_SPEED 0.95
#define MD_SPEED 0.55
#define LO_SPEED 0.12

// Góc servo
#define SER_OPEN  60
#define SER_CLOSE  180

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
PS2X ps2x;

// Đừng đụng vào mấy cái const này!
const double PWM_servo_const = 2.275555555555556;   // =2/(20/4096)/180, tính PWM 1 độ

double MOTOR_SPEED = MD_SPEED;
uint8_t LIFT_POWER = 100;
bool DRIVE_ENABLED = false;

int x, y;
int8_t y_neg, x_neg, curr_l, curr_r, last_l = 0, last_r = 0;
double ps2_pow, m_pow;
bool y_wait;

uint8_t X_JOY_CALIB = 128;
uint8_t Y_JOY_CALIB = 127;

bool PS2_Connect() {
  Serial.print("Connecting... ");

  int error = -1;
  // (clock, command, attention, data, pressure, rumble)
  error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, false, false);
  Serial.print(".");

  switch (error) {
    case 0:
      Serial.println(" Success");
      return true;
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
  return false;
}

// DÙNG CHO SERVO 180
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

// > 0 THUẬN, < 0 NGƯỢC
// Note: PWM motor dead zone: 0 .. 204.5
// Conversion % power to PWM: y = 39.2979798 x + 165.2020202
// KHI NÀO DÙNG BYPASSDRIVE
// Set 0 khẩn cấp, hoặc chỉnh bàn nâng
void Quay_Motor(uint8_t channel1, uint8_t channel2, double power, bool bypassdrivecheck) {
  if ((!DRIVE_ENABLED) && (!bypassdrivecheck)) {
    if (power != 0) Serial.println("Driving is disabled. Press Select to enable.");
    return;
  }

  // Cap max độ lớn
  if (power > 100) power = 100;
  if (power < -100) power = -100;

  bool thuan = (power > 0);
  uint8_t pwm_send = (int)(39.2979798 * abs(power) + 165.2020202);

  pwm.setPin(channel1, pwm_send *   thuan);
  pwm.setPin(channel2, pwm_send * (!thuan));

  if (power == 0) return;
  Serial.print(" >> Spinning motor at channel, % power: ");
  Serial.print(channel1);
  Serial.print(", ");
  Serial.println(power);
}

void Safe_Motor() {
  // Đơ để đảm bảo an toàn khi dò motor ngược chiều
  // Nghĩa là last đang tiến (+) mà nhận curr lệnh lùi (-) thì sẽ ra 1 số < 0, do đó nhân lại để dò đơ 50ms
  // FLAGS: BYPASS_DRIVE_MODE
  if ((last_l * curr_l < 0) || (last_r * curr_r < 0)) {
    Quay_Motor(MOL1, MOL2, 0, true);
    Quay_Motor(MOR1, MOR2, 0, true);
    delay(50);
  }
}

void Change_Speed(uint8_t speed) {
  MOTOR_SPEED = speed;

  Serial.print("Changed speed to :");
  Serial.println(speed);
}

void setup() {
  Serial.begin(115200);
  Serial.println("Press Select to toggle Drive");
  Serial.println("In need of calibrating joysticks, press Cross, default X=128 and Y=127");

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);
  Wire.begin();
  Wire.setClock(400000);

  // Reset 
  // FLAGS: BYPASS_DRIVE_MODE
  DRIVE_ENABLED = false;
  Quay_Servo_180(SER1, 0);
  Quay_Servo_180(SER2, 0);
  Quay_Motor(MOL1, MOL2, 0, true);
  Quay_Motor(MOR1, MOR2, 0, true);
  Quay_Motor(SLIFT1, SLIFT2, 0, true);
  
  // Liên tục kết nối cho đến khi thành công
  Serial.println("Initializing PS2...");
  while (!PS2_Connect()) {}
}

void loop() {
  ps2x.read_gamepad(false, false);

  // Chú ý: x_neg = -1 nghĩa là quẹo trái, 1 là quẹo phải
  y = Y_JOY_CALIB - ps2x.Analog(PSS_LY);
  x = X_JOY_CALIB - ps2x.Analog(PSS_RX);
  y_neg = 1 - 2*(y < 0);
  x_neg = 1 - 2*(x < 0);
  y = abs(y);
  x = abs(x);

  y_wait = false;

  // Báo mất kết nối
  if(x == -127 && y == -128) {
    Quay_Motor(MOL1, MOL2, 0, true);
    Quay_Motor(MOR1, MOR2, 0, true);
    Quay_Motor(SLIFT1, SLIFT2, 0, true);
    DRIVE_ENABLED = false;
    
    Serial.println("Disconnected. Reconnecting...");
    PS2_Connect();
  }

  // Ấn Select để bật/tắt Drive
  if (ps2x.Button(PSB_SELECT)) {
    DRIVE_ENABLED = !DRIVE_ENABLED;
    Serial.print("Toggled drive: ");
    if (DRIVE_ENABLED) Serial.println("ON");
    else Serial.println("OFF");
    delay(2000);
  }

  // Servo 1: PSB_L1, PSB_L2
  if (ps2x.Button(PSB_L2)) Quay_Servo_180(SER1, SER_CLOSE);   // càng đóng
  if (ps2x.Button(PSB_L1)) Quay_Servo_180(SER1, SER_OPEN);    // càng mở
  // Servo 2: PSB_R1, PSB_R2
  if (ps2x.Button(PSB_R2)) Quay_Servo_180(SER2, SER_CLOSE);   // càng đóng
  if (ps2x.Button(PSB_R1)) Quay_Servo_180(SER2, SER_OPEN);    // càng mở

  // Chỉnh tốc độ
  if (ps2x.Button(PSB_TRIANGLE)) Change_Speed(LO_SPEED);
  if (ps2x.Button(PSB_SQUARE))   Change_Speed(MD_SPEED);
  if (ps2x.Button(PSB_CIRCLE))   Change_Speed(HI_SPEED);

  // Calibrate lại joystick
  // Bỏ tay ra khỏi joystick trong khi calibrate
  if (ps2x.Button(PSB_CROSS)) {
    X_JOY_CALIB = ps2x.Analog(PSS_RX);
    Y_JOY_CALIB = ps2x.Analog(PSS_LY);

    Serial.print(X_JOY_CALIB);
    Serial.print(" and y = ");
    Serial.println(Y_JOY_CALIB);
    Serial.print("Calibrated to x = ");
    delay(2000);
  }

  // Nâng hạ
  // FLAGS: BYPASS_DRIVE_MODE
  if ((ps2x.Button(PSB_PAD_UP)) && (ps2x.Button(PSB_PAD_DOWN))) Serial.println("LIFT Input conflict!");
  else {
    if (ps2x.Button(PSB_PAD_UP)) {
      Quay_Motor(SLIFT1, SLIFT2, LIFT_POWER, true);
      delay(150);
      Quay_Motor(SLIFT1, SLIFT2, 0, true);
    }
    if (ps2x.Button(PSB_PAD_DOWN)) {
      Quay_Motor(SLIFT1, SLIFT2, -LIFT_POWER, true);
      delay(150);
      Quay_Motor(SLIFT1, SLIFT2, 0, true);
    }
  }

  // Chuyển từ gamepad sang % power: y = 0.7983870968 x - 1.39516129
  // Tránh stick-drift (-2..2)
  if (x <= 2 || y <= 2) {
    Quay_Motor(MOL1, MOL2, 0, true);
    Quay_Motor(MOR1, MOR2, 0, true);
  }
  if (y > 2) {
    curr_l = y_neg; curr_r = y_neg;
    ps2_pow = 0.7983870968*y - 1.39516129;
    m_pow = y_neg * MOTOR_SPEED * ps2_pow;

    Safe_Motor();
    y_wait = true;

    Quay_Motor(MOL1, MOL2, m_pow, false);
    Quay_Motor(MOR1, MOR2, m_pow, false);
  }
  if (x > 2) {
    curr_l = x_neg; curr_r = -x_neg;
    ps2_pow = 0.7983870968*x - 1.39516129;
    m_pow = MOTOR_SPEED * ps2_pow;

    Safe_Motor();
    if (y_wait) delay(50);

    Quay_Motor(MOL1, MOL2, curr_l * m_pow, false);
    Quay_Motor(MOR1, MOR2, curr_r * m_pow, false);
  }

  last_l = curr_l;
  last_r = curr_r;
  
  //Serial.print("Stick values: x, y = ");
  Serial.print(x);
  Serial.print(", ");
  Serial.println(y);
  delay(50);
}