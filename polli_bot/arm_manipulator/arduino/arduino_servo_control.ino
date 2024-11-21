#include <Servo.h>  // 서보 모터를 사용할 경우

Servo motor1, motor2, motor3, motor4, motor5;
const int motor1Pin = 6, motor2Pin = 9, motor3Pin = 10, motor4Pin = 11, motor5Pin = 3;

// end-effector pusher 동작 상태 변수
bool isPushing = false;

void setup() {
  Serial.begin(115200);

  motor1.attach(motor1Pin);
  motor2.attach(motor2Pin);
  motor3.attach(motor3Pin);
  motor4.attach(motor4Pin);
  motor5.attach(motor5Pin);

}

void loop() {
  if (Serial.available() > 0) {
    // 시리얼 데이터 읽기
    String input = Serial.readStringUntil('\n');  // '\n' 문자까지 읽기
    input.trim();  // 불필요한 공백 제거

    // 디버깅: 입력 데이터 출력
    // Serial.println("Received: " + input);

    // 시리얼 데이터를 ','로 분리
    float motorPositions[5];
    int index = 0;
    char *token = strtok(const_cast<char *>(input.c_str()), ",");
    while (token != nullptr && index < 5) {
      motorPositions[index] = atof(token);  // 문자열을 float로 변환
      token = strtok(nullptr, ",");
      index++;
    }

    // 5개의 모터 사용
    if (index == 5) {
      float motor1Position = motorPositions[0];  // joint_1
      float motor2Position = motorPositions[1];  // joint_2
      float motor3Position = motorPositions[2];  // joint_3
      float motor4Position = motorPositions[3];  // joint_4
      float motor5Position = motorPositions[4];  // joint_5

      // 서보 모터 위치 변환 및 제어
      int servo1 = mapToServoRange(motor1Position);
      int servo2 = mapToServoRange(motor2Position);
      int servo3 = mapToServoRange(motor3Position);
      int servo4 = mapToServoRange(motor4Position);

      motor1.write(servo1);
      motor2.write(servo2);
      motor3.write(servo3);
      motor4.write(servo4);

      // motor5의 동작 제어
      if (motor5Position > 0.08) {
        if (!isPushing) {
          motor5.write(165);       // push (정방향)
          delay(1100);             // 1.1초 대기
          motor5.write(90);        // 정지
          isPushing = true;       // 다음 동작은 pull
        } 
        else if(isPushing) {
          motor5.write(15);        // pull (역방향)
          delay(1080);             // 1.1초 대기
          motor5.write(90);        // 정지
          isPushing = false;        // 다음 동작은 push
        }
      }

      // 모터 위치를 한 줄로 전송
      // Serial.print(servo1 * 270 / 180);
      // Serial.print(",");
      // Serial.print(servo2 * 270 / 180);
      // Serial.print(",");
      // Serial.print(servo3 * 270 / 180);
      // Serial.print(",");
      // Serial.println(servo4 * 270 / 180);

    } else {
      // Serial.println("Invalid data received");
    }

    while (Serial.available() > 0) {
      Serial.read();  // 남아 있는 데이터를 모두 읽어 버퍼 비우기
    }
  }
}

// 서보 모터의 각도 범위(0-180)로 변환하는 함수
int mapToServoRange(float position) {
  float degree = position * (180.0 / 3.14159);
  return (int)(degree * (180.0 / 270.0));
}