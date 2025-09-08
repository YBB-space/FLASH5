#include "HX711.h"

#define Pressure_PIN A0 // 압력센서 핀
#define CLK   2 // 로드셀 핀1
#define DOUT  3 // 로드셀 핀2
#define LED_PIN 7 // LED 핀
#define RELAY_PIN 8 // 릴레이 핀
#define IGNITION_INPUT_PIN 9 // 외부 점화 입력 핀
#define ignition_time   10 // 점화시간 입력

float thrust_cal_factor = 6510;
HX711 hx711;

bool ignition = false;
unsigned long ignitionStart = 0;
unsigned long loopStartTime;

int IFP_mode = 0;

void setup() {
  Serial.begin(19200);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);
  pinMode(IGNITION_INPUT_PIN, INPUT); // 외부 점화 신호 입력핀

  hx711.begin(DOUT, CLK);
  hx711.set_gain(128);
  hx711.set_scale(thrust_cal_factor);
  hx711.tare();

  loopStartTime = millis();
}

void loop() {
  // 센서값 측정
  hx711.set_scale(thrust_cal_factor);
  float thrust = hx711.get_units(1);

  int psensorValue = analogRead(Pressure_PIN);
  float pressure = psensorValue * (10.0 / 1023.0);

  unsigned long loopTime = millis() - loopStartTime;

  int ignition_signal = digitalRead(RELAY_PIN) == HIGH ? 1 : 0;
  int ignition_input_signal = digitalRead(IGNITION_INPUT_PIN) == HIGH ? 1 : 0;

  // IFP 모드가 켜져 있고 외부 점화 신호가 들어오면 릴레이 강제 ON
  if (IFP_mode == 0 && ignition_input_signal == 1) {
    digitalWrite(RELAY_PIN, HIGH);
    digitalWrite(LED_PIN, HIGH);
  }
  // 그 외엔 ignition 수동 점화 처리
  else if (ignition) {
    if (millis() - ignitionStart <= ignition_time * 1000) {
      digitalWrite(RELAY_PIN, HIGH);
      digitalWrite(LED_PIN, HIGH);
    } else {
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      ignition = false;
    }
  }
  // ignition도 없고 IFP도 아닌 경우 → 릴레이 OFF
  else {
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_PIN, LOW);
  }

  // 시리얼 출력
  Serial.println(String(thrust) + ", " + String(pressure) + ", " + String(psensorValue) + ", " + 
                 String(loopTime) + ", " + String(ignition_signal) + ", " + 
                 String(ignition_input_signal) + ", " + String(IFP_mode));

  // 명령 수신
  if (Serial.available()) {
    String cmd = Serial.readStringUntil('\n');
    cmd.trim();

    if (cmd == "ignition") {
      ignitionStart = millis();
      ignition = true;
    } else if (cmd == "t") {
      hx711.tare();
    } else if (cmd == "+") {
      thrust_cal_factor += 10;
    } else if (cmd == "-") {
      thrust_cal_factor -= 10;
    } else if (cmd == "IFP") {
      digitalWrite(RELAY_PIN, LOW);
      digitalWrite(LED_PIN, LOW);
      ignition = false;
      Serial.println("IFP: 릴레이 강제 차단됨.");
    } else if (cmd == "IFP_ON") {
      IFP_mode = 1;
      Serial.println("IFP 모드 활성화됨.");
    } else if (cmd == "IFP_OFF") {
      IFP_mode = 0;
      Serial.println("IFP 모드 비활성화됨.");
    } else {
      Serial.println("알 수 없는 명령어입니다.");
    }
  }
}