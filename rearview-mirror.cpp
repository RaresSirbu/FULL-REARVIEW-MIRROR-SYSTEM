#include <Arduino.h>
#include <Servo.h>
const byte x_key = A1;
const byte y_key = A0;
#define SWT 53
const byte MOTENR1 = 8;
const byte INR1 = A2;
const byte INR2 = A3;
const byte INR3 = A4;
const byte INR4 = A5;
const byte MOTENR2 = 9;
const byte MOTENL1 = 10;
const byte INL1 = 22;
const byte INL2 = 24;
const byte INL3 = 26;
const byte INL4 = 28;
const byte MOTENL2 = 11;
const byte SERVO_STANG = 12;
const byte SERVO_DREPT = 13;
const byte BUTON_RETRACTARE = 36;
#define TRIG_STANG 7
#define ECHO_STANG 6
#define TRIG_DREPT 5
#define ECHO_DREPT 4
#define BUTON_STANG 2
#define BUTON_DREPT 3
#define LED_STANG 30
#define LED_DREPT 32
#define BUZZER 34
const float DISTANTA_PRAG = 50.0;
const unsigned long INTERVAL_SENZOR = 100;
const int MARIME_FILTRU = 5;
const unsigned long INTERVAL_BUZZER = 300;
const unsigned long TIMEOUT_JOYSTICK = 10000;
const unsigned long DEBOUNCE_RETRACTARE = 300;
const unsigned long DEBOUNCE_SWT = 200;
const int JOYSTICK_DEADZONE_MIN = 400;
const int JOYSTICK_DEADZONE_MAX = 600;
const int JOYSTICK_ACTIV_THRESHOLD = 300;
const int UNFOLDED_POS = 90;
const int FOLDED_POS = 0;
const unsigned long FOLDING_TIME = 2000;
float distanteStang[MARIME_FILTRU];
float distanteDrept[MARIME_FILTRU];
int indexStang = 0;
int indexDrept = 0;
unsigned long ultimaCitireStang = 0;
unsigned long ultimaCitireDrept = 50;
float distantaStangFiltrata = 999.0;
float distantaDreptFiltrata = 999.0;
unsigned long ultimaSchimbareBuzzer = 0;
bool buzzerState = false;
int motor_speed_h;
int motor_speed_v;
bool oglindaActiva = true;
unsigned long ultimaActivitateJoystick = 0;
bool ultimulStadiuSWT = true;
unsigned long ultimaComutareOglinda = 0;
Servo servoStang;
Servo servoDrept;
bool oglinziRetractate = false;
bool retractareInProgres = false;
unsigned long startRetractare = 0;
int tintaRetractare = UNFOLDED_POS;
unsigned long ultimaApasareRetractare = 0;
enum StareLaterala {
  SIGUR,
  OBIECT_DETECTAT,
  AVERTISMENT_ACTIV
};
StareLaterala stareStang = SIGUR;
StareLaterala stareDrept = SIGUR;
float citesteUltrasonic(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long durata = pulseIn(echoPin, HIGH, 30000);
  if (durata == 0) return 999.0;
  return durata * 0.034 / 2;
}
float calculeazaMedie(float* arr) {
  float suma = 0;
  for (int i = 0; i < MARIME_FILTRU; i++) suma += arr[i];
  return suma / MARIME_FILTRU;
}
void actualizeazaStare(StareLaterala &stare, bool obiectDetectat, bool butonApasat) {
  switch (stare) {
    case SIGUR:
      if (obiectDetectat) stare = OBIECT_DETECTAT;
      break;
    case OBIECT_DETECTAT:
      if (!obiectDetectat) stare = SIGUR;
      else if (butonApasat) stare = AVERTISMENT_ACTIV;
      break;
    case AVERTISMENT_ACTIV:
      if (!obiectDetectat) stare = SIGUR;
      else if (!butonApasat) stare = OBIECT_DETECTAT;
      break;
  }
}
void opresteToateMotoarele() {
  digitalWrite(INR1, LOW); digitalWrite(INR2, LOW);
  digitalWrite(INR3, LOW); digitalWrite(INR4, LOW);
  digitalWrite(INL1, LOW); digitalWrite(INL2, LOW);
  digitalWrite(INL3, LOW); digitalWrite(INL4, LOW);
  analogWrite(MOTENR1, 0);
  analogWrite(MOTENR2, 0);
  analogWrite(MOTENL1, 0);
  analogWrite(MOTENL2, 0);
}

void controlMotors(int x_pos, int y_pos, byte pinH1, byte pinH2, byte pinV1, byte pinV2, byte enH, byte enV) {
  if (oglinziRetractate || retractareInProgres) {
    analogWrite(enH, 0);
    analogWrite(enV, 0);
    return;
  }
  int x_constrained = constrain(x_pos, 0, 1023);
  int y_constrained = constrain(y_pos, 0, 1023);
  if (x_constrained < JOYSTICK_DEADZONE_MIN) { 
    motor_speed_h = map(x_constrained, JOYSTICK_DEADZONE_MIN, 0, 0, 255);
    motor_speed_h = constrain(motor_speed_h, 0, 255);
    digitalWrite(pinH1, LOW); digitalWrite(pinH2, HIGH);
    analogWrite(enH, motor_speed_h);
  } else if (x_constrained < JOYSTICK_DEADZONE_MAX) { 
    digitalWrite(pinH1, LOW); digitalWrite(pinH2, LOW);
    analogWrite(enH, 0);
  } else { 
    motor_speed_h = map(x_constrained, JOYSTICK_DEADZONE_MAX, 1023, 0, 255);
    motor_speed_h = constrain(motor_speed_h, 0, 255);
    digitalWrite(pinH1, HIGH); digitalWrite(pinH2, LOW);
    analogWrite(enH, motor_speed_h);
  }
  if (y_constrained < JOYSTICK_DEADZONE_MIN) { 
    motor_speed_v = map(y_constrained, JOYSTICK_DEADZONE_MIN, 0, 0, 255);
    motor_speed_v = constrain(motor_speed_v, 0, 255);
    digitalWrite(pinV1, LOW); digitalWrite(pinV2, HIGH);
    analogWrite(enV, motor_speed_v);
  } else if (y_constrained < JOYSTICK_DEADZONE_MAX) { 
    digitalWrite(pinV1, LOW); digitalWrite(pinV2, LOW);
    analogWrite(enV, 0);
  } else { 
    motor_speed_v = map(y_constrained, JOYSTICK_DEADZONE_MAX, 1023, 0, 255);
    motor_speed_v = constrain(motor_speed_v, 0, 255);
    digitalWrite(pinV1, HIGH); digitalWrite(pinV2, LOW);
    analogWrite(enV, motor_speed_v);
  }
}
void setup() {
  pinMode(MOTENR1, OUTPUT); pinMode(INR1, OUTPUT); pinMode(INR2, OUTPUT);
  pinMode(INR3, OUTPUT); pinMode(INR4, OUTPUT); pinMode(MOTENR2, OUTPUT);
  pinMode(MOTENL1, OUTPUT); pinMode(INL1, OUTPUT); pinMode(INL2, OUTPUT);
  pinMode(INL3, OUTPUT); pinMode(INL4, OUTPUT); pinMode(MOTENL2, OUTPUT);
  pinMode(SWT, INPUT_PULLUP);
  pinMode(BUTON_RETRACTARE, INPUT_PULLUP);
  servoStang.attach(SERVO_STANG);
  servoDrept.attach(SERVO_DREPT);
  servoStang.write(UNFOLDED_POS);
  servoDrept.write(UNFOLDED_POS);
  pinMode(TRIG_STANG, OUTPUT); pinMode(ECHO_STANG, INPUT);
  pinMode(TRIG_DREPT, OUTPUT); pinMode(ECHO_DREPT, INPUT);
  pinMode(BUTON_STANG, INPUT_PULLUP); pinMode(BUTON_DREPT, INPUT_PULLUP);
  pinMode(LED_STANG, OUTPUT); pinMode(LED_DREPT, OUTPUT);
  pinMode(BUZZER, OUTPUT);
  for (int i = 0; i < MARIME_FILTRU; i++) {
    distanteStang[i] = 999.0;
    distanteDrept[i] = 999.0;
  }
  
  opresteToateMotoarele();
}
void loop() {
  unsigned long acum = millis();
  if (digitalRead(BUTON_RETRACTARE) == LOW && (acum - ultimaApasareRetractare) > DEBOUNCE_RETRACTARE) {
    ultimaApasareRetractare = acum;
    if (!retractareInProgres) {
      retractareInProgres = true;
      startRetractare = acum;
      tintaRetractare = oglinziRetractate ? UNFOLDED_POS : FOLDED_POS;
      if (!oglinziRetractate) opresteToateMotoarele();
    }
  }
  if (retractareInProgres) {
    unsigned long elapsed = acum - startRetractare;
    if (elapsed >= FOLDING_TIME) {
      servoStang.write(tintaRetractare);
      servoDrept.write(tintaRetractare);
      retractareInProgres = false;
      oglinziRetractate = !oglinziRetractate;
    } else {
      int progres = map(elapsed, 0, FOLDING_TIME, oglinziRetractate ? FOLDED_POS : UNFOLDED_POS, oglinziRetractate ? UNFOLDED_POS : FOLDED_POS);
      progres = constrain(progres, 0, 180);
      servoStang.write(progres);
      servoDrept.write(progres);
    }
  }
  if (acum - ultimaCitireStang >= INTERVAL_SENZOR) {
    ultimaCitireStang = acum;
    distanteStang[indexStang] = citesteUltrasonic(TRIG_STANG, ECHO_STANG);
    distantaStangFiltrata = calculeazaMedie(distanteStang);
    indexStang = (indexStang + 1) % MARIME_FILTRU;
  }
  
  if (acum - ultimaCitireDrept >= INTERVAL_SENZOR) {
    ultimaCitireDrept = acum;
    distanteDrept[indexDrept] = citesteUltrasonic(TRIG_DREPT, ECHO_DREPT);
    distantaDreptFiltrata = calculeazaMedie(distanteDrept);
    indexDrept = (indexDrept + 1) % MARIME_FILTRU;
  }
  actualizeazaStare(stareStang, distantaStangFiltrata < DISTANTA_PRAG, digitalRead(BUTON_STANG) == LOW);
  actualizeazaStare(stareDrept, distantaDreptFiltrata < DISTANTA_PRAG, digitalRead(BUTON_DREPT) == LOW);
  
  digitalWrite(LED_STANG, (stareStang != SIGUR) ? HIGH : LOW);
  digitalWrite(LED_DREPT, (stareDrept != SIGUR) ? HIGH : LOW);
  
  if (stareStang == AVERTISMENT_ACTIV || stareDrept == AVERTISMENT_ACTIV) {
    if (acum - ultimaSchimbareBuzzer >= INTERVAL_BUZZER) {
      ultimaSchimbareBuzzer = acum;
      buzzerState = !buzzerState;
      digitalWrite(BUZZER, buzzerState);
    }
  } else {
    if (buzzerState) {
      digitalWrite(BUZZER, LOW);
      buzzerState = false;
    }
  }
  if (!oglinziRetractate && !retractareInProgres) {
    int x_pos = analogRead(x_key);
    int y_pos = analogRead(y_key);
    bool stadiuCurentSWT = digitalRead(SWT);
    if (stadiuCurentSWT == LOW && ultimulStadiuSWT == HIGH && (acum - ultimaComutareOglinda) > DEBOUNCE_SWT) {
      oglindaActiva = !oglindaActiva;
      opresteToateMotoarele(); 
      ultimaActivitateJoystick = acum;
      ultimaComutareOglinda = acum;
    }
    ultimulStadiuSWT = stadiuCurentSWT;
    if (x_pos < JOYSTICK_ACTIV_THRESHOLD || x_pos > (1023 - JOYSTICK_ACTIV_THRESHOLD) || 
        y_pos < JOYSTICK_ACTIV_THRESHOLD || y_pos > (1023 - JOYSTICK_ACTIV_THRESHOLD)) {
      ultimaActivitateJoystick = acum;
    }    
    if (acum - ultimaActivitateJoystick < TIMEOUT_JOYSTICK) {
      if (oglindaActiva) {
        controlMotors(x_pos, y_pos, INR1, INR2, INR3, INR4, MOTENR1, MOTENR2);
      } else {
        controlMotors(x_pos, y_pos, INL1, INL2, INL3, INL4, MOTENL1, MOTENL2);
      }
    } else {
      opresteToateMotoarele();
    }
  }
}