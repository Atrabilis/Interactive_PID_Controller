#include <Arduino.h>

#define ENCA 2
#define ENCB 3
#define PWM 5
#define IN2 6
#define IN1 7

volatile int posi = 0;
long prevT = 0;
float eprev = 0;
float eintegral = 0;
const int maxIntegral = 20;
bool antiWindupEnabled = true;

float kp = 5; // Valores iniciales para los parámetros PID
float ki = 3;
float kd = .5;
int target = 300; // Valor inicial para la referencia

String inputString = "";         // Una cadena para almacenar los mensajes entrantes
bool stringComplete = false;     // ¿Se ha recibido toda la cadena?
int target = 300;                  // Posición de referencia

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);

  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);

  Serial.println("Ready for PID parameters.");
  inputString.reserve(200);
}

void loop() {
  if (Serial.available() > 0) {
    String receivedData = Serial.readStringUntil('\n');
    parseAndUpdatePID(receivedData);
  }

  long currT = micros();
  float deltaT = ((float)(currT - prevT)) / (1e6);
  prevT = currT;

  int pos = 0;
  noInterrupts();
  pos = posi;
  interrupts();

  int e = pos - target;

  float dedt = (e - eprev) / deltaT;

  eintegral += e * deltaT;
  if (antiWindupEnabled) {
    if (eintegral > maxIntegral) {
      eintegral = maxIntegral;
    } else if (eintegral < -maxIntegral) {
      eintegral = -maxIntegral;
    }
  }

  float u = kp * e + kd * dedt + ki * eintegral;

  float pwr = fabs(u);
  if (pwr > 255) {
    pwr = 255;
  }

  int dir = (u < 0) ? -1 : 1;

  setMotor(dir, pwr, PWM, IN1, IN2);

  eprev = e;
  if (stringComplete) {
    // Parsear la cadena recibida aquí
    if(inputString.startsWith("T")) {
      target = inputString.substring(1).toInt();
      Serial.print("Target updated: ");
      Serial.println(target);
    } else {
      // Parsear otros comandos si los hay
    }
    // Limpiar la cadena para la próxima lectura
    inputString = "";
    stringComplete = false;
  }
  Serial.print(target);
  Serial.print(" ");
  Serial.print(pos);
  Serial.print(" ");
  Serial.print(e);
  Serial.print(" ");
  Serial.print(dedt);
  Serial.print(" ");
  Serial.println(eintegral);
}

void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }
}

void readEncoder() {
  int b = digitalRead(ENCB);
  posi += (b > 0) ? 1 : -1;
}

void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    inputString += inChar;
    if (inChar == '\n') {
      stringComplete = true;
    }
  }
}

void parseAndUpdatePID(String data) {
  char inputChars[data.length() + 1];
  data.toCharArray(inputChars, sizeof(inputChars));
  char* ptr = NULL;
  float newPidValues[3] = {kp, ki, kd}; // Inicializa con valores actuales
  bool pidValuesReceived[3] = {false, false, false}; // Monitorea qué valores se recibieron
  bool windupCommandReceived = false;
  bool referenceReceived = false; // Flag para el nuevo valor de referencia

  ptr = strtok(inputChars, ",");
  while (ptr != NULL) {
    String pidValue(ptr);
    if (pidValue.startsWith("P") && pidValue.length() > 1) {
      newPidValues[0] = pidValue.substring(1).toFloat();
      pidValuesReceived[0] = true;
    } else if (pidValue.startsWith("I") && pidValue.length() > 1) {
      newPidValues[1] = pidValue.substring(1).toFloat();
      pidValuesReceived[1] = true;
    } else if (pidValue.startsWith("D") && pidValue.length() > 1) {
      newPidValues[2] = pidValue.substring(1).toFloat();
      pidValuesReceived[2] = true;
    } else if (pidValue.startsWith("W")) {
      antiWindupEnabled = pidValue.substring(1).toInt() == 1;
      windupCommandReceived = true;
    } else if (pidValue.startsWith("R") && pidValue.length() > 1) { // Parsea el nuevo valor de referencia
      target = pidValue.substring(1).toInt();
      referenceReceived = true;
    }
    ptr = strtok(NULL, ",");
  }

  // Actualiza los valores si se recibieron nuevos
  if (pidValuesReceived[0] || pidValuesReceived[1] || pidValuesReceived[2]) {
    kp = newPidValues[0];
    ki = newPidValues[1];
    kd = newPidValues[2];
    Serial.print("PID updated - P: "); Serial.print(kp);
    Serial.print(", I: "); Serial.print(ki);
    Serial.print(", D: "); Serial.println(kd);
  }

  if (windupCommandReceived) {
    Serial.print("Anti-windup command received - Status: "); Serial.println(antiWindupEnabled ? "Enabled" : "Disabled");
  }

  if (referenceReceived) { // Si se recibió un nuevo valor de referencia, confírmalo
    Serial.print("New reference received: ");
    Serial.println(target);
  }
}
