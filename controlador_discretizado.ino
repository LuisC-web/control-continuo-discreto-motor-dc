#include "Wire.h"
#include "Adafruit_MCP4725.h"

Adafruit_MCP4725 dac;

// ---------------- Configuración del Controlador ----------------
// 1: Diferencia Atrás | 2: Diferencia Adelante | 3: Regla Trapezoidal 
// 4: Invarianza al Impulso | 5: Invarianza al Escalón
int METODO_DISCRETO = 5; 

const float Kp = 1.8;
const float Ki = 10.0;
const float T = 0.023; // Tiempo de muestreo: 20ms

// Variables de estado del controlador
float e_k = 0.0;
float e_k_1 = 0.0;
float u_k = 0.0;
float u_k_1 = 0.0;
unsigned long tiempoAnteriorControl = 0;

// ---------------- Pines ----------------
const int PIN_U_ANALOG = A7;  // Entrada del error acondicionado eac(t)
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10;
const int encoderA = 2;
const int encoderB = 3;

// ---------------- Parámetros del Sistema ----------------
const float V_MAX = 10.0;
const int PPR = 750;
const int M = 1;
const int N = 3700;

volatile uint32_t acumuladoDT = 0;
volatile uint8_t contadorFlancos = 0;
volatile unsigned long tiempoAnteriorA = 0;
volatile bool primerFlanco = true;
volatile float rad_seg = 0.0;
volatile unsigned long ultimoFlancoTiempo = 0;

void flancoA();

int calcularPWM_Experimental(float Voltaje) {
  float PWM = 47.598 + Voltaje * (-30.14 + Voltaje * (15.529 + Voltaje * (-2.5903 + Voltaje * 0.1495)));
  return constrain((int)PWM, 0, 255);
}

float calcularVoltaje_Velocidad(float Velocidad) {
  float Voltaje = 0.8895 * Velocidad + 0.839;
  if(Voltaje > 10.0) Voltaje = 10.0;
  else if (Voltaje < -10.0) Voltaje = -10.0;  
  return Voltaje;
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  if (!dac.begin(0x60)) Serial.println("Error: No se detectó el MCP4725.");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(PIN_U_ANALOG, INPUT);
  pinMode(encoderA, INPUT_PULLUP); pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), flancoA, RISING);
}

void loop() {
  // =========================================================================
  // 1. LECTURA DEL ERROR ACONDICIONADO: eac(t) = e(t)*0.2 + 2.5
  // =========================================================================
  int adc_raw = analogRead(PIN_U_ANALOG);
  float voltaje_pin = adc_raw * (5.0 / 1023.0); 
  
  // Despejar e(t) del error acondicionado -> e[k]
  e_k = (voltaje_pin - 2.6) * 5.0; 

  // =========================================================================
  // 2. CALCULAR u[k] SEGÚN EL MÉTODO ELEGIDO
  // =========================================================================
  switch (METODO_DISCRETO) {
    case 1: // Diferencia Atrás
      u_k = u_k_1 + (Kp + Ki * T) * e_k - Kp * e_k_1;
      break;
    case 2: // Diferencia Adelante
      u_k = u_k_1 + Kp * e_k + (Ki * T - Kp) * e_k_1;
      break;
    case 3: // Trapezoidal (Tustin) - Corregido el signo de resta
      u_k = u_k_1 + (Kp + (Ki * T) / 2.0) * e_k + ((Ki * T) / 2.0 - Kp) * e_k_1;
      break;
    case 4: // Invarianza al Impulso
      u_k = u_k_1 + (Kp * T + Ki * T) * e_k - Kp * T * e_k_1;
      break;
    case 5: // Invarianza al Escalón (ZOH)
      u_k = u_k_1 + Kp * e_k + (Ki * T - Kp) * e_k_1;
      break;
  }

  // =========================================================================
  // 3. ANTI-WINDUP Y SATURACIÓN FÍSICA (-10 a 10V)
  // =========================================================================
  u_k = constrain(u_k, -10.0, 10.0);

  // =========================================================================
  // 4. TRANSFORMAR u[k] EN ACCIÓN PWM
  // =========================================================================
  float u_abs = abs(u_k);
  int pwm_calculado = calcularPWM_Experimental(u_abs);

  if (pwm_calculado == 0) {
    digitalWrite(IN1, LOW); digitalWrite(IN2, LOW);
  } else if (u_k > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW); digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, pwm_calculado);

  // Actualizar variables para la siguiente iteración k+1
  e_k_1 = e_k;
  u_k_1 = u_k;

  // =========================================================================
  // 5. SALIDA DEL DAC Y TELEMETRÍA
  // =========================================================================
  noInterrupts();
  float velocidad_actual = rad_seg;
  interrupts();

  float VelocidadToVoltaje = calcularVoltaje_Velocidad(velocidad_actual);
  float voltaje_salida = VelocidadToVoltaje * 0.25 + 2.5;
  voltaje_salida = constrain(voltaje_salida, 0.0, 5.0);

  uint16_t dac_value = (uint16_t)((voltaje_salida / 5.0) * 4095.0);
  dac.setVoltage(dac_value, false);

  Serial.print("e[k]:"); Serial.print(e_k, 2);
  Serial.print(" | u[k]:"); Serial.print(u_k, 2);
  Serial.print(" | PWM:"); Serial.print(pwm_calculado);
  Serial.print(" | rad/s:"); Serial.print(velocidad_actual, 1);
  Serial.print(" | Voltaje entrada:"); Serial.print(voltaje_pin, 1);

  Serial.print(" | DAC_V:"); Serial.println(voltaje_salida, 2);

  // =========================================================================
  // 6. TIMEOUT SI EL MOTOR SE DETIENE
  // =========================================================================
  if (micros() - ultimoFlancoTiempo > 300000UL) {
    noInterrupts();
    rad_seg = 0.0;
    acumuladoDT = 0;
    contadorFlancos = 0;
    primerFlanco = true;
    interrupts();
  }

  // =========================================================================
  // 7. PERIODO DE MUESTREO (T)
  // =========================================================================
  // T está en segundos (0.02), lo multiplicamos por 1000 para ms
  delay(T * 1000.0); 
}

void flancoA() {
  unsigned long tiempoActual = micros();
  ultimoFlancoTiempo = tiempoActual; 
  if (primerFlanco) {
    tiempoAnteriorA = tiempoActual;
    primerFlanco = false;
    return;
  }
  acumuladoDT += tiempoActual - tiempoAnteriorA;
  tiempoAnteriorA = tiempoActual;
  contadorFlancos++;

  if (contadorFlancos >= M) {
    int sentido = (digitalRead(encoderB) == HIGH) ? 1 : -1;
    rad_seg = sentido * ((2.0f * M * PI * 1e6f) / ((float)PPR * (float)acumuladoDT));
    acumuladoDT = 0;
    contadorFlancos = 0;
  }
}