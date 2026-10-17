#include "Wire.h"
#include "Adafruit_MCP4725.h"

Adafruit_MCP4725 dac;

// ---------------- Pines ----------------
const int PIN_U_ANALOG = A7;  // Entrada de u(t) acondicionada desde el Op-Amp
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10;  // PWM al L298N
const int encoderA = 2;
const int encoderB = 3;

// ---------------- Parámetros de Calibración ----------------
const float V_MAX = 10.0;     // Voltaje máximo de control (+/- 10V)
const int PPR = 750;   // Pulsos por vuelta del encoder
const int M = 1;
const int N = 3700;
// ---------------- Variables del Encoder ----------------
volatile uint32_t acumuladoDT = 0;
volatile uint8_t contadorFlancos = 0;
volatile unsigned long tiempoAnteriorA = 0;
volatile int indexA = 0;
volatile bool primerFlanco = true;
volatile uint16_t deltaT[N];
volatile float rad_seg = 0.0;
volatile unsigned long ultimoFlancoTiempo = 0;

void flancoA();

// ---------------- Función de Polinómica Experimental PWM = f(V) ----------------
int calcularPWM_Experimental(float Voltaje) {
  // Evaluación eficiente por la regla de Horner:
  // PWM = 47.598 + V * (-30.14 + V * (15.529 + V * (-2.5903 + V * 0.1495)))
  float PWM = 47.598 + Voltaje * (-30.14 + Voltaje * (15.529 + Voltaje * (-2.5903 + Voltaje * 0.1495)));

  // Opcional: limitar los valores y convertir a entero para el L298N
  return constrain((int)PWM, 0, 255);
}

float calcularVoltaje_Velocidad(float Velocidad) {
  // Evaluación eficiente por la regla de Horner:
  // PWM = 47.598 + V * (-30.14 + V * (15.529 + V * (-2.5903 + V * 0.1495)))
  float Voltaje = 0.8895 * Velocidad + 0.839;
  if(V_MAX>10){
    Voltaje=10;
  }else if (-V_MAX<-10) {
    Voltaje=-10;  
  }
  return Voltaje;
}
void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!dac.begin(0x60)) {
    Serial.println("Error: No se detectó el MCP4725.");
  }

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(PIN_U_ANALOG, INPUT);
  pinMode(encoderA, INPUT_PULLUP);
  pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), flancoA, RISING);
}

void loop() {
  // =========================================================================
  // 1. LECTURA DE u(t) Y CONVERSIÓN CON LA FUNCIÓN EXPERIMENTAL
  // =========================================================================
    int adc_raw = analogRead(PIN_U_ANALOG);  // Lee valor de 0 a 1023
      
      // 1. Convertir la lectura cruda al voltaje FÍSICO real del pin (0.0V a 5.0V)
  float voltaje_pin = adc_raw * (5.0 / 1023.0); 

  // 2. Convertir el voltaje físico a tu escala de Control (-10V a +10V)
  // Como 2.5V representa 0V, le restamos 2.5 para centrarlo (queda de -2.5 a +2.5).
  // Luego lo multiplicamos por 4 para expandirlo al rango de -10 a +10.
  float u_control = (voltaje_pin - 2.4) * 5.0;
  constrain(u_control, -10, 10);


  float u_abs = abs(u_control);
  int pwm_calculado = calcularPWM_Experimental(u_abs);
  // Asignación de dirección y actuación sobre el Puente H
  if (pwm_calculado == 0) {
    digitalWrite(IN1, LOW);




    digitalWrite(IN2, LOW);
  } else if (u_control > 0) {
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, pwm_calculado);
  // =========================================================================
  // 2. TIMEOUT SI EL MOTOR SE DETIENE
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
  // 3. GENERACIÓN DE REALIMENTACIÓN y(t) AL DAC MCP4725
  // =========================================================================
  noInterrupts();
  interrupts();

  // Normalización de RPM a [-1.0, 1.0] respecto a la velocidad máxima


  // Mapear de [-1.0, 1.0] a rango de 12 bits del DAC (0 a 4095)
  // 0 RPM -> 2047 (~2.5V de salida)
  float VelocidadToVoltaje= calcularVoltaje_Velocidad(rad_seg);
  // 1. Calculas el voltaje físico que deseas obtener
  float voltaje_salida = VelocidadToVoltaje * 0.25 + 2.5;

  // 2. Limitas el valor por seguridad (el DAC no puede dar más de 5V ni menos de 0V)
  voltaje_salida = constrain(voltaje_salida, 0.0, 5.0);

  // 3. Conviertes los Voltios (0 a 5.0) a bits del DAC (0 a 4095)
  uint16_t dac_value = (uint16_t)((voltaje_salida / 5.0) * 4095.0);

  // 4. Envías el valor al módulo
  dac.setVoltage(dac_value, false);

  // Telemetría
  static unsigned long ultimoPrint = 0;
  if (millis() - ultimoPrint >= 20) {
    ultimoPrint = millis();
    
    Serial.print("ADC:");
    Serial.print(voltaje_pin, 2);
    Serial.print("| u_Voltios:");
    Serial.print(u_control, 2);
    Serial.print(" | PWM_Exp:");
    Serial.print(pwm_calculado);
    Serial.print(" | rad/s:");
    Serial.print(rad_seg, 1);
    Serial.print(" | DAC_Val:");
    Serial.print(dac_value);
    Serial.print(" | Voltaje de salida:");
    Serial.println(voltaje_salida,2);
  }
}

// =========================================================================
// INTERRUPCIÓN: Medición Instantánea de Velocidad (M Flancos)
// =========================================================================
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

    // 2. Multiplicar toda la ecuación por la variable "sentido"
    rad_seg = sentido * ((2.0f * M * PI * 1e6f) / ((float)PPR * (float)acumuladoDT));
    
    acumuladoDT = 0;
    contadorFlancos = 0;
  }
}
