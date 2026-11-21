// =============================================================================
// Controlador por Realimentación de Estados con Integrador y Estimador Actual
// Microcontrolador: Arduino Mega 2560
//  - Timer1 (CTC) dispara la ISR cada T:
//      1) Lee e[k] desde A7 (error acondicionado) y y[k] desde el encoder
//      2) Calcula estimador actual x_est[k] usando y[k]
//      3) Calcula estado integral v[k] = v[k-1] + e[k]
//      4) Calcula u[k], satura y aplica PWM (ZOH)
//      5) Actualiza variables [k-1] = [k]
//  - loop(): únicamente salida continua del DAC (velocidad acondicionada)
// =============================================================================
#include "Wire.h"
#include "Adafruit_MCP4725.h"

Adafruit_MCP4725 dac;

// ---------------- Configuración de Tiempo ----------------
float T = 0.014;                      // Periodo de muestreo [s] (igual al de MATLAB)



// ---------------- Ganancias del Controlador y Estimador Actual (MATLAB) ---------
const float K1   = 3.4377;            // Ganancia integral (K1)
const float K2_1 = 4.6523;            // Realimentación estado 1 (K2(1))
const float K2_2 = 0.0071;            // Realimentación estado 2 (K2(2))

// Matrices precalculadas del estimador actual:
// x_est[k] = (G - L*C*G)*x_est[k-1] + (H - L*C*H)*u[k-1] + L*y[k]
// x_est[k] = Ae*x_est[k-1] + Be*u[k-1] + L*y[k]
float Ae11 = 0.0, Ae12 = 0.0, Ae21 = -0.1460, Ae22 = 0;
float Be1  = 0.0, Be2  = 10.9952;
const float L1   = 1;            // Ganancia estimador actual L(1)
const float L2   = -10.1051;            // Ganancia estimador actual L(2)

// ---------------- Variables de Estado (solo se usan dentro de la ISR) ----------
float x1_est_k = 0.0, x1_est_k_1 = 0.0;
float x2_est_k = 0.0, x2_est_k_1 = 0.0;
float v_k      = 0.0, v_k_1      = 0.0;
float u_k      = 0.0, u_k_1      = 0.0;
float y_k      = 0.0, y_k_1      = 0.0;
float e_k      = 0.0;

// ---------------- Pines ----------------
const int PIN_U_ANALOG = A7;          // Entrada del error acondicionado eac(t)
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10;
const int encoderA = 2;
const int encoderB = 3;

// ---------------- Parámetros del Sistema ----------------
const float V_MAX = 12.0;
const int PPR = 509.9;

volatile unsigned long tiempoAnteriorA = 0;
volatile bool primerFlanco = true;
volatile float rad_seg = 0.0;
volatile unsigned long ultimoFlancoTiempo = 0;

// ---------------- Telemetría (comentada en loop) ----------------
const uint8_t IMPRIMIR_CADA = 1;
volatile uint32_t kMuestra = 0;
volatile uint32_t telem_k = 0;
volatile float telem_e  = 0.0;
volatile float telem_y  = 0.0;
volatile float telem_u  = 0.0;
volatile float telem_x1 = 0.0;
volatile float telem_x2 = 0.0;
volatile bool nuevaMuestra = false;
volatile uint8_t contMuestras = 0;

void flancoA();

int calcularPWM_Experimental(float Voltaje) {
  if (fabs(Voltaje) < 2.18) return 0; 
  float PWM = -374274.5 + Voltaje * (222018.6 + Voltaje * (-52479.65 + Voltaje * (6181.513 + Voltaje * (-362.9321 + Voltaje * 8.500599))));
  int pwm_out = constrain((int)PWM, 0, 255);
  return (pwm_out < 40) ? 0 : pwm_out;
}

float calcularVoltaje_Velocidad(float Velocidad) {
  if (fabs(Velocidad) < 2.26) return 0.0;
  float signo = (Velocidad >= 0.0) ? 1.0 : -1.0;
  float Voltaje = signo * ((0.9948340 * fabs(Velocidad)) - 0.3015020);
  return constrain(Voltaje, -V_MAX, V_MAX);
}


// Configura Timer1 para el periodo deseado e inicializa variables en cero
void configurarTimer1(float T_deseado) {
  const uint16_t presc[] = {8, 64, 256, 1024};
  const uint8_t  bits[]  = {(1 << CS11),
                            (1 << CS11) | (1 << CS10),
                            (1 << CS12),
                            (1 << CS12) | (1 << CS10)};

  uint8_t i;
  float ticks = 0;
  for (i = 0; i < 4; i++) {
    ticks = T_deseado * 16000000.0 / presc[i];
    if (ticks <= 65536.0) break;
  }
  if (i == 4) { i = 3; ticks = 65536.0; }

  uint16_t ocr = (uint16_t)(ticks + 0.5) - 1;

  noInterrupts();
  TIMSK1 &= ~(1 << OCIE1A);
  T = (ocr + 1.0) * presc[i] / 16000000.0;
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | bits[i];
  OCR1A  = ocr;
  TCNT1  = 0;
  
  // Inicializar TODAS las variables en CERO
  x1_est_k = x1_est_k_1 = 0.0;
  x2_est_k = x2_est_k_1 = 0.0;
  v_k      = v_k_1      = 0.0;
  u_k      = u_k_1      = 0.0;
  y_k      = y_k_1      = 0.0;
  e_k      = 0.0;

  kMuestra = 0;
  contMuestras = 0;
  nuevaMuestra = false;

  TIFR1 |= (1 << OCF1A);
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

  Serial.print("T,");
  Serial.println(T, 6);
}

void setup() {
  Serial.begin(115200);
  Wire.begin();
  Wire.setClock(400000);
  if (!dac.begin(0x60)) Serial.println("Error: No se detecto el MCP4725.");

  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT); pinMode(ENA, OUTPUT);
  pinMode(PIN_U_ANALOG, INPUT);
  pinMode(encoderA, INPUT_PULLUP); pinMode(encoderB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderA), flancoA, RISING);
  configurarTimer1(T);
}

// =========================================================================
// LOOP: únicamente salida continua del DAC (velocidad)
// =========================================================================
void loop() {
  noInterrupts();
  float velocidad_actual = rad_seg;
  interrupts();

  float V_sin_acond = calcularVoltaje_Velocidad(velocidad_actual);            // -10 a 10 V
  float V_acond = constrain(V_sin_acond * 0.25 + 2.5, 0.0, 5.0);              // 0 a 5 V (sale al DAC)

  uint16_t dac_value = (uint16_t)((V_acond / 5.0) * 4095.0);
  dac.setVoltage(dac_value, false);

  // Telemetría: una línea por cada muestra del controlador
  /*if (nuevaMuestra) {
    noInterrupts();
    uint32_t k = telem_k;
    float e  = telem_e;
    float y  = telem_y;
    float u  = telem_u;
    float x1 = telem_x1;
    float x2 = telem_x2;
    nuevaMuestra = false;
    interrupts();

    Serial.print(k);     Serial.print(',');
    Serial.print(e, 3);  Serial.print(',');
    Serial.print(y, 3);  Serial.print(',');
    Serial.print(u, 3);  Serial.print(',');
    Serial.print(x1, 3); Serial.print(',');
    Serial.println(x2, 3);
  }*/
}

// =========================================================================
// ISR DEL TIMER: se ejecuta cada T
// =========================================================================
ISR(TIMER1_COMPA_vect) {
  sei(); // Permite anidar la ISR del encoder

  // 1. Leer e[k] desde A7: eac(t) = e(t)*0.2 + 2.5
  int adc_raw = analogRead(PIN_U_ANALOG);
  float voltaje_pin = adc_raw * (5.0 / 1023.0) - 0.15;
  e_k = (voltaje_pin - 2.5) * 5.0;

  // 2. Verificar timeout del encoder y acceder al último valor medido y[k]
  noInterrupts();
  unsigned long ultimoFlanco = ultimoFlancoTiempo;
  if (micros() - ultimoFlanco > 300000UL) {
    rad_seg = 0.0;
    primerFlanco = true;
  }
  float vel_actual = rad_seg;
  interrupts();

  y_k = calcularVoltaje_Velocidad(vel_actual);

  // 3. Implementar el estimador actual (ecuación desglosada):
  //    x_est[k] = (G - L*C*G)*x_est[k-1] + (H - L*C*H)*u[k-1] + L*y[k]
  x1_est_k = Ae11 * x1_est_k_1 + Ae12 * x2_est_k_1 + Be1 * u_k_1 + L1 * y_k;
  x2_est_k = Ae21 * x1_est_k_1 + Ae22 * x2_est_k_1 + Be2 * u_k_1 + L2 * y_k;

  // 4. Calcular estado del integrador v[k] = v[k-1] + e[k]
  v_k = v_k_1 + e_k;

  // 5. Calcular acción de control u[k] = -K2*x_est[k] + K1*v[k]
  float u_raw = -K2_1 * x1_est_k - K2_2 * x2_est_k + K1 * v_k;

  // 6. Saturar u[k] antes de enviarla al motor (y anti-windup)
  u_k = constrain(u_raw, -V_MAX, V_MAX);
  if (u_k != u_raw) {
    v_k = v_k_1;
  }

  // 7. Enviar u[k] al puente H (retenedor de orden cero)
  int pwm_calculado = calcularPWM_Experimental(abs(u_k));

  if (pwm_calculado == 0) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  } else if (u_k > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, pwm_calculado);

  // 8. Guardar datos para telemetría
  if (++contMuestras >= IMPRIMIR_CADA) {
    contMuestras = 0;
    telem_k  = kMuestra;
    telem_e  = e_k;
    telem_y  = y_k;
    telem_u  = u_k;
    telem_x1 = x1_est_k;
    telem_x2 = x2_est_k;
    nuevaMuestra = true;
  }
  kMuestra++;

  // 9. Actualizar variables para la siguiente iteración [k-1] = [k]
  x1_est_k_1 = x1_est_k;
  x2_est_k_1 = x2_est_k;
  v_k_1      = v_k;
  u_k_1      = u_k;
  y_k_1      = y_k;
}

// =========================================================================
// ISR DEL ENCODER: velocidad en cada flanco
// =========================================================================
void flancoA() {
  unsigned long tiempoActual = micros();
  ultimoFlancoTiempo = tiempoActual;

  if (primerFlanco) {
    tiempoAnteriorA = tiempoActual;
    primerFlanco = false;
    return;
  }

  unsigned long dt = tiempoActual - tiempoAnteriorA;
  tiempoAnteriorA = tiempoActual;
  if (dt == 0) return;

  int sentido = (digitalRead(encoderB) == HIGH) ? 1 : -1;
  rad_seg = sentido * ((2.0f * PI * 1e6f) / ((float)PPR * (float)dt));
}
