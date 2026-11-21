// =============================================================================
// Controlador PI discretizado con interrupción de timer (Arduino Mega 2560)
//  - Timer1 (CTC) dispara la ISR cada T: lee e[k], calcula u[k], saca PWM (ZOH)
//  - loop(): solo DAC (velocidad acondicionada) + envío de telemetría por Serial
//  - El método y T se cambian aquí en el código (METODO_DISCRETO y T)
//  < 
// Al iniciar envía una línea de cabecera:   T,<periodo real en s>,M,<método>
// Telemetría (CSV, una línea por muestra):  k,e[k],u[k],DAC_sin_acond,DAC_acond
// =============================================================================
#include "Wire.h"
#include "Adafruit_MCP4725.h"

Adafruit_MCP4725 dac;

// ---------------- Configuración del Controlador ----------------
// 1: Diferencia Atrás | 2: Diferencia Adelante - Escalon | 3: Regla Trapezoidal
// 4: Invarianza al Impulso
int METODO_DISCRETO = 3;

const float Kp = 1.5;
const float Ki = 10.0;
float T = 0.0014;                    // Periodo de muestreo deseado [s] (cámbialo aquí)

// Coeficientes de C(z) = (b0 + b1 z^-1 + b2 z^-2) / (a0 + a1 z^-1 + a2 z^-2)
float b0 = 0.0, b1 = 0.0, b2 = 0.0, a0 = 1.0, a1 = -1.0, a2 = 0.0;

// Variables de estado (solo se usan dentro de la ISR)
float e_k = 0.0, e_k_1 = 0.0, e_k_2 = 0.0;
float u_k = 0.0, u_k_1 = 0.0, u_k_2 = 0.0;

// ---------------- Pines ----------------
const int PIN_U_ANALOG = A7;        // Entrada del error acondicionado eac(t)
const int IN1 = 8;
const int IN2 = 9;
const int ENA = 10;
const int encoderA = 2;
const int encoderB = 3;

// ---------------- Parámetros del Sistema ----------------
const float V_MAX = 10.0;
const int PPR = 750;

volatile unsigned long tiempoAnteriorA = 0;
volatile bool primerFlanco = true;
volatile float rad_seg = 0.0;
volatile unsigned long ultimoFlancoTiempo = 0;

// ---------------- Telemetría ----------------
const uint8_t IMPRIMIR_CADA = 1;    // imprime 1 de cada N muestras (subir si T es muy pequeño)
volatile uint32_t kMuestra = 0;     // índice de muestra k
volatile uint32_t telem_k = 0;
volatile float telem_e = 0.0;
volatile float telem_u = 0.0;
volatile bool nuevaMuestra = false;
volatile uint8_t contMuestras = 0;

void flancoA();

int calcularPWM_Experimental(float Voltaje) {
  float PWM = -374274.0 + Voltaje * (222019.0 + Voltaje * (-52480.0 + Voltaje * (6181.5 + Voltaje * (-362.93 + Voltaje * 8.5006))));
  return constrain((int)PWM, 0, 255);
}

float calcularVoltaje_Velocidad(float Velocidad) {
  float Voltaje = 0.8895 * Velocidad + 0.839;
  return constrain(Voltaje, -V_MAX, V_MAX);
}

void calcularCoeficientes() {
  a0 = 1.0; a1 = -1.0; a2 = 0.0; b2 = 0.0;
  switch (METODO_DISCRETO) {
    case 1: // Diferencia Atrás
      b0 = Kp + Ki * T;       b1 = -Kp;
      break;
    case 2: // Diferencia Adelante - escalon
      b0 = Kp;                b1 = Ki * T - Kp;
      break;
    case 3: // Trapezoidal (Tustin)
      b0 = Kp + Ki * T / 2.0; b1 = Ki * T / 2.0 - Kp;
      break;
    case 4: // Invarianza al Impulso
          b0 = Kp + Ki * T;   
          b1 = -Kp;
          break;
  }
}

// Configura Timer1 para el periodo deseado (elige el prescaler más fino posible)
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
  if (i == 4) { i = 3; ticks = 65536.0; }          // T demasiado grande: límite ~4.19 s

  uint16_t ocr = (uint16_t)(ticks + 0.5) - 1;

  noInterrupts();
  TIMSK1 &= ~(1 << OCIE1A);                        // detiene la ISR mientras se reconfigura
  T = (ocr + 1.0) * presc[i] / 16000000.0;         // periodo REAL que quedó
  TCCR1A = 0;
  TCCR1B = (1 << WGM12) | bits[i];                 // CTC + prescaler elegido
  OCR1A  = ocr;
  TCNT1  = 0;

  calcularCoeficientes();                          // usa el T real
  e_k_1 = e_k_2 = 0.0;                             // condiciones iniciales en cero
  u_k_1 = u_k_2 = 0.0;
  kMuestra = 0;
  contMuestras = 0;
  nuevaMuestra = false;

  TIFR1 |= (1 << OCF1A);                           // limpia bandera pendiente
  TIMSK1 |= (1 << OCIE1A);
  interrupts();

  Serial.print("T,");
  Serial.print(T, 6);                              // periodo REAL
  Serial.print(",M,");
  Serial.println(METODO_DISCRETO);                 // método en uso
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
  configurarTimer1(T);   // calcula coeficientes y deja u[k-1]=u[k-2]=e[k-1]=e[k-2]=0
}

// =========================================================================
// LOOP: solo salida del DAC (velocidad) + telemetría
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
    float e = telem_e;
    float u = telem_u;
    nuevaMuestra = false;
    interrupts();

    Serial.print(k);               Serial.print(',');
    Serial.print(e, 3);            Serial.print(',');
    Serial.print(u, 3);            Serial.print(',');
    Serial.print(V_sin_acond, 3);  Serial.print(',');
    Serial.println(V_acond, 3);
  }*/ 
}

// =========================================================================
// ISR DEL TIMER: se ejecuta cada T
// =========================================================================
ISR(TIMER1_COMPA_vect) {
  sei(); // permite anidar la ISR del encoder (la bandera del timer ya se limpió)

  // 1. Leer e[k]:  eac(t) = e(t)*0.2 + 2.5
  int adc_raw = analogRead(PIN_U_ANALOG);
  float voltaje_pin = adc_raw * (5.0 / 1023.0)-0.15;
  e_k = (voltaje_pin - 2.5) * 5.0;

  // 2. Calcular u[k] con la ecuación en diferencias general
  u_k = (b0 / a0) * e_k + (b1 / a0) * e_k_1 + (b2 / a0) * e_k_2
        - (a1 / a0) * u_k_1 - (a2 / a0) * u_k_2;

  // 3. Saturación física (-10 a 10 V) y anti-windup
  u_k = constrain(u_k, -10.0, 10.0);

  // 4. Sacar u[k]: se mantiene constante hasta la siguiente ISR (retenedor)
  int pwm_calculado = calcularPWM_Experimental(abs(u_k));

  if (pwm_calculado == 0) {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, LOW);
  } else if (u_k > 0) {
    digitalWrite(IN1, HIGH); digitalWrite(IN2, LOW);
  } else {
    digitalWrite(IN1, LOW);  digitalWrite(IN2, HIGH);
  }
  analogWrite(ENA, pwm_calculado);

  // 5. Telemetría: solo copia valores, la impresión se hace en loop()
  if (++contMuestras >= IMPRIMIR_CADA) {
    contMuestras = 0;
    telem_k = kMuestra;
    telem_e = e_k;
    telem_u = u_k;
    nuevaMuestra = true;
  }
  kMuestra++;

  // 6. Actualizar variables
  u_k_2 = u_k_1;
  u_k_1 = u_k;
  e_k_2 = e_k_1;
  e_k_1 = e_k;

  // 7. Timeout si el motor se detiene
  noInterrupts();
  unsigned long ultimoFlanco = ultimoFlancoTiempo;
  interrupts();

  if (micros() - ultimoFlanco > 300000UL) {
    noInterrupts();
    rad_seg = 0.0;
    primerFlanco = true;
    interrupts();
  }
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
