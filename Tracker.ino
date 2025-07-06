#include <Wire.h>
#include <TimeLib.h>
#include <SolarPosition.h>
#include <RTClib.h>
RTC_DS1307 rtc;


#include "LowPower.h"
#define led  13

#define IN1_Y  5
#define IN2_Y  6
#define EN_Y 2
#define IN1_X 10
#define IN2_X 9
#define EN_X 11
#define en A0

#define TOTAL_STEPS_X 1200
#define MOTOR_DELAY_X 50
#define VELOCIDADE_MOTOR_X 255
#define TOTAL_STEPS_Y 440
#define MOTOR_DELAY_Y 50
#define VELOCIDADE_MOTOR_Y 255
double latitude = -23.3045;
double longitude = -51.1696;
const int UTC_OFFSET = -3;
int posicaoAtualX = 0;
int posicaoAtualY = 0;
int posicaoDesejadaX = 0;
int posicaoDesejadaY = 0;
bool modoManual = false;
bool painelEstacionado = false;
bool movimentoEmCurso = false;
bool enablesleep = false;
unsigned long ultimaAtualizacao = 0;
unsigned long ultimaMensagemDebug = 0;
#define DEG_TO_RAD 0.01745329252
#define RAD_TO_DEG 57.295779513
double calcularIncidencia(double elevacao, double azimute, double inclinacaoPainel, double azimutePainel) {
  double el = elevacao * DEG_TO_RAD;
  double az = azimute * DEG_TO_RAD;
  double tilt = inclinacaoPainel * DEG_TO_RAD;
  double azp = azimutePainel * DEG_TO_RAD;
  double cosIncidencia = sin(el) * cos(tilt) + cos(el) * sin(tilt) * cos(az - azp);
  if (cosIncidencia > 1.0) cosIncidencia = 1.0;
  if (cosIncidencia < -1.0) cosIncidencia = -1.0;
  return acos(cosIncidencia) * RAD_TO_DEG;
}
// Liga todos os timers usados por analogWrite()
// Liga timers PWM (para analogWrite funcionar)
void ligarTimersPWM() {
  PRR &= ~(1 << PRTIM0); // Liga Timer0
  PRR &= ~(1 << PRTIM1); // Liga Timer1
  PRR &= ~(1 << PRTIM2); // Liga Timer2
}

// Desliga/ Desliga timers PWM (para reduzir consumo ao dormir)
void desligarTimersPWM() {
  PRR |= (1 << PRTIM0);  // Desliga Timer0
  PRR |= (1 << PRTIM1);  // Desliga Timer1
  PRR |= (1 << PRTIM2);  // Desliga Timer2
}

void moverParaX(int destino);
void moverParaY(int destino);
void motorStopX() {
  digitalWrite(IN1_X, LOW);
  digitalWrite(IN2_X, LOW);
  analogWrite(EN_X, 0);
}
void motorStepRightX() {
  digitalWrite(IN1_X, LOW);
  digitalWrite(IN2_X, HIGH);
  analogWrite(EN_X, VELOCIDADE_MOTOR_X);
  delay(MOTOR_DELAY_X);
  motorStopX();
  posicaoAtualX++;
}

double inclinacaoPainel = 15;
double azimutePainel = 0.0;    // Exemplo: painel virado para o sul
void motorStepLeftX() {
  digitalWrite(IN1_X, HIGH);
  digitalWrite(IN2_X, LOW);
  analogWrite(EN_X, VELOCIDADE_MOTOR_X);
  delay(MOTOR_DELAY_X);
  motorStopX();
  posicaoAtualX--;
}
void motorStopY() {
  digitalWrite(IN1_Y, LOW);
  digitalWrite(IN2_Y, LOW);
  analogWrite(EN_Y, 0);
}
void motorStepUpY() {
  digitalWrite(IN1_Y, LOW);
  digitalWrite(IN2_Y, HIGH);
  analogWrite(EN_Y, VELOCIDADE_MOTOR_Y);
  delay(MOTOR_DELAY_Y);
  motorStopY();
  posicaoAtualY++;
}

void motorStepDownY() {
  digitalWrite(IN1_Y, HIGH);
  digitalWrite(IN2_Y, LOW);
  analogWrite(EN_Y, VELOCIDADE_MOTOR_Y);
  delay(MOTOR_DELAY_Y);
  motorStopY();
  posicaoAtualY--;
}
void moverParaXY(int destinoX, int destinoY) {
  destinoX = constrain(destinoX, 0, TOTAL_STEPS_X);
  destinoY = constrain(destinoY, 0, TOTAL_STEPS_Y);

  while (posicaoAtualX != destinoX || posicaoAtualY != destinoY) {
    if (posicaoAtualX < destinoX) {
      motorStepRightX();
    } else if (posicaoAtualX > destinoX) {
      motorStepLeftX();
    }

    if (posicaoAtualY < destinoY) {
      motorStepUpY();
    } else if (posicaoAtualY > destinoY) {
      motorStepDownY();
    }
  }

  enablesleep = true;
}


/*
void moverParaX(int destino) {
  destino = constrain(destino, 0, TOTAL_STEPS_X);
  while (posicaoAtualX < destino) motorStepRightX();
  while (posicaoAtualX > destino) motorStepLeftX();
}
void moverParaY(int destino) {

  destino = constrain(destino, 0, TOTAL_STEPS_Y);
  while (posicaoAtualY < destino) motorStepUpY();
  while (posicaoAtualY > destino) motorStepDownY();
  enablesleep = true;


}
*/




void forcarFimDeCursoX() {
  Serial.println("Calibrando motor X...");
    digitalWrite(en, HIGH);

  for (int i = 0; i < TOTAL_STEPS_X; i++) motorStepLeftX();
  posicaoAtualX = 0;
  posicaoDesejadaX = 0;
  Serial.println(" motor X Calibrado");
     digitalWrite(en, LOW);


}
void forcarFimDeCursoY() {
     digitalWrite(en, HIGH);

  Serial.println("Calibrando motor Y...");

  for (int i = 0; i < TOTAL_STEPS_Y; i++) motorStepDownY();
  posicaoAtualY = 0;
  posicaoDesejadaY = 0;
  Serial.println(" motor Y Calibrado");
     digitalWrite(en, LOW);


}
int anguloParaPassosX(float azimute) {
  azimute = fmod(azimute, 360.0);
  if (azimute < 0) azimute += 360.0;
  float anguloReal;
  if (azimute >= 0 && azimute <= 90) {
    anguloReal = 90 - azimute;
  } else if (azimute >= 270 && azimute <= 360) {
    anguloReal = 450 - azimute;
  } else {
    return -1;
  }
  float passos = anguloReal * (TOTAL_STEPS_X / 180.0);
  Serial.print("Passos calculados: ");
  Serial.println(passos);

  return constrain((int)passos, 0, TOTAL_STEPS_X);
}

#define PASSOS_Y_MIN 0
#define PASSOS_Y_MAX 420
#define ANGULO_PAINEL_MIN 105.0
#define ANGULO_PAINEL_MAX 180.0
int anguloParaPassosY(float elevacao) {
  // Primeiro converte elevação solar para ângulo do painel entre 105 e 180 graus
  float anguloPainel = ANGULO_PAINEL_MIN + (elevacao * (ANGULO_PAINEL_MAX - ANGULO_PAINEL_MIN)) / 90.0;

  // Depois faz a proporção para o motor (de PASSOS_Y_MIN a PASSOS_Y_MAX)
  float proporcao = (anguloPainel - ANGULO_PAINEL_MIN) / (ANGULO_PAINEL_MAX - ANGULO_PAINEL_MIN);

  return PASSOS_Y_MIN + proporcao * (PASSOS_Y_MAX - PASSOS_Y_MIN);
}

time_t converterDataParaTimeT(int ano, int mes, int dia, int hora, int minuto, int segundo) {
  tmElements_t tm;
  tm.Year = ano - 1970;
  tm.Month = mes;
  tm.Day = dia;
  tm.Hour = hora;
  tm.Minute = minuto;
  tm.Second = segundo;
  return makeTime(tm);
}

void setup() {
  Serial.begin(115200);
  delay(1000); // Pequena pausa para o Serial conectar


  if (! rtc.begin()) {
    Serial.println("Couldn't find RTC");
    Serial.flush();
    while (1) delay(10);
  }

  if (! rtc.isrunning()) {
    Serial.println("RTC is NOT running, let's set the time!");
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  }
 // rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
//setTime(17, 10, 0, 27, 6, 2025); 
  pinMode(en, OUTPUT);
  digitalWrite(en, LOW);
  pinMode(IN1_X, OUTPUT);
  pinMode(IN2_X, OUTPUT);
  pinMode(EN_X, OUTPUT);
  pinMode(IN1_Y, OUTPUT);
  pinMode(IN2_Y, OUTPUT);
  pinMode(EN_Y, OUTPUT);
  motorStopX();
  motorStopY();

  movimentoEmCurso = true;
  ultimaAtualizacao = millis();

  DateTime now = rtc.now();

  int ano = now.year();
  int mes = now.month();
  int dia = now.day();
  int hora = now.hour();
  int minuto = now.minute();
  int segundo = now.second();
  time_t tempoUTC = converterDataParaTimeT(ano, mes, dia, hora, minuto, segundo);

  Serial.println("---- Resultado ----");
  Serial.print("Data e hora UTC: ");
  Serial.print(hora);
  Serial.print(":");
  Serial.print(minuto);
  Serial.print(":");
  Serial.print(segundo);
  Serial.print(" UTC, ");
  Serial.print(dia);
  Serial.print("/");
  Serial.print(mes);
  Serial.print("/");
  Serial.println(ano);
     digitalWrite(en, HIGH);

  forcarFimDeCursoX();
  forcarFimDeCursoY();
  ultimaAtualizacao = millis() + 120001;

}



void prepararPinosParaSleep() {
  // Pinos usados para motores
  int pinos[] = {IN1_Y, IN2_Y, EN_Y, IN1_X, IN2_X, EN_X};
  for (int i = 0; i < 6; i++) {
    pinMode(pinos[i], OUTPUT);     // Configura como saída
    digitalWrite(pinos[i], LOW);   // Coloca em nível baixo para não acionar nada e evitar consumo
  }
}





bool soneca = false;

void loop() {

  DateTime now = rtc.now();

  int ano = now.year();
  int mes = now.month();
  int dia = now.day();
  int hora = now.hour();
  int minuto = now.minute();
  int segundo = now.second();

  // Converte hora local para UTC
  time_t tempoUTC = converterDataParaTimeT(ano, mes, dia, hora, minuto, segundo) - (UTC_OFFSET * SECS_PER_HOUR);

  // Calcula posição solar
  SolarPosition_t pos = calculateSolarPosition(tempoUTC, latitude * DEG_TO_RAD, longitude * DEG_TO_RAD);
  double elevacao = pos.elevation * RAD_TO_DEG;
  Serial.print("Elevacao: ");
  Serial.println(elevacao);
  double azimute = pos.azimuth * RAD_TO_DEG;

  // Define posições desejadas com base na elevação
  if (elevacao > 2) {
    posicaoDesejadaX = anguloParaPassosX(azimute);
    posicaoDesejadaY = anguloParaPassosY(elevacao);
    painelEstacionado = false;
      

    soneca = true;
    
  } else {
        Serial.println("by by indo dormir");

    forcarFimDeCursoX();
    forcarFimDeCursoY();
    posicaoDesejadaX = 0;
    posicaoDesejadaY = 0;
    painelEstacionado = true;
  }

  // Move os motores
  movimentoEmCurso = true;
  if (movimentoEmCurso) {
   ligarTimersPWM();
      digitalWrite(en, HIGH);

  moverParaXY(posicaoDesejadaX, posicaoDesejadaY);
  delay(50);
  movimentoEmCurso = false;
     digitalWrite(en, LOW);

  }

  // Prepara os pinos antes de dormir
  prepararPinosParaSleep();
  Serial.println("by by cochilo");
     digitalWrite(en, LOW);


  if (soneca)  {
    delay(50);

    soneca = false;
    for (int i = 0; i < 15; i++) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // 120 segundos
    }
  } else {
        delay(50);

    for (int i = 0; i < 500; i++) {
      LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF); // ~1h 6min
    }
  }

}
