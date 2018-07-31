#include <PID_v1.h>

const bool debug = false;

#define preto      0
#define branco     1

#define E2  0
#define E3  1
#define E4  2
#define EC2 3
#define EC3 4
#define C1  5
#define C2  6
#define C3  7
#define DC2 8
#define DC3 9
#define D2  10
#define D3  11
#define D4  12

enum Estado {
  ESTADO_PRINCIPAL = 0,
  ESTADO_GIRANDO_HORARIO_ANGULO,
  ESTADO_GIRANDO_ANTIHORARIO_ANGULO,
  ESTADO_ANDANDO_FRENTE_DISTANCIA,
  ESTADO_PROCEDIMENTO_VERDE_ESQUERDA,
  ESTADO_PROCEDIMENTO_VERDE_DIREITA,
  ESTADO_PARADO,
  ESTADO_TESTE,
  ESTADO_OBSTACULO_PASSO_1,
  ESTADO_OBSTACULO_PASSO_2,
  ESTADO_OBSTACULO_PASSO_3,
  ESTADO_OBSTACULO_PASSO_4,
  ESTADO_OBSTACULO_PASSO_5,
  ESTADO_OBSTACULO_PASSO_6,
  ESTADO_OBSTACULO_PASSO_7,
  ESTADO_OBSTACULO_PASSO_0,
  ESTADO_PAUSA_PROXIMO
};

#define n_leituras 5

//                          E2   E3   E4   EC2  EC3  C1   C2   C3   DC2  DC3  D2   D3   D4
const int sensor[13] =     {A5, A13, A14, A12, A11, A10, A9,  A8,  A7,  A6,  A15,  A4,  A3};
const int min_preto[13]  = {700, 750, 750, 650, 650, 600, 500, 750, 510, 650, 700, 750, 750};
int cor[13], medicao[13], nova_cor[13], indice_mudar[13];
//dessas variáveis a única que deve ser lida é cor[]

#define Kp 0.7
#define Ki 0.05
#define Kd 0.03

#define NOVENTA ( 3.1415926535 * 0.55 )

#define v_min 150
#define v_max 255

#define v_reto 250
#define v_curva 400
#define frente 1
#define tras -1
#define desligado 0

#define andando_frente 1
#define andando_direita 2
#define andando_esquerda 3
#define nao_andando 4

const double RAIO_DA_RODA = 32; //milímetros
const double DISTANCIA_ENTRE_AS_RODAS = 130; //milímetros
/*
   são 20 risquinhos:
  volta inteira = 20*interrupts
  tempo entre interrupts = 1/20*tempo para volta completa
  tempo para volta completa = 20*tempo entre interrupts
  velocidade angular = 2pi*1/tempo para volta completa
  velocidade momentânea = velocidade angular * raio da roda
  velocidade momentânea = 2*pi*raio da roda/(20*tempo entre interrupts)
*/

class motor {
  public:
    int pino_frente, pino_tras, pino_encoder, pino_pwm;
    double v_desejada, pwm;
    volatile double v_real, diff, tempo;
    PID* pid;
    double calc_velocidade() {
      return ((TWO_PI * RAIO_DA_RODA * 1000000) / (20.*diff));
    }

    void atualizar_pwm() {
      pid->Compute();
      analogWrite(pino_pwm, pwm);
    }
    motor (int pfrente, int ptras, int encoder, int PWM) {
      pino_frente = pfrente;
      pino_tras = ptras;
      pino_encoder = encoder;
      pino_pwm = PWM;
      pwm = 255;
      pinMode(pino_frente, OUTPUT);
      pinMode(pino_tras, OUTPUT);
      pinMode(pino_encoder, INPUT);
      pinMode(pino_pwm, OUTPUT);
      analogWrite(pino_pwm, pwm);
      v_desejada = 250;
      pid = new PID(&v_real, &pwm, &v_desejada, Kp, Ki, Kd, DIRECT);
      pid->SetSampleTime(20);
      pid->SetMode(AUTOMATIC);
      pid->SetOutputLimits(v_min, v_max);
    }
    void sentido(int mover) {
      switch (mover) {
        case frente :
          digitalWrite(pino_frente, HIGH);
          digitalWrite(pino_tras, LOW);
          break;
        case tras :
          digitalWrite(pino_frente, LOW);
          digitalWrite(pino_tras, HIGH);
          break;
        case desligado :
          digitalWrite(pino_frente, LOW);
          digitalWrite(pino_tras, LOW);
      }
    }
};


#define MOTOR_F_IN4 50 // IN4 - MOTOR esquerdo - frente = LOW
#define MOTOR_F_IN3 52  // IN3 - MOTOR esquerdo - frente = HIGH
#define MOTOR_F_IN2 46 // IN2 - MOTOR Direito           LOW
#define MOTOR_F_IN1 48  // IN1 - MOTOR direito - frente HIGH
#define MOTOR_T_IN4 44
#define MOTOR_T_IN3 42
#define MOTOR_T_IN2 40
#define MOTOR_T_IN1 38

#define PINO_F_INT0 2 // 
#define PINO_F_INT1 3 // comparadores dos encoders
#define PINO_T_INT0 18 // pinos onde estão ligados os 
#define PINO_T_INT1 19 // comparadores dos encoders

#define PWM_F1 11   //pino ENB do driver ponte H (MOTOR DIREITO)
#define PWM_F2 10    //pino ENA (MOTOR ESQUERDO)
#define PWM_T1 4    // DIREITO
#define PWM_T2 5     // ESQUERDO

#define ECHO_PIN 21 //pino INT2 (precisa de interrupcao)
#define TRIGGER_PIN 22

motor motor_ef(MOTOR_F_IN1, MOTOR_F_IN2, PINO_F_INT0, PWM_F1);
motor motor_df(MOTOR_F_IN4, MOTOR_F_IN3, PINO_F_INT1, PWM_F2);
motor motor_et(MOTOR_T_IN3, MOTOR_T_IN4, PINO_T_INT0, PWM_T2);
motor motor_dt(MOTOR_T_IN2, MOTOR_T_IN1, PINO_T_INT1, PWM_T1);

void intef_encoder() {
  motor_ef.diff = micros() - motor_ef.tempo;
  motor_ef.tempo = micros();
  motor_ef.v_real = motor_ef.calc_velocidade();
}
void intdf_encoder() {
  motor_df.diff = micros() - motor_df.tempo;
  motor_df.tempo = micros();
  motor_df.v_real = motor_df.calc_velocidade();
}
void intet_encoder() {
  motor_et.diff = micros() - motor_et.tempo;
  motor_et.tempo = micros();
  motor_et.v_real = motor_et.calc_velocidade();
}
void intdt_encoder() {
  motor_dt.diff = micros() - motor_dt.tempo;
  motor_dt.tempo = micros();
  motor_dt.v_real = motor_dt.calc_velocidade();
}

#define TESTE_OBSTACULO 1

const float CONVERSAO_P_MILIMETROS = 0.1715; // = 343/2000 milimetos por microsegundo
volatile long tempo_distancia;
volatile float distancia_mm;
volatile int contando;

//void int_inicio_contagem() {
//  //  if (contando){
//  tempo_distancia = micros();
//  detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
//  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), int_fim_contagem, FALLING);
//  //  }
//  //  contando = 1;
//}
//
//void int_fim_contagem() {
//  //  if (contando){
//  tempo_distancia = micros() - tempo_distancia;
//  distancia_mm = tempo_distancia * CONVERSAO_P_MILIMETROS;
//  detachInterrupt(digitalPinToInterrupt(ECHO_PIN));
//  //    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), int_inicio_contagem, RISING);
//  contando = 0;
//  //  }
//}

void int_mudanca_ultrassom() {
  if (digitalRead(ECHO_PIN) == HIGH){
    contando = 1;
    tempo_distancia = micros();
  } else {
    contando = 0;
    tempo_distancia = micros() - tempo_distancia;
    distancia_mm = tempo_distancia * CONVERSAO_P_MILIMETROS;
  }
}

byte movimento;

void configurar_sensores_cor();

void andar_frente() {
  motor_ef.v_desejada = v_reto;
  motor_df.v_desejada = v_reto;
  motor_et.v_desejada = v_reto;
  motor_dt.v_desejada = v_reto;
  motor_ef.sentido(frente);
  motor_df.sentido(frente);
  motor_et.sentido(frente);
  motor_dt.sentido(frente);
  movimento = andando_frente;
}

void virar_direita_suave() {
  motor_ef.v_desejada = v_curva;
  motor_df.v_desejada = v_curva;
  motor_et.v_desejada = v_curva;
  motor_dt.v_desejada = v_curva;
  motor_ef.sentido(frente);
  motor_df.sentido(desligado);
  motor_et.sentido(frente);
  motor_dt.sentido(frente);
  movimento = andando_direita;
}

void virar_esquerda_suave() {
  motor_ef.v_desejada = v_curva;
  motor_df.v_desejada = v_curva;
  motor_et.v_desejada = v_curva;
  motor_dt.v_desejada = v_curva;
  motor_ef.sentido(desligado);
  motor_df.sentido(frente);
  motor_et.sentido(frente);
  motor_dt.sentido(frente);
  movimento = andando_esquerda;
}

void virar_direita_media() {
  motor_ef.v_desejada = v_curva;
  motor_df.v_desejada = v_curva;
  motor_et.v_desejada = v_curva;
  motor_dt.v_desejada = v_curva;
  motor_ef.sentido(frente);
  motor_df.sentido(desligado);
  motor_et.sentido(desligado);
  motor_dt.sentido(frente);
  movimento = andando_direita;
}

void virar_esquerda_media() {
  motor_ef.v_desejada = v_curva;
  motor_df.v_desejada = v_curva;
  motor_et.v_desejada = v_curva;
  motor_dt.v_desejada = v_curva;
  motor_ef.sentido(desligado);
  motor_df.sentido(frente);
  motor_et.sentido(frente);
  motor_dt.sentido(desligado);
  movimento = andando_esquerda;
}

void virar_esquerda_acentuada() {
  motor_ef.v_desejada = v_curva;
  motor_df.v_desejada = v_curva;
  motor_et.v_desejada = v_curva;
  motor_dt.v_desejada = v_curva;
  motor_ef.sentido(tras);
  motor_df.sentido(frente);
  motor_et.sentido(tras);
  motor_dt.sentido(frente);
  movimento = andando_esquerda;
}

void virar_direita_acentuada() {
  motor_ef.v_desejada = v_curva;
  motor_df.v_desejada = v_curva;
  motor_et.v_desejada = v_curva;
  motor_dt.v_desejada = v_curva;
  motor_ef.sentido(frente);
  motor_df.sentido(tras);
  motor_et.sentido(frente);
  motor_dt.sentido(tras);
  movimento = andando_direita;
}

void nao_virar_nada() {
  motor_ef.sentido(desligado);
  motor_df.sentido(desligado);
  motor_et.sentido(desligado);
  motor_dt.sentido(desligado);
  movimento = nao_andando;
}

void avaliar_sensores();

long tempo_atual;
bool wait;
int primeira_vez;
unsigned int cont;

Estado estado_atual = ESTADO_PRINCIPAL;
Estado proximo_estado = ESTADO_PRINCIPAL;

float angulo_restante;
float distancia_desejada;
float distancia_restante;
long tempo_restante;
float v_desejada_final = 250;

void girando() {
  if (primeira_vez++ != 0){
    angulo_restante -= ((((motor_df.v_real + motor_dt.v_real) / 2) + ((motor_ef.v_real + motor_et.v_real) / 2)) / DISTANCIA_ENTRE_AS_RODAS) * (micros() - tempo_atual) / 1000000; //era pra ser uma subtração, mas aqui as velocidades são sempre positivas
  }
  tempo_atual = micros();
}

void andando() {
  if (primeira_vez++ != 0){
    distancia_restante -= ((motor_df.v_real + motor_ef.v_real) / 2) * (micros() - tempo_atual) / 1000000;
  }
  tempo_atual = micros();
  //  motor_ef.v_desejada = (distancia_restante / distancia_desejada) * v_desejada_final;
  //  motor_df.v_desejada = (distancia_restante / distancia_desejada) * v_desejada_final;
  //  motor_et.v_desejada = (distancia_restante / distancia_desejada) * v_desejada_final;
  //  motor_dt.v_desejada = (distancia_restante / distancia_desejada) * v_desejada_final;
}

void funcao_estado_principal() {
  if (distancia_mm <= 85) {
    // OBSTACULO DETECTADO
    // DESVIAR
    tempo_restante = 350000;
    estado_atual = ESTADO_OBSTACULO_PASSO_0;
//    angulo_restante = NOVENTA;
//    estado_atual = ESTADO_OBSTACULO_PASSO_1;
  }
#if TESTE_OBSTACULO == 1
  andar_frente();
#endif
#if TESTE_OBSTACULO == 0
  if (cor[C2] == preto) {
    if (cor[C1] == preto && cor[C3] == preto && movimento == andando_frente) {
      if (cor[E2] == preto && cor[E3] == preto && cor[E4] == preto && cor[D2] == preto && cor[D3] == preto && cor[D4] == preto ) {
        // VERDE DOS DOIS LADOS DETECTADO
        angulo_restante = NOVENTA * 2;
        virar_esquerda_acentuada();
        estado_atual = ESTADO_GIRANDO_ANTIHORARIO_ANGULO;
      }
      else if (cor[E2] == preto && cor[E3] == preto && cor[E4] == preto) {
        // VERDE NA ESQUERDA DETECTADO
        angulo_restante = NOVENTA;
        virar_esquerda_acentuada();
        estado_atual = ESTADO_GIRANDO_ANTIHORARIO_ANGULO;
      }
      else if (cor[D2] == preto && cor[D3] == preto && cor[D4] == preto) {
        // VERDE NA DIREITA DETECTADO
        angulo_restante = NOVENTA;
        virar_direita_acentuada();
        estado_atual = ESTADO_GIRANDO_HORARIO_ANGULO;
      }
    }

    else if (cor [C1] == preto || cor [C3] == preto) andar_frente();
  }
  if (cor[C1] == branco) {
    if (cor[E2] == preto) virar_esquerda_acentuada();
    else if (cor[D2] == preto) virar_direita_acentuada();
    else if (cor[EC2] == preto) virar_esquerda_media();
    else if (cor[DC2] == preto) virar_direita_media();
    if (cor[E2] == branco && cor[EC2] == branco && cor[C2] == branco && cor[DC2] == branco && cor[D2] == branco) {
      if (cor[C3] == preto) {
        if (cor[E3] == preto || cor[EC3] == preto) virar_esquerda_acentuada();
        else if (cor[D3] == preto || cor[DC3] == preto) virar_direita_acentuada();
      }
    }
  }
  else if (cor[C3] == branco) {
    if (cor[DC3] == preto && cor[C3] == branco) virar_esquerda_suave();
    else if (cor[EC3] == preto && cor[C3] == branco) virar_direita_suave();
  }
#endif
}

void funcao_girando_horario_angulo() {
  girando();
  if (angulo_restante <= 0) {
    estado_atual = ESTADO_PRINCIPAL;
  }
}

void funcao_girando_antihorario_angulo() {
  girando();
  if (angulo_restante <= 0) {
    estado_atual = ESTADO_PRINCIPAL;
  }
}

void funcao_andando_frente_distancia() {
  andando();
  if (distancia_restante <= 0) {
    estado_atual = ESTADO_PRINCIPAL;
    //    motor_ef.v_desejada = v_desejada_final;
    //    motor_df.v_desejada = v_desejada_final;
    //    motor_et.v_desejada = v_desejada_final;
    //    motor_dt.v_desejada = v_desejada_final;
  }
}

void funcao_procedimento_verde_esquerda() {
  andando();
  if (distancia_restante <= 0) {
    angulo_restante = NOVENTA;
    virar_esquerda_acentuada();
    estado_atual = ESTADO_GIRANDO_ANTIHORARIO_ANGULO;
    //    motor_ef.v_desejada = v_desejada_final;
    //    motor_df.v_desejada = v_desejada_final;
    //    motor_et.v_desejada = v_desejada_final;
    //    motor_dt.v_desejada = v_desejada_final;
  }
}

void funcao_procedimento_verde_direita() {
  andando();
  if (distancia_restante <= 0) {
    angulo_restante = NOVENTA;
    virar_direita_acentuada();
    estado_atual = ESTADO_GIRANDO_HORARIO_ANGULO;
    //    motor_ef.v_desejada = v_desejada_final;
    //    motor_df.v_desejada = v_desejada_final;
    //    motor_et.v_desejada = v_desejada_final;
    //    motor_dt.v_desejada = v_desejada_final;
  }
}

void funcao_parado() {
  nao_virar_nada();
}

void funcao_teste() {
  // ideia abandonada
}

#define PAUSA 150000

void funcao_obstaculo_passo_1() {
  virar_esquerda_acentuada();
  girando();
  if (angulo_restante <= 0) {
    distancia_restante = 170; //milímetros
//    estado_atual = ESTADO_OBSTACULO_PASSO_2;
    tempo_restante = PAUSA;
    proximo_estado = ESTADO_OBSTACULO_PASSO_2;
    estado_atual = ESTADO_PAUSA_PROXIMO;
  }
}

void funcao_obstaculo_passo_2() {
  andar_frente();
  andando();
  if (distancia_restante <= 0) {
    angulo_restante = NOVENTA;
//    estado_atual = ESTADO_OBSTACULO_PASSO_3;
    tempo_restante = PAUSA;
    proximo_estado = ESTADO_OBSTACULO_PASSO_3;
    estado_atual = ESTADO_PAUSA_PROXIMO;
  }
}

void funcao_obstaculo_passo_3() {
  virar_direita_acentuada();
  girando();
  if (angulo_restante <= 0) {
    distancia_restante = 250; //milímetros
//    estado_atual = ESTADO_OBSTACULO_PASSO_4;
    tempo_restante = PAUSA;
    proximo_estado = ESTADO_OBSTACULO_PASSO_4;
    estado_atual = ESTADO_PAUSA_PROXIMO;
  }
}

void funcao_obstaculo_passo_4() {
  andar_frente();
  andando();
  if (distancia_restante <= 0) {
    angulo_restante = NOVENTA;
//    estado_atual = ESTADO_OBSTACULO_PASSO_5;
    tempo_restante = PAUSA;
    proximo_estado = ESTADO_OBSTACULO_PASSO_5;
    estado_atual = ESTADO_PAUSA_PROXIMO;
  }
}

void funcao_obstaculo_passo_5() {
  virar_direita_acentuada();
  girando();
  if (angulo_restante <= 0) {
    distancia_restante = 170; //milímetros
//    estado_atual = ESTADO_OBSTACULO_PASSO_6;
    tempo_restante = PAUSA;
    proximo_estado = ESTADO_OBSTACULO_PASSO_6;
    estado_atual = ESTADO_PAUSA_PROXIMO;
  }
}

void funcao_obstaculo_passo_6() {
  andar_frente();
  andando();
//  if (distancia_restante <= 0) {
  if (cor[C2] == PRETO){
    angulo_restante = NOVENTA;
//    estado_atual = ESTADO_OBSTACULO_PASSO_7;
    tempo_restante = PAUSA;
    proximo_estado = ESTADO_OBSTACULO_PASSO_7;
    estado_atual = ESTADO_PAUSA_PROXIMO;
  }
}

void funcao_obstaculo_passo_7() {
  virar_esquerda_acentuada();
  girando();
  if (angulo_restante <= 0) {
    estado_atual = ESTADO_PRINCIPAL;
  }
}

void funcao_obstaculo_passo_0(){
  nao_virar_nada();
  if (primeira_vez++ != 0){
    tempo_restante -= micros() - tempo_atual;
  }
  tempo_atual = micros();
  if (tempo_restante <= 0){
    angulo_restante = NOVENTA;
    estado_atual = ESTADO_OBSTACULO_PASSO_1;
  }
}

void funcao_pausa_proximo(){
  nao_virar_nada();
  if (primeira_vez++ != 0){
    tempo_restante -= micros() - tempo_atual;
  }
  tempo_atual = micros();
  if (tempo_restante <= 0){
    estado_atual = proximo_estado;
  }
}

void (*funcoes[])() = {
  funcao_estado_principal,
  funcao_girando_horario_angulo,
  funcao_girando_antihorario_angulo,
  funcao_andando_frente_distancia,
  funcao_procedimento_verde_esquerda,
  funcao_procedimento_verde_direita,
  funcao_parado,
  funcao_teste,
  funcao_obstaculo_passo_1,
  funcao_obstaculo_passo_2,
  funcao_obstaculo_passo_3,
  funcao_obstaculo_passo_4,
  funcao_obstaculo_passo_5,
  funcao_obstaculo_passo_6,
  funcao_obstaculo_passo_7,
  funcao_obstaculo_passo_0,
  funcao_pausa_proximo
};

void setup() {

  attachInterrupt(digitalPinToInterrupt(motor_ef.pino_encoder), intef_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(motor_df.pino_encoder), intdf_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(motor_et.pino_encoder), intet_encoder, RISING);
  attachInterrupt(digitalPinToInterrupt(motor_dt.pino_encoder), intdt_encoder, RISING);

  //  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), int_inicio_contagem, RISING);
  attachInterrupt(digitalPinToInterrupt(ECHO_PIN), int_mudanca_ultrassom, CHANGE);

  configurar_sensores_cor();
  if (debug) Serial.begin(115200);
  cont = 0;
  pinMode(22, OUTPUT);
  pinMode(23, OUTPUT);
  pinMode(24, OUTPUT);
  pinMode(25, OUTPUT);
  pinMode(26, OUTPUT);
  pinMode(27, OUTPUT);
  pinMode(28, OUTPUT);
  pinMode(29, OUTPUT);
  pinMode(30, OUTPUT);
  pinMode(31, OUTPUT);
  pinMode(32, OUTPUT);
  pinMode(33, OUTPUT);
  pinMode(34, OUTPUT);
  pinMode(35, OUTPUT);

  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  delay(1000);
  andar_frente();
}

void debug_led() {
  digitalWrite(34, cor[C1]);
  digitalWrite(35, cor[C2]);
  digitalWrite(31, cor[E2]);
  digitalWrite(32, cor[EC2]);
  digitalWrite(30, cor[D2]);
  digitalWrite(23, cor[DC2]);
  digitalWrite(29, cor[E3]);
  digitalWrite(33, cor[EC3]);
  digitalWrite(25, cor[C3]);
  digitalWrite(26, cor[DC3]);
  digitalWrite(28, cor[D3]);
  digitalWrite(24, cor[E4]);
  digitalWrite(27, cor[D4]);
}

void debug_led_2() {
  digitalWrite(34, 0);
  digitalWrite(35, 0);
  digitalWrite(31, 0);
  digitalWrite(32, 0);
  digitalWrite(30, 0);
  digitalWrite(23, 0);
  digitalWrite(29, 0);
  digitalWrite(33, 0);
  digitalWrite(25, 0);
  digitalWrite(26, 0);
  digitalWrite(28, 0);
  digitalWrite(24, 0);
  digitalWrite(27, 0);
}

void debug_led_3() {
  digitalWrite(34, 1);
  digitalWrite(35, 1);
  digitalWrite(31, 1);
  digitalWrite(32, 1);
  digitalWrite(30, 1);
  digitalWrite(23, 1);
  digitalWrite(29, 1);
  digitalWrite(33, 1);
  digitalWrite(25, 1);
  digitalWrite(26, 1);
  digitalWrite(28, 1);
  digitalWrite(24, 1);
  digitalWrite(27, 1);
}

Estado estado_anterior = ESTADO_TESTE;
#define comprimento_verde 20
void loop() {
  cont++;
  atualizar_sensores_cor();
  debug_led();
  motor_ef.atualizar_pwm();
  motor_df.atualizar_pwm();
  motor_et.atualizar_pwm();
  motor_dt.atualizar_pwm();
  if (!contando && (millis() % 10 <= 0)) {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(3);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);
//    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), int_inicio_contagem, RISING);
//    attachInterrupt(digitalPinToInterrupt(ECHO_PIN), int_fim_contagem, FALLING);
//    tempo_distancia = micros();
  }

  if (estado_anterior != estado_atual){
    primeira_vez = 0;
  }
  estado_anterior = estado_atual;

  //parte de teste, retirar depois
  //  if (estado_atual == ESTADO_PRINCIPAL || estado_atual == ESTADO_TESTE) {
  //    /*if (millis() % 3000 == 0) { //aos sete segundos ele começa a meia volta
  //      angulo_restante = 3.1415926535;
  //      virar_esquerda_acentuada();
  //      estado_atual = ESTADO_GIRANDO_ANTIHORARIO_ANGULO;
  //      debug_led_2();
  //      } else */
  //    if (millis() == 3000 || millis() == 3001 || millis() == 3002) { // aos 19 segundos ele começa a andar 30cm
  //      /*distancia_desejada = 55; // milimetros
  //      distancia_restante = 55; // milimetros
  //      andar_frente();
  //      estado_atual = ESTADO_PROCEDIMENTO_VERDE_ESQUERDA;
  //      debug_led_2();*/
  //
  //      angulo_restante = NOVENTA;
  //      virar_esquerda_acentuada();
  //      estado_atual = ESTADO_GIRANDO_ANTIHORARIO_ANGULO;
  //      debug_led_2();
  //    } else {
  //      motor_ef.sentido(desligado);
  //      motor_df.sentido(desligado);
  //      motor_et.sentido(desligado);
  //      motor_dt.sentido(desligado);
  //      estado_atual = ESTADO_TESTE;
  //      debug_led_3();
  //      tempo_atual = micros();
  //    }
  //  }
  // fim da parte de teste

  funcoes[estado_atual]();

  /*
    if (movimento == dar_meia_volta) {
    comprimento_faltando_d  += motor_df.v_real * (micros() - tempo_atual) / 1000000;
    tempo_atual = micros();
    if (comprimento_faltando_d >= 135 * 3.14159 * 3 / 4 && cor[C2] == preto) andar_frente();
    }
    else if (movimento == ir_esquerda) {
    comprimento_faltando_d  += motor_df.v_real * (micros() - tempo_atual) / 1000000;
    tempo_atual = micros();
    if (comprimento_faltando_d >= 135 * 3.14159 / 4 && cor[C2] == preto) andar_frente();
    }
    else if (movimento == ir_direita) {
    comprimento_faltando_e += motor_ef.v_real * (micros() - tempo_atual) / 1000000;
    tempo_atual = micros();
    if (comprimento_faltando_e >= 135 * 3.14159 / 4 && cor[C2] == preto) andar_frente();
    }
    else {
    if (cor[C1] == preto && cor[C2] == preto && cor[C3] == preto) {
      if (cor[E2] == preto && cor[E3] == preto && cor[E4] == preto && cor[D2] == preto && cor[D3] == preto && cor[D4] == preto) meia_volta();
      else if (cor[E2] == preto && cor[E3] == preto && cor[E4] == preto) virar_esquerda_verde();
      else if (cor[D2] == preto && cor[D3] == preto && cor[D4] == preto) virar_direita_verde();
    }
    else if (cor[C2] == preto && (cor[C1] == preto || cor[C3] == preto)) andar_frente();
    if (cor[C1] == branco) {
      if (cor[E2] == preto) virar_esquerda_acentuada();
      else if (cor[D2] == preto) virar_direita_acentuada();
      else if (cor[EC2] == preto) virar_esquerda_media();
      else if (cor[DC2] == preto) virar_direita_media();
    }
    else if (movimento == ir_frente) {
      if (cor[DC3] == preto ) virar_esquerda_suave();
      else if (cor[EC3] == preto) virar_direita_suave();
    }
    comprimento_faltando_e = 0;
    comprimento_faltando_d = 0;
    tempo_atual = micros();
    }
  */


  if (millis() % 1000 <= 1 && debug) {
    noInterrupts();
    /*Serial.println(motor_ef.v_real);
      Serial.println(motor_df.v_real);
      Serial.println(motor_et.v_real);
      Serial.println(motor_dt.v_real);

      Serial.println();
    */
    Serial.print("medicao atual = ");
    Serial.print(distancia_mm);
    Serial.println(" milimetros");
    Serial.println(contando);

#if TESTE_OBSTACULO == 0
    Serial.print("                ");
    Serial.print(medicao[C1]);
    Serial.print(" ");
    Serial.println(cor[C1]);


    Serial.print(medicao[E2]);
    Serial.print(" ");
    Serial.print(cor[E2]);
    Serial.print("   ");
    Serial.print(medicao[EC2]);
    Serial.print(" ");
    Serial.print(cor[EC2]);
    Serial.print("   ");
    Serial.print(medicao[C2]);
    Serial.print(" ");
    Serial.print(cor[C2]);
    Serial.print("   ");
    Serial.print(medicao[DC2]);
    Serial.print(" ");
    Serial.print(cor[DC2]);
    Serial.print("   ");
    Serial.print(medicao[D2]);
    Serial.print(" ");
    Serial.println(cor[D2]);

    Serial.print("  ");
    Serial.print(medicao[E3]);
    Serial.print(" ");
    Serial.print(cor[E3]);
    Serial.print("  ");
    Serial.print(medicao[EC3]);
    Serial.print(" ");
    Serial.print(cor[EC3]);
    Serial.print("  ");
    Serial.print(medicao[C3]);
    Serial.print(" ");
    Serial.print(cor[C3]);
    Serial.print("  ");
    Serial.print(medicao[DC3]);
    Serial.print(" ");
    Serial.print(cor[DC3]);
    Serial.print("  ");
    Serial.print(medicao[D3]);
    Serial.print(" ");
    Serial.println(cor[D3]);

    Serial.print("  ");
    Serial.print(medicao[E4]);
    Serial.print(" ");
    Serial.print(cor[E4]);
    Serial.print("                       ");
    Serial.print(medicao[D4]);
    Serial.print(" ");
    Serial.println(cor[D4]);

    Serial.println(cont);
    //Serial.println(modo);
    Serial.println(" ");
#endif
    cont = 0;
    interrupts();
  }
}

void atualizar_sensores_cor() {
  for (int i = 0; i < 13; i++) {
    medicao[i] = analogRead(sensor[i]);
    if (medicao[i] > min_preto[i]) {
      if (cor[i] == preto) {
        nova_cor[i] = preto;
        indice_mudar[i] = 0;
      }
      else if (nova_cor[i] != preto) {
        nova_cor[i] = preto;
        indice_mudar[i] = 0;
      }
      else if (indice_mudar[i] < n_leituras ) indice_mudar[i]++;
      else {
        cor[i] = preto;
        indice_mudar[i] = 0;
      }
    }
    else {
      if (cor[i] == branco) {
        nova_cor[i] = branco;
        indice_mudar[i] = 0;
      }
      else if (nova_cor[i] != branco) {
        nova_cor[i] = branco;
        indice_mudar[i] = 0;
      }
      else if (indice_mudar[i] < n_leituras ) indice_mudar[i]++;
      else {
        cor[i] = branco;
        indice_mudar[i] = 0;
      }
    }
  }
}

void configurar_sensores_cor() { //os parâmetros indicam em que cor o sensor vai estar quando o programa começar
  for (int i = 0; i < 13; i++) {
    cor[i] = branco;
    nova_cor[i] = cor[i];
    pinMode(sensor[i], INPUT);
    indice_mudar[i] = 0;
  }
}


