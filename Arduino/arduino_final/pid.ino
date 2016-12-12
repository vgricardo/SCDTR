#include <math.h>

//Constantes para a conversão tensão lida-lux
#define RBASE 65078.575
#define EXPO 1.4241

#define NSAMPLE 3 //Nº de amostragens por ciclo, para calcular a média

//Ganhos do controlador
#define Ts 0.002992
#define Kf 0.2
#define Kp 0.5
#define Ki 30
#define Kd 0.01//0.0011
#define Kw 3
#define a 0.1

float u3l_ant = 0, u3ll_ant = 0, e_ant = 0; //Variáveis de estado do controlador

void inicializa_pid(void) {
  u3l_ant = u3ll_ant = e_ant = 0;
}

float input2lux(int vldr) { //Converte de tensões lidas para lux
  float vin = vldr*5.0/1024.0;
  float R = 50000/vin - 10000;
  return pow(RBASE/R, EXPO);
}

int le_adc(void) { //Lê NSAMPLE amostras e calcula a média
  int adc = 0;
  for(int i=0;i<NSAMPLE;i++)
    adc += analogRead(INPIN);
  return adc/NSAMPLE;
}

int compute_control(int ref, float y) { //Calcula o valor da variável de controlo
  float e, u3l, u3ll, u, unsat, intg;
  
  e = ref - y; //Erro
  u3l = 1/((Kd/a)+Ts)*(Kd*(e - e_ant)+Kd/a*u3l_ant); //Derivador
  u3ll = u3ll_ant + Ki*Ts*e; //Integrador
  intg = Ki*Ts*e;
  
  //Guarda valores para a próxima iteração
  e_ant = e;
  u3l_ant = u3l;
  u3ll_ant = u3ll;

  //Satura o controlo (anti-windup)
  unsat = Kf*ref + Kp*e + u3l + u3ll;
  u = constrain(unsat, 0, 100);
  if (u-unsat != 0)
    u3ll_ant -= intg;

  return u;
}
