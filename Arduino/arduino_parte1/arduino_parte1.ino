#include <avr/interrupt.h> 
#include <avr/io.h>
//#include <math.h>

//Constantes para a conversão tensão lida-lux
#define RBASE 65078.575
#define EXPO 1.4241

//Pinos usados
#define OUTPIN 9
#define INPIN 1

#define NSAMPLE 3 //Nº de amostragens por ciclo, para calcular a média
#define MAX_REF 3 //Nº de carateres da referência

//Nº de amostras a enviar para o PC
#define DEBUG_SAMPLES 200

//Ganhos do controlador
#define Ts 0.001496
#define Kf 0.2
#define Kp 0.69
#define Ki 40
#define Kd 0.0022
#define Kw 8
#define a 0.69

int control_flag; //Flag de controlo, se == 1, então há que atuar o sistema
int ref; //Referência pedida
int left2print = DEBUG_SAMPLES; //Nº de amostras que falta enviar

double u3l_ant, u3ll_ant, e_ant; //Variáveis de estado do controlador

void timer2_init() { //Ativa as interrupções do Timer2
  TCCR2B = 0x00;        //Desativa o Timer2 durante a configuração
  TCNT2  = 68;         //Conta de 68 a 255
  TIFR2  = 0x00;        //Limpa a flag de overflow to timer
  TCCR2A = 0x00;        //Geração de onda normal
  TCCR2B = 0x05;        //Prescaler = 128: 1/16E6*(255-68)*128 = 1.496ms
  TIMSK2 = 0x01;        //Ativa interrupções de overflow
}

ISR(TIMER2_OVF_vect) { //ISR do Timer 2
  control_flag = 1; //Ativa flag => novo ciclo de controlo
  //Reinicia o Timer2
  TCNT2 = 68;           //Conta de 68 a 255
  TIFR2 = 0x00;          //Limpa a flag de overflow to timer
}

double input2lux(int vldr) { //Converte de tensões lidas para lux
  double vin = vldr*5.0/1024.0;
  double R = 50000/vin - 10000;
  return pow(RBASE/R, EXPO);
}

int compute_control(int ref, double y) { //Calcula o valor da variável de controlo
  double e, u3l, u3ll, u, unsat;
  
  e = ref - y; //Erro
  u3l = 1/((Kd/a)+Ts)*(Kd*(e - e_ant)+Kd/a*u3l_ant); //Derivador
  u3ll = u3ll_ant + Ki*Ts*e; //Integrador
  
  //Guarda valores para a próxima iteração
  e_ant = e;
  u3l_ant = u3l;
  u3ll_ant = u3ll;

  //Satura o controlo (anti-windup)
  unsat = Kf*ref + Kp*e + u3l + u3ll;
  u = constrain(unsat, 0, 100);
  u3ll_ant += Ki*Ts*Kw*(u-unsat); //Atualiza estado do integrador com o termo de anti-windup
  u += Ki*Ts*Kw*(u-unsat);
  return u;
}

int read_adc(void) { //Lê NSAMPLE amostras e calcula a média
  int i, adc = 0;
  for(i=0;i<NSAMPLE;i++)
    adc += analogRead(INPIN);
  return adc/NSAMPLE;
}

void setup() {
  Serial.begin(115200); //Inicializa a comunicação série
  pinMode(OUTPIN, OUTPUT);
  u3l_ant = u3ll_ant = e_ant = 0; //Variáveis de estado do controlador
  control_flag = 0;
  ref = 0; //Referência inicial (sistema desligado)
  //Altera a frequência do PWM
  TCCR1B &= ~0x07; //Limpa estado dos pinos 9 e 10
  TCCR1B |= 0x02; //Freq. PWM = 3921.16 Hz
  
  timer2_init(); //Ativa as interrupções do Timer2
}

void loop() {
  int i = 0;
  char vec[MAX_REF+1]; //Armazena a referência lida da porta série
  int c, u, y_bin;
  int new_ref = 0; //1 se recebeu nova referência
  double y;
  if (control_flag) {
    control_flag = 0; //Desativa flag

    if (Serial.available() >= 3){ //Nova referência disponível, ler
      for(i=0;i<MAX_REF;i++)
        vec[i] = Serial.read();
      vec[i] = '\0';
      new_ref = 1; //Recebemos nova referência
    }
    
    if (new_ref) {
      ref = atoi(vec); //Converte para inteiro
      new_ref = 0; //Nova referência lida
      left2print = DEBUG_SAMPLES; //Se recebeu nova referência, há que enviar dados para o PC
    }
    
    //Lê a saída do sistema
    y_bin = read_adc();
    y = input2lux(y_bin);
    
    u = compute_control(ref, y); //Calcula atuação
    analogWrite(OUTPIN, map(u, 0, 100, 0, 255)); //Atua o sistema
    
    if (left2print-- > 0) { //Envia amostras para o computador, em hexadecimal (para poupar tempo)
      Serial.print(y_bin, HEX);
      Serial.print('\n');
      Serial.print(u, HEX);
      Serial.print('\n');
    }
  }
}
