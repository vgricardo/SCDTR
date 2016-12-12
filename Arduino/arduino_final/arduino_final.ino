#include <Wire.h>
#define ADDRESS 1
#define TAM_MATRIZ 24
#define MAX_MSG TAM_MATRIZ+1 //+1 (\0)
#define NUMCHR 4
#define TAM_REF NUMCHR
   
//Ações (em cada ciclo de controlo)
#define OFF 0
#define ON 1
#define LE 2
#define FIM 3
#define NOP 4
#define I2C 5
#define ESPERA_COLUNAS 6
#define CONTROLO 7

//Estados (para a calibração)
#define CALIBR 0
#define LE_MATRIZ 1
#define NOVA_REF 2

#define DIM 3
#define DIM2 3*DIM

#define M 1000
#define INFEASIBLE -1
#define UNBOUNDED -2

#define OUTPIN 9
#define INPIN 1

#define T 2992 //Período de amostragem

//Variáveis para a calibração e máquina
int nchar, estado, acao;
int col_matriz;
char leitura[MAX_MSG];
char col_lida[TAM_MATRIZ / 2 + 1]; //+1 ('\0')

//Matrizes do Sistema
float E[DIM][DIM];
float O[DIM];
float L[DIM] = {0, 0, 0};

//Matrizes do Simplex;
float A[DIM][DIM2];
float b[DIM];
int c[DIM2] = {1, 1, 1, 0, 0, 0, M, M, M};
int var[DIM2] = {6, 7, 8, 0, 1, 2, 3, 4, 5}; //Variáveis básicas (3 primeiras posições) + restantes variáveis

bool novo_l = false;
bool to_print = false;
char ultimo_chr = '\0';
float ref = 0;

void le_i2c(int n) { //Rotina de tratamento de interrupções I2C, lê e descodifica as mensagens recebidas
  int i, j, ident;
  char tmp[NUMCHR + 1] = {0, 0, 0, 0, 0}; //+1 ('\0')
  char caso;
  bool prim_leitura = true, ident_lido = false;
  i = j = 0;

  while (Wire.available()) { // Dados disponíveis
    if (prim_leitura) {
      prim_leitura = false;
      caso = Wire.read();
      continue;
    }
    if (caso == 'e') { //Coluna da matriz de acoplamentos
      tmp[i] = Wire.read(); //Lê carater
      if (i == NUMCHR - 1) { //4º carater
        i = 0;
        E[j][col_matriz] = (atof(tmp)) * 0.1;
        j++;
      }
      else i++;
    }
    else { //Referência (vinda de outro Arduino)
      if (!ident_lido) {
        ident_lido = true;
        ident = Wire.read() - '0';
        continue;
      }
      tmp[i] = Wire.read();
      if (i == NUMCHR - 1) { //4º carater
        L[ident - 1] = (atof(tmp)) * 0.1;
        novo_l = true; //Nova referência, reiniciar simplex
      }
      i++;
    }
  }
  if (caso == 'e')
    col_matriz++;
}

void setup() {
  Serial.begin(115200); //Inicializa porta série

  //Inicialização do I2C
  Wire.begin(ADDRESS);
  TWAR = (ADDRESS << 1) | 1; //Ativa broadcasts
  Wire.setClock(400000L); //Frequência de operação máxima
  Wire.onReceive(le_i2c);

  //Inicia máquina de estados
  nchar = 1;
  estado = CALIBR;
  acao = OFF;
  col_matriz = 1;
}

//Debugging, imprime a matriz A no ecrã
void printmatriz(float A[][DIM2], int dim1, int dim2) {
  for (int i = 0; i < dim1; i++) {
    for (int j = 0; j < dim2; j++) {
      Serial.print(A[i][j]);
      Serial.print(" ");
    }
    Serial.print("\n");
  }
}

void reinicia(void) { //Põe o sistema no estado inicial (ao receber ordem de reiniciar)
  //Máquinas de estados
  nchar = 1;
  estado = CALIBR;
  acao = OFF;
  //Simplex
  col_matriz = 1;
  for (int i = 0; i < DIM; i++) //Referências a zero
    L[i] = 0;
  inicializa_pid();
  ref = 0;
}

void loop() {
  int i, tempo, tempo_ant = 0;
  char tmp[NUMCHR + 1] = {0, 0, 0, 0, 0}; //+1 ('\0')
  float sol;

  while (Serial.available() && ultimo_chr == '\0') //Remove \0's que tenham ficado por ler
    ultimo_chr = Serial.read();

  if (Serial.available() >= nchar || (Serial.available() == nchar - 1 && ultimo_chr != '\0')) {
    leitura[0] = ultimo_chr;
    ultimo_chr = '\0';
    for (i = 1; i < nchar; i++)
      leitura[i] = Serial.read();
    leitura[i] = '\0';

    switch (estado) {
      case CALIBR: //Modo de calibração
        acao = *leitura - '0';
        break;
      case LE_MATRIZ: //Aguardar a coluna da matriz de acoplamento
        for (i = 0; i < TAM_MATRIZ / 2; i++) //Guarda o recebido para retransmitir aos restantes
          col_lida[i] = leitura[i];
        col_lida[i] = '\0';
        for (i = 0; i < TAM_MATRIZ / NUMCHR; i++) {
          for (int j = 0; j < NUMCHR; j++)
            tmp[j] = leitura[NUMCHR * i + j];
          if (i < DIM)
            E[i][0] = (atof(tmp)) * 0.1;
          else
            O[i - DIM] = (atof(tmp)) * 0.1;
        }
        estado = NOVA_REF;
        nchar = TAM_REF; //Mensagens mais curtas
        acao = I2C;
        break;
      case NOVA_REF: //Aguardar novas referências
        if (*leitura == 'r') {//Recalibra
          reinicia();
          break;
        }
        if (*leitura == 'a') {//Amostra
          to_print = true;
          break;
        }
        for (i = 0; i < TAM_REF; i++)
          tmp[i] = leitura[i];
        L[ADDRESS - 1] = ((atof(tmp)) * 0.1;
        
        Wire.beginTransmission(0); //Transmite leitura
        Wire.write('r');
        Wire.write(ADDRESS + '0');
        Wire.write(tmp);
        Wire.endTransmission();
        acao = CONTROLO;
        novo_l = true;
        break;
    }
  }
  switch (acao) {
    case OFF: //Desliga o LED
      analogWrite(OUTPIN, 0);
      break;
    case ON: //Liga o LED
      analogWrite(OUTPIN, 255);
      break;
    case LE: //Amostra o sensor
      Serial.print(le_adc(), HEX);
      Serial.print('\n');
      acao = NOP;
      break;
    case FIM: //Aguarda que o servidor envie a coluna
      nchar = TAM_MATRIZ;
      estado = LE_MATRIZ;
      acao = NOP;
      break;
    case NOP: //Espera...
      break;
    case I2C:
      Wire.beginTransmission(0); //Broadcast da coluna recebida
      Wire.write('e');
      Wire.write(col_lida);
      Wire.endTransmission();
      acao = ESPERA_COLUNAS;
      break;
    case ESPERA_COLUNAS:
      if (col_matriz == 3) { //Recebeu todas as colunas
        acao = NOP;
      }
      break;
    case CONTROLO:
      tempo = micros();
      if (tempo_ant > tempo) { //Overflow da função micros
        tempo_ant = tempo;
        break;
      }
      
      if (tempo - tempo_ant > T) {
        int y_bin, u;
        float y;
        
        tempo_ant = tempo;
        if (novo_l) { //Nova referência, resolver simplex
          novo_l = false;
          inicializa_matrizes(E, L, O, A, b, var); //reinicia as matrizes para o simplex
          sol = simplex(A, b, c, var); //Resolve o simplex
          if (sol >= 0) //Problema com solução
            analogWrite(OUTPIN, map(sol*100, 0, 100, 0, 255)); //Feed-forward do simplex
          ref = L[ADDRESS - 1];
        }
        
        //Lê a saída do sistema
        y_bin = le_adc();
        y = input2lux(y_bin);

        u = compute_control(ref, y); //Calcula a atuação
        analogWrite(OUTPIN, map(u, 0, 100, 0, 255)); //Atua o sistema

        if (to_print) { //O servidor pediu para enviar dados
          to_print = false;
          Serial.print(y_bin, HEX); //iluminância
          Serial.print('.');
          Serial.print(u, HEX); //duty cycle
          Serial.print('\n');
        }
      }
      break;
  }
}
