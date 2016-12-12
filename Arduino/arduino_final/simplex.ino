//Restaura os valores originais nas matrizes e vetores utilizados pelo simplex (necessário para poder fazer as contas)
void inicializa_matrizes(float E[][DIM], float *L, float *O, float A[][DIM2], float *b, int *var) {
  const float tmpA[DIM][DIM2] = {{0, 0, 0, -1, 0, 0, 1, 0, 0}, {0, 0, 0, 0, -1, 0, 0, 1, 0}, {0, 0, 0, 0, 0, -1, 0, 0, 1}};
  const int tmpvar[DIM2] = {6, 7, 8, 0, 1, 2, 3, 4, 5};
  for(int i = 0; i < DIM; i++) {
    for (int j = 0; j< DIM2; j++)
      A[i][j] = tmpA[i][j]; //Inicializa a matriz A
    b[i] = L[i] - O[i]; //Inicializa b
  }
  for(int i = 0; i < DIM; i++)
    for(int j = 0; j < DIM; j++)
      A[i][j] = E[i][j]; //Copia a matriz de acoplamentos para a matriz A
  for(int i = 0; i < DIM2; i++)
    var[i] = tmpvar[i]; //Reinicia a lista das variáveis
}

//Copia v2 para v1
void copia_vec(float *v1, float *v2) {
  for (int i = 0; i < DIM; i++)
    v1[i] = v2[i];
}

//Calcula a combinação linear dos vetores v1 e v2, com pesos p1 e p2, respetivamente, e guarda o resultado em v1
void comb_linear_vec( float *v1, float *v2, int p1, int p2) {
  for (int i = 0; i < DIM; i++)
    v1[i] = p1*v1[i] + p2*v2[i];
}

//Retorna true se vec for lex positivo, false se não.
bool lexpositivo(float *vec, int vsize) {
  for (int i = 0; i < vsize; i++) {
    if (vec[i] == 0) //Ignorar o zero
      continue;
    if (vec[i] > 0) //Primeiro elemento não nulo é positivo => lexpositivo
      return true;
    return false; //Elemento negativo
  }
  return false; // Tudo zeros
}

//Escolha a coluna a entrar na base, ordenando de entre as possíveis por lex-positividade
int min_lexc(int &pos, float A[][DIM2], int *c, int *var) {
  int min_var = -1;
  float ch;
  float vec[2+DIM], min_vec[2+DIM], tmp_vec[2+DIM];
    
  for (int i=DIM;i<DIM2;i++) { //Só percorre as que estão fora
    ch = 0;
    for (int x=0;x<DIM;x++) { //Custo reduzido
      vec[x+1] = -A[x][var[i]];
      ch += A[x][var[i]]*c[var[x]];
    }
    ch = c[var[i]] - ch;
    vec[0] = ch;
    vec[DIM-1] = 1;
    
    if (min_var == -1) { //Primeira variável
      copia_vec(min_vec, vec);
      min_var = var[i];
      pos = i;
    }
    else {
      copia_vec(tmp_vec, vec);
      comb_linear_vec(tmp_vec, min_vec, -1, 1); //tmp_vec = min_vec - tmp_vec
      if (lexpositivo(tmp_vec, DIM)) {
        copia_vec(min_vec, vec);
        min_var = var[i];
        pos = i;
      }
    }
  }
  if (lexpositivo(min_vec, DIM))
    min_var = -1;
  return min_var;
}

//Escolhe a coluna a sair, utilizando o critério da lex-positividade
int lexleaving(int &llinha, float A[][DIM2], float *b, int e, int *var) {
  int linha, l;
  float j[1+DIM], minj[1+DIM], tmp_j[1+DIM];
  llinha = -1;
  for(int i=0;i<DIM;i++) {
    if (A[i][e] < 0)
      continue;
    j[0] = b[i]/A[i][e];
    for (int x = 0; x <DIM; x++)
      j[x+1] = (x == i) ? 1.0/A[i][e] : 0;
    
    if (llinha == -1) { // 1ª iteração
      l = var[i];
      llinha = i;
      copia_vec(minj, j);
    }
    else {
      copia_vec(tmp_j, minj);
      comb_linear_vec(tmp_j, j, 1, -1);
      if (lexpositivo(tmp_j, DIM)) {
        l = var[i];
        copia_vec(minj, j);
        llinha = i;
      }
    }
  }
  if (!lexpositivo(minj, DIM)) {
    l = -1;
    llinha = -1;
  }
  return l;
}

//Troca as entradas i e j do vetor vec
void swap(int *vec, int i, int j) {
  int tmp = vec[j];
  vec[j] = vec[i];
  vec[i] = tmp;
}

//Aplica o algoritmo de simplex a resolver o problema de otimização dado por min(c'*d) s. t. A*d = b. Retorna a solução para este Arduino ou INFEASIBLE ou UNBOUNBED quando não existe
float simplex(float A[][DIM2], float *b, int *c, int *var) {
  int e, l, pos, llinha;
  float tmp;
  while(true){
    e = min_lexc(pos, A, c, var); //Coluna a entrar
    if (e == -1) {
      for (int i = 0; i< DIM; i++)
        if ((var[i] != 0 && var[i] != 1 && var[i] != 2) || b[i] > 1) //Estabilizou numa solução que contém variáveis auxiliares
          return INFEASIBLE;
      return b[ADDRESS-1]; //Solução encontrada
    }
    l = lexleaving(llinha, A, b, e, var); //Variável a sair
    if (l == -1)
      return UNBOUNDED;
    
    swap(var, pos, llinha); //Troca a variável que entra pela que sai
    
    //Eliminação de Gauss
    b[llinha] = b[llinha]/A[llinha][e];
    tmp = A[llinha][e];
    for (int i= 0; i<DIM2;i++)
      A[llinha][i] /= tmp;
    for (int i = 0; i<DIM;i++) {
      if (i == llinha)
        continue;
      b[i] = b[i] - b[llinha]*A[i][e];
      tmp = A[i][e];
      for (int j = 0; j< DIM2; j++)
        A[i][j] = A[i][j] - A[llinha][j]*tmp;
    }
  }
}
