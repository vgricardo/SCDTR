//Compilar:  g++ -std=c++11 servidor.cpp com_serv.cpp -lpthread -lboost_system -o servidor

#include <iostream>
#include <cmath>
#include <boost/bind.hpp>
#include <boost/asio.hpp>
#include <pthread.h>
#include <semaphore.h>
#include "com_serv.h"

//Porta TCP
#define PORTA 17000

//Endereços das portas série
#define SIMAO "/dev/ttyACM0"
#define JOTA "/dev/ttyACM1"
#define TELMO "/dev/ttyACM2"

//Mensagens para calibração dos Arduinos
#define OFF "0"
#define ON "1"
#define LE "2"
#define FIM "3"

//Outras constantes
#define JOAO_PESTANA 100000
#define T 0.3
#define BUE 1000
#define MAX_STR 4

//Variáveis para cálculo das estatísticas
extern float ilum[NPORTAS][IL_ANT];
extern int pos;
extern bool nova_ref[NPORTAS];
extern bool reinicia;
extern int n_amostras;
extern int duty[NPORTAS];
extern int ocup[NPORTAS];
extern float referencia[NPORTAS];
extern float min_ilum[NPORTAS];
extern float O[NPORTAS];
extern float energia[NPORTAS];
extern float erro_conf[NPORTAS];
extern float var[NPORTAS];
extern float ilum_ref;

//Semáforo para sincronismo
extern sem_t semaforo;

using namespace boost::asio;
using namespace std;

//Thread para correr o servidor
void *thread_servidor(void *arg) {
	boost::asio::io_service servico_serv;
	server s(servico_serv, PORTA);
	servico_serv.run();
}

//Conversão de hexadecimal para lux (os dados dos arduinos vêm em hexadecimal)
float hex2lux(const char* s) {
	const float rbase = 65078.575;
	const float expo = 1.4241;
	
	float r = strtol(s, NULL, 16)*5.0/1024; //Converte para decimal
	return pow(rbase/(50000/r - 10000), expo); //Converte para lux, através da fórmula exponencial calculada na 1ª parte do projeto
}

//Converte os valores lidos dos Arduinos (y e u, em hexadecimal) para as unidades corretas (lux e %)
void amostra_arduino(float &y, int &u, string s) {
	char y_str[MAX_STR], u_str[MAX_STR];
	int i, j;
	
	for (i=0; i<s.length();i++) {
		if (s.c_str()[i] == '.') //Os campos são separados por um '.'
			break;
		y_str[i] = s.c_str()[i];
	}
	y_str[i++] = '\0';
	for (j = 0; i<s.length();i++) {
		if (s.c_str()[i] == '\n')
			break;
		u_str[j++] = s.c_str()[i];
	}
	y = hex2lux(y_str); //Conversão para lux
	u = strtol(u_str, NULL, 16); //Conversão para decimal
}

//Função para reiniciar (recalibrar) os Arduinos
void reinicia_sistema(serial_port **sp) {
	boost::asio::streambuf leitura;
	float E[NPORTAS][NPORTAS];
	
	//Inicializa variáveis
	pos = 0;
	n_amostras = 0;
	reinicia = false;
	ilum_ref = 70;
	for(int i = 0; i< NPORTAS; i++) {
		nova_ref[i] = false;
		duty[i] = 0;
		ocup[i] = 0;
		referencia[i] = 0;
		min_ilum[i] = 0;
		energia[i] = 0;
		erro_conf[i] = 0;
		var[i] = 0;
	}
	
	cout << "A iniciar a calibração...\n";	
	for (int i = 0; i<NPORTAS;i++) { //Lê o vetor das iluminâncias exteriores (todos os Arduinos desligados)
		write(*sp[i], boost::asio::buffer(LE));
		read_until(*sp[i], leitura, '\n');
		string s((std::istreambuf_iterator<char>(&leitura)), std::istreambuf_iterator<char>()); //Converte para std::string
		O[i] = hex2lux(s.c_str());
	}
	
	for (int i = 0; i<NPORTAS;i++){ //Liga, alternadamente, cada Arduino e lê as iluminâncias em cada sensor
		write(*sp[i], boost::asio::buffer(ON));
		usleep(JOAO_PESTANA); //Pausa de 0.1s
		for (int j = 0; j<NPORTAS;j++) {
			write(*sp[j], boost::asio::buffer(LE));
			read_until(*sp[j], leitura, '\n');
			string s((std::istreambuf_iterator<char>(&leitura)), std::istreambuf_iterator<char>()); //Converte para std::string
			E[j][i] = hex2lux(s.c_str()) - O[j]; //Subtrai o vetor das iluminâncias exteriores
		}
		write(*sp[i], boost::asio::buffer(OFF));
	}
	
	for(int i=0;i<NPORTAS;i++) { //Mostra a matriz de acoplamento no ecrã
		for (int j=0;j<NPORTAS;j++)
			cout << E[i][j] << " ";
		cout << endl;
	}
	
	//Envia a coluna da matriz de calibração e o vetor das iluminâncias externas para cada Arduino
	char conv[MAX_STR+1];
	for (int i = 0; i < NPORTAS; i++) {
		write(*sp[i], boost::asio::buffer(FIM));
		usleep(JOAO_PESTANA); //Pausa de 0.1s
		for (int j = 0; j <NPORTAS; j++) {
			sprintf(conv, "%04.0f",10*E[j][i]);
			string s(conv);
			write(*sp[i], boost::asio::buffer(s));
		}
		for (int j = 0; j <NPORTAS; j++) {
			sprintf(conv, "%04.0f",10*O[j]);
			string s(conv);
			write(*sp[i], boost::asio::buffer(s));
		}
	}
	cout << "Calibração concluída.\n";
	
	//Todos desligados (inicialmente todas as referências são nulas)
	for (int i = 0; i <NPORTAS; i++) {
		usleep(JOAO_PESTANA); //Pausa de 0.1s
		sprintf(conv, "%04.0f",10*referencia[i]);
		string s(conv);
		write(*sp[i], boost::asio::buffer(s));
	}
}

//Função para cálculo do maior de 2 números
float max(float x1, float x2) {
	return (x1 > x2) ? x1 : x2;
}

int main(void) {
	//Inicializa os semáforos (para garantir sincronismo)
	sem_init(&semaforo, 0, 1);
	
	//Lança o servidor
	pthread_t tservidor;
	pthread_create(&tservidor, NULL, thread_servidor, NULL);
	
	//Abre as portas série
	cout << "A abrir as portas série...\n";
	string nomes[NPORTAS] = {SIMAO, JOTA, TELMO};
	boost::asio::io_service io_serial[NPORTAS];
	boost::system::error_code ec;
	serial_port *sp[NPORTAS];
	for (int i=0; i< NPORTAS;i++) {
		sp[i] = new serial_port(io_serial[i]);
		sp[i]->open(nomes[i], ec);
		if (ec) {
			cout << "Erro ao abrir a porta série!\n";
			exit(-1);
		}
		
		sp[i]->set_option(serial_port_base::baud_rate(115200), ec);
		if (ec) {
			cout << "Erro ao abrir a porta série!\n";
			exit(-1);
		}
	}
	cout << "A aguardar pelos Arduinos...\n";
	sleep(3);
	
	//Calibra os arduinos
	reinicia_sistema(sp);
	
	cout << "A aguardar comandos...\n";
	boost::asio::streambuf leitura;
	while (true) {
		usleep(3*JOAO_PESTANA); //300 ms
		
		sem_wait(&semaforo); //Bloqueia o acesso da thread que lida com os clientes às estruturas de dados
		
		n_amostras++;
		for (int i = 0; i <NPORTAS; i++) {
			char conv[MAX_STR+1];
			if (nova_ref[i]) { //Alterado o estado de um dos LEDs (pelo cliente)
				sprintf(conv, "%04.0f",10*referencia[i]);
				string s(conv);
				write(*sp[i], boost::asio::buffer(s)); //Envio da nova referência
			}
		}
		
		for (int i = 0; i<NPORTAS;i++) {
			energia[i] += duty[i]/100.0*T; //Cálculo da energia (utiliza o duty cycle da iteração anterior)
			
			//Pedido de amostragem dos Arduinos
			write(*sp[i], boost::asio::buffer("amos"));
			read_until(*sp[i], leitura, '\n');
			
			//Lê a resposta de cada Arduino
			string s((std::istreambuf_iterator<char>(&leitura)), std::istreambuf_iterator<char>()); //Converte para std::string
			amostra_arduino(ilum[i][pos], duty[i], s); //Converte para lux e %
			
			//Cálculo das estatísticas necessárias
			erro_conf[i] = (n_amostras-1.0)/n_amostras*erro_conf[i]+1.0/n_amostras*max(referencia[i]-ilum[i][pos], 0);
			
			var[i] = (n_amostras - 1.0)/n_amostras*var[i] +
			1.0/(n_amostras*pow(T,2))*fabs(ilum[i][pos]-2*ilum[i][(pos-1+IL_ANT)%IL_ANT]+ilum[i][(pos-2+IL_ANT)%IL_ANT]);
			if (ilum[i][pos] < min_ilum[i])
				min_ilum[i] = ilum[i][pos];
			
			if (nova_ref[i]) {
				nova_ref[i] = false;
				min_ilum[i] = BUE; //Atualiza mínimo
			}
		}
		pos++;
		if (pos >= IL_ANT)
			pos = 0;
		if (reinicia) { //Pedido para reiniciar o sistema
			for (int i = 0; i<NPORTAS;i++)
				write(*sp[i], boost::asio::buffer("rein"));
			reinicia_sistema(sp);
			reinicia = false;
		}
		sem_post(&semaforo); //Concluída a alteração das estruturas de dados, volta a permitir o acesso a estas por parte da thread que lida com os clientes
	}
	
	for(int i=0; i<NPORTAS;i++) //Fecha as portas série
		sp[i]->close();
	exit(0);
}
