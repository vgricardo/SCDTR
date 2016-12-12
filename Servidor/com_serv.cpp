#include <cstdio>
#include <semaphore.h>
#include <ctype.h>
#include "com_serv.h"

#define T 0.3 //Período de amostragem, em segundos

//Variáveis para cálculo das estatísticas
float ilum[NPORTAS][IL_ANT];
int pos;
bool nova_ref[NPORTAS];
bool reinicia;
int n_amostras;
int duty[NPORTAS];
int ocup[NPORTAS];
float referencia[NPORTAS];
float min_ilum[NPORTAS];
float O[NPORTAS];
float energia[NPORTAS];
float erro_conf[NPORTAS];
float var[NPORTAS];

float ilum_ref;

sem_t semaforo; //Semáforo, para sincronismo


session::session(boost::asio::io_service& io_service) : socket_(io_service) {;} //Inicializador da classe, cria uma nova sessão (para um novo cliente)
tcp::socket& session::socket() { //Cria um novo socket
	return socket_;
}

void session::start() { //Inicia uma nova sessão assíncrona
		socket_.async_read_some(boost::asio::buffer(data_, MAX_LENGTH),
		boost::bind(&session::handle_read, this,
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred));
}

void session::handle_read(const boost::system::error_code& error, size_t bytes_transferred) { //Novo pedido de um cliente
	char chr, t, str[MAX_LENGTH];
	std::string resposta;
	int n, estado;
	float nova_ilum;
	sem_wait(&semaforo); //Impede o acesso por parte do ciclo principal (que comunica com os Arduinos)
											 //às estruturas de dados que armazenam as estatísticas

	//Processa a mensagem recebida
	if (*data_ == 'r' && !isalnum(data_[1])) {// Reiniciar o sistema
		resposta = ACK; //reinicia
		reinicia = true;
	}
	else if (sscanf(data_, "g %c %d", &chr, &n) == 2) { //Pedido de GET individual
		if (n > NPORTAS || n <= 0) { //Pedido inválido
			resposta = INVALIDA;
		}
		else {
			switch(chr) {
				case 'l': //g l ... (pedido de iluminânicia)
					sprintf(str, "l %d %.1f\n", n, ilum[n-1][pos]);
					resposta = std::string(str);
					break;
				case 'd': //g d ... (pedido de duty cycle)
					sprintf(str, "d %d %d\n", n, duty[n-1]);
					resposta = std::string(str);
					break;
				case 'o': //g o ... (pedido de estado de ocupação)
					sprintf(str, "o %d %d\n", n, ocup[n-1]);
					resposta = std::string(str);
					break;
				case 'L': //g L ... (pedido de mínimo de iluminância)
					sprintf(str, "L %d %.1f\n", n, min_ilum[n-1]);
					resposta = std::string(str);
					break;
				case 'O': //g O ... (pedido de iluminância externa)
					sprintf(str, "O %d %.1f\n", n, O[n-1]);
					resposta = std::string(str);
					break;
				case 'r': //g r ... (pedido de referência)
					sprintf(str, "r %d %.1f\n", n, referencia[n-1]);
					resposta = std::string(str);
					break;
				case 'p': //g p ... (pedido do consumo de potência)
					sprintf(str, "p %d %.1f\n", n, duty[n-1]/100.0);
					resposta = std::string(str);
					break;
				case 'e': //g e ... (pedido do consumo de energia)
					sprintf(str, "e %d %.1f\n", n, energia[n-1]);
					resposta = std::string(str);
					break;
				case 'c': //g c ... (pedido do erro de conforto)
					sprintf(str, "c %d %.1f\n", n, erro_conf[n-1]);
					resposta = std::string(str);
					break;
				case 'v': //g v ... (pedido da variância)
					sprintf(str, "v %d %.1f\n", n, var[n-1]);
					resposta = std::string(str);
					break;
				default: //Pedido inválido
					resposta = INVALIDA;
					break;
			}
		}
	}
	else if (sscanf(data_, "g %c %c", &chr, &t) == 2) { //Pedido de GET para todos (T)
		if (t != 'T') { //Pedido inválido
			resposta = INVALIDA;
		}
		else {
			switch(chr) {
				case 'p': //g p T (pedido de consumo de potência total)
					sprintf(str, "p T %.1f\n", (duty[0]+duty[1]+duty[2])/100.0);
					resposta = std::string(str);
					break;
				case 'e': //g e T (pedido de consumo de energia total)
					sprintf(str, "e T %.1f\n", energia[0]+energia[1]+energia[2]);
					resposta = std::string(str);
					break;
				case 'c': //g c T (pedido de erro de conforto total)
					sprintf(str, "c T %.1f\n", erro_conf[0]+erro_conf[1]+erro_conf[2]);
					resposta = std::string(str);
					break;
				case 'v': //g v T (pedido de variância total)
					sprintf(str, "v T %.1f\n", var[0]+var[1]+var[2]);
					resposta = std::string(str);
					break;
				case 'n': //g n T (pedido do número da amostras até ao instante atual - feature adicional (não pedido no enunciado))
					sprintf(str, "n %d\n", n_amostras);
					resposta = std::string(str);
					break;
				case 't': //g t T (pedido do tempo deste o último reinício, em segundos - feature adicional (não pedido no enunciado))
					sprintf(str, "t %.1f\n", n_amostras*T);
					resposta = std::string(str);
					break;
				case 'R': //g R T (pedido da referência (em lux) dada aos Arduinos)
					sprintf(str, "R %.0f\n", ilum_ref);
					resposta = std::string(str);
					break;
				default: //Pedido inválido
					resposta = INVALIDA;
					break;
			}
		}
	}
	else if (sscanf(data_, "s %d %d", &n, &estado) == 2) { //Pedido de SET (definir) estado de ocupação
		if (n > NPORTAS || n <= 0 || (estado != 0 && estado != 1)) { //Pedido inválido (nº de arduino ou estado inválido)
			resposta = INVALIDA;
		}
		else {
			resposta = ACK;
			ocup[n-1] = estado;
			referencia[n-1] = estado*ilum_ref; //Define a nova referência. Na próxima amostragem (no main), a referência será pedida ao Arduino correspondente
			nova_ref[n-1] = true; //flag para sinalizar a existência de novo pedido
		}
	}
	else if (sscanf(data_, "s R %f", &nova_ilum) == 1) { //Pedido de SET (definir) um novo valor para as referências (em lux)
		if (nova_ilum < 0) //Referências negativas são inválidas
			resposta = INVALIDA;
		else {
			ilum_ref = nova_ilum; //Armazena o valor da nova referência
			resposta = ACK;
		}
	}
	else
		resposta = INVALIDA;

	sem_post(&semaforo); //Desbloqueia o acesso às estruturas que armazenam as estatísticas

	for(int i = 0; i<MAX_LENGTH;i++) //Limpa o buffer de leitura. Necessário devido a bug da biblioteca boost.
		data_[i] = '\0';

	if (!error) { //Caso não haja erros de comunicação, envia a resposta para o cliente
		boost::asio::async_write(socket_,
		boost::asio::buffer(resposta, resposta.length()),
		boost::bind(&session::handle_write, this,
		boost::asio::placeholders::error));
	}
	else {
		delete this;
	}
}

void session::handle_write(const boost::system::error_code& error) {
	if (!error) {
		socket_.async_read_some(boost::asio::buffer(data_, MAX_LENGTH),
		boost::bind(&session::handle_read, this,
		boost::asio::placeholders::error,
		boost::asio::placeholders::bytes_transferred));
	}
	else {
		delete this;
	}
}

server::server(boost::asio::io_service& io_service, short port) : //Construtor da classe server
	io_service_(io_service), acceptor_(io_service, tcp::endpoint(tcp::v4(), port)) {
		start_accept();
}

void server::start_accept() { //Accept de um novo cliente
	session* new_session = new session(io_service_);
	acceptor_.async_accept(new_session->socket(),
	boost::bind(&server::handle_accept, this, new_session,
	boost::asio::placeholders::error));
}

void server::handle_accept(session* new_session, const boost::system::error_code& error) { //Inicia uma nova sessão para a nova ligação recebida
	if (!error) {
		new_session->start();
	}
	else {
		delete new_session;
	}
	start_accept();
}
