#ifndef COM_SERV
#define COM_SERV

#include <boost/bind.hpp>
#include <boost/asio.hpp>

#define MAX_LENGTH 1024
#define NPORTAS 3 //Nº de Arduinos
#define IL_ANT 3 //Para cálculo das variâncias, são necessárias 3 amostras: a atual e as 2 anteriores
#define INVALIDA "QUERY_INVALIDA\n" //Resposta a pedido inválido
#define ACK "ack\n" //Resposta de acknowledge

using boost::asio::ip::tcp;

//Classe para lidar com os clientes
class session {
	public:
		session(boost::asio::io_service& io_service);
		tcp::socket& socket();
		void start();
	private:
		void handle_read(const boost::system::error_code& error, size_t bytes_transferred);
		void handle_write(const boost::system::error_code& error);

		tcp::socket socket_;
		char data_[MAX_LENGTH]; //String onde é armazenada a resposta para o cliente
};

//Classe que implementa o servidor
class server {
	public:
		server(boost::asio::io_service& io_service, short port);
	private:
		void start_accept();
		void handle_accept(session* new_session, const boost::system::error_code& error);
		boost::asio::io_service& io_service_;
		tcp::acceptor acceptor_;
};

#endif
