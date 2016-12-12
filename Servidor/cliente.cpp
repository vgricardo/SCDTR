//Compilar com: g++ -std=c++11 cliente.cpp -lpthread -lboost_system -o cliente
//Correr: ./cliente 127.0.0.1

#include <cstdlib>
#include <cstring>
#include <iostream>
#include <boost/asio.hpp>

#define PORTA "17000" //Porta TCP para comunicação com o servidor

using boost::asio::ip::tcp; //Módulos de TCP do boost

enum { max_length = 1024 }; //Dimensão máxima da mensagem

int main(int argc, char* argv[]) {
	try {
		if (argc != 2) { //Erro na chamada do cliente
			std::cerr << "Utilização: cliente <host>\n";
			return 1;
		}

		boost::asio::io_service io_service; //Serviço para comunicação

		tcp::resolver resolver(io_service);
		
		//Verifica a conetividade do servidor especificado (argv[1]) através da porta 17000
		tcp::resolver::query query(tcp::v4(), argv[1], PORTA);
		tcp::resolver::iterator iterator = resolver.resolve(query);

		tcp::socket s(io_service);
		boost::asio::connect(s, iterator);

		using namespace std;
		
		std::cout << "Mensagem: ";
		char request[max_length];
		std::cin.getline(request, max_length); //Lê a string do utilizador
		size_t request_length = std::strlen(request);
		boost::asio::write(s, boost::asio::buffer(request, request_length)); //Envia a string para o servidor

		char reply[max_length];
		
		boost::asio::streambuf leitura;
		size_t reply_length = boost::asio::read_until(s,
		leitura, '\n'); //Lê a resposta do servidor
		std::cout << "Resposta: " << &leitura; //Escreve a resposta no ecrã
	}
	catch (std::exception& e) { //Ocorreu um erro na ligação/comunicação
		std::cerr << "Erro: " << e.what() << "\n";
	}
	return 0;
}
