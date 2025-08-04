#include "rclcpp/rclcpp.hpp"                     // Biblioteca principal do ROS2 (C++)
#include "nav_msgs/msg/odometry.hpp"             // Mensagem de odometria do ROS2

#include <websocketpp/config/asio_no_tls.hpp>    // Configuracao do WebSocket (sem TLS/SSL)
#include <websocketpp/server.hpp>                // Servidor WebSocket

#include <jsoncpp/json/json.h>                   // Biblioteca para manipular JSON
#include <set>                                    // Para armazenar conexoes ativas
#include <thread>                                 // Para rodar o servidor em thread separada
#include <mutex>                                  // Para proteger conexoes em ambiente multithread

using websocketpp::connection_hdl;
using websocketpp::config::asio;
using websocketpp::server;

class ROS2WebSocketBridge : public rclcpp::Node { // Classe para fazer a ponte entre o ROS e a UE5, que herda Node (no') do ROS

public:
    ROS2WebSocketBridge() : Node("ros2_unreal_bridge") {    // (Construtor) Inicializa o no' e o servidor WebSocket

        ws_server.init_asio();           // Configura o servidor para usar ASIO (I/O assincrono)

        // Handler para quando um cliente se conecta                 // handler -> analago a um callback, controle remoto
        ws_server.set_open_handler([this](connection_hdl hdl) {      // hdl -> representa uma conexao
            std::lock_guard<std::mutex> lock(connection_mutex);      // Uso de 1 thread
            connections_.insert(hdl);                                // Adiciona nova conexao ativa // metodo de set
            });

        // Registra handler de encerramento de conexao
        ws_server.set_close_handler([this](connection_hdl hdl) {
            std::lock_guard<std::mutex> lock(connection_mutex);     // Uso de 1 thread
            connections_.erase(hdl);                                  // Remove conexao encerrada // metodo de set
            });

        ws_server.listen(8080);         // Escuta na porta 8080
        ws_server.start_accept();       // Comeca a aceitar conexoes

        odom_sub = this->create_subscription<nav_msgs::msg::Odometry>( // Inscricao no topico de odometria
            "/rov/odometry", 10,                                       // Topico e tamanho da fila
            [this](const nav_msgs::msg::Odometry::SharedPtr msg) {     // funcao lambda (sem nome)
                this->odom_callback(msg);                              // lambda encaminha a "msg" para 'odom_callback' ao ser recebida
            });

        ws_thread = std::thread([this]() {   // separa uma thread para rodar o servidor WebSocket
            ws_server.run();                  // inicia o loop do servidor WebSocket
            });
    }

    ~ROS2WebSocketBridge() {           // Destrutor para encerrar thread e conexoes de forma segura
        ws_server.stop_listening();    // Para de aceitar novas conexoes
        ws_server.stop();              // Encerra conexoes ativas

        if (ws_thread.joinable()) {   // verifica se a thread esta rodando
            ws_thread.join();         // Aguarda thread do servidor encerrar
        }
    }

private:
    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) { // Funcao para montar JSON e enviar

        Json::Value json_data;  // Objeto JSON para armazenar os dados em uma estrutura generica

        json_data["position"]["x"] = msg->pose.pose.position.x; // Preenche os campos de posicao/orientacao a partir da mensagem ROS2
        json_data["position"]["y"] = msg->pose.pose.position.y;
        json_data["position"]["z"] = msg->pose.pose.position.z;
        json_data["orientation"]["x"] = msg->pose.pose.orientation.x;
        json_data["orientation"]["y"] = msg->pose.pose.orientation.y;
        json_data["orientation"]["z"] = msg->pose.pose.orientation.z;
        json_data["orientation"]["w"] = msg->pose.pose.orientation.w;

        Json::StreamWriterBuilder builder;                                         // configura a formatacao do json
        const std::string json_real_data = Json::writeString(builder, json_data);  // Converte o JSON para string e armazena em json_real_data

        std::lock_guard<std::mutex> lock(connection_mutex);  // Protege envio multithread

        for (auto hdl : connections_) {     // Envia para todos os clientes conectados
            try {
                ws_server.send(hdl, json_real_data, websocketpp::frame::opcode::text);  // Envia a string especificando que e' um texto e nao binario
            }
            catch (const websocketpp::exception& e) { // Tratamento de excessao para erro de envio
                RCLCPP_WARN(this->get_logger(), "Erro ao enviar WebSocket: %s", e.what());  
            }
        }
    }

    // Atributos
    server<asio> ws_server;  // Instancia do servidor WebSocket
    std::thread ws_thread;   // Thread onde o servidor roda
    std::mutex connection_mutex;  // Protege o acesso a 'connections_' entre threads
    std::set<connection_hdl, std::owner_less<connection_hdl>> connections_;  // Armazena conexoes ativas (padrao websocket++)
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub;  // Assinatura do topico ROS2
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);  // Inicializa o ROS2
    auto node = std::make_shared<ROS2WebSocketBridge>();  // Cria a instancia do no'
    rclcpp::spin(node);  // Mantem o no' ativo
    rclcpp::shutdown();  // Encerra o ROS2
    return 0;
}
