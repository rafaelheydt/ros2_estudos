// Biblioteca principal do ROS 2 em C++
#include "rclcpp/rclcpp.hpp"

// Definição de uma classe que herda de rclcpp::Node
// Cada nó em ROS 2 é representado como uma classe/objeto
class MyNode : public rclcpp::Node
{
public: 
    // Construtor da classe
    // O parâmetro "cpp_test" é o nome do nó no grafo do ROS 2
    MyNode() : Node("cpp_test"), counter_(0)
    {
        // Exibe mensagem no terminal (log do ROS 2)
        RCLCPP_INFO(this->get_logger(), "Hello World !");

        // Cria um temporizador (timer) que dispara a cada 1 segundo
        // e chama a função timer_callback()
        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),   // intervalo do timer
            std::bind(&MyNode::timer_callback, this) // função callback
        );
    };

private:
    // Função que será chamada periodicamente pelo timer
    void timer_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello World %d", counter_);
        counter_++; // incrementa o contador a cada execução
    }

    // Ponteiro para o objeto Timer
    rclcpp::TimerBase::SharedPtr timer_;
    // Contador de execuções do callback
    int counter_;
};

// Função principal (ponto de entrada do programa)
int main(int argc, char **argv)
{
    // Inicializa o ROS 2
    rclcpp::init(argc, argv);

    // Cria o nó "MyNode" e o coloca dentro de um ponteiro inteligente (shared_ptr)
    auto node = std::make_shared<MyNode>();

    // Mantém o nó em execução (loop interno que processa callbacks)
    rclcpp::spin(node);

    // Encerra o ROS 2 quando terminar
    rclcpp::shutdown();
    return 0;
}
