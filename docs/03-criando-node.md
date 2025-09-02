# 03 - Criando Primeiros Nodes ROS 2

Neste capítulo vamos criar dois pacotes de exemplo:

* **Python** (`my_py_pkg`)
* **C++** (`my_cpp_pkg`)

Ambos terão um nó simples que imprime "Hello World!" periodicamente.

---

## 1. Criando um pacote em Python

Dentro do workspace:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclpy
```

Isso gera a seguinte estrutura:

```
my_py_pkg/
├── package.xml
├── setup.cfg
├── setup.py
├── resource/
├── my_py_pkg/
│   └── __init__.py
└── test/
```

---

## 2. Criando um Nó em Python

Crie o arquivo `my_py_pkg/my_first_node.py` com o seguinte conteúdo:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

# Criamos uma classe chamada MyNode que herda de Node.
class MyNode(Node):
    def __init__(self):
        # Inicializa o nó com o nome "py_test"
        super().__init__("py_test")

        # Cria uma variável contador
        self.counter_ = 0

        # Escreve no log do ROS 2 (equivalente a print, mas integrado ao sistema de logs)
        self.get_logger().info("Hello World !")

        # Cria um timer que chama a função timer_callback a cada 1 segundo
        self.create_timer(1.0, self.timer_callback)

    # Função chamada pelo timer a cada 1 segundo
    def timer_callback(self):
        # Mostra o valor do contador no log
        self.get_logger().info("Hello " + str(self.counter_))
        # Incrementa o contador
        self.counter_ += 1


def main(args=None):
    # Inicializa a comunicação com o ROS 2
    rclpy.init(args=args)

    # Cria o nó
    node = MyNode()

    # Mantém o nó em execução até ser encerrado
    rclpy.spin(node)

    # Finaliza a comunicação com o ROS 2
    rclpy.shutdown()


# Garante que a função main só seja chamada se o arquivo for executado diretamente
if __name__ == "__main__":
    main()
```

---

## 3. Arquivo `setup.py`

Crie `setup.py` dentro da pasta `my_py_pkg/`:

```python
from setuptools import find_packages, setup

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
         # Necessário para que o ROS 2 encontre o pacote
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        # Inclui o package.xml
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rafael',
    maintainer_email='rafael@todo.todo',
    description='Meu primeiro pacote em Python no ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Aqui registramos o executável do nó
            # Formato: "<nome-do-executável> = <pacote>.<arquivo>:<função>"
            "py_node = my_py_pkg.my_first_node:main"
        ],
    },
)
```

---

## 4. Compilando e executando o pacote Python

1. Vá até a raiz do workspace:

```bash
cd ~/ros2_ws
```

2. Compile usando o **colcon**:

```bash
colcon build --packages-select my_py_pkg
```

3. Carregue as configurações do workspace compilado:

```bash
source install/setup.bash
```

4. Execute o nó:

```bash
ros2 run my_py_pkg py_node
```

Saída esperada:

```
[INFO] [1681234567.123456789] [py_test]: Hello World !
[INFO] [1681234568.123456789] [py_test]: Hello 0
[INFO] [1681234569.123456789] [py_test]: Hello 1
...
```

---

## 5. Criando um Nó em C++ (my\_cpp\_pkg)

Se ainda não criou o pacote:

```bash
cd ~/ros2_ws/src
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```

Estrutura gerada:

```
my_cpp_pkg/
├── CMakeLists.txt
├── package.xml
├── include/
└── src/
```

---

### 5.1. Arquivo `src/cpp_node.cpp`

```cpp
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

```

---

### 5.2. CMakeLists.txt

```cmake
cmake_minimum_required(VERSION 3.5)
project(my_cpp_pkg)
# Padrões de compilação (opcional, mas recomendado)
if(NOT CMAKE_CXX_STANDARD) set(CMAKE_CXX_STANDARD 17) endif()

# Encontrar dependências
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

 # Criar executável a partir do fonte
add_executable(cpp_node src/cpp_node.cpp)

# Lincar dependências ao executável
ament_target_dependencies(cpp_node rclcpp)

# Instalar o executável para que 'ros2 run' o encontre
install(TARGETS
    cpp_node
    DESTINATION lib/${PROJECT_NAME})

ament_package()
```

---

### 5.3. Compilando e executando o pacote C++

```bash
cd ~/ros2_ws
colcon build --packages-select my_cpp_pkg
source install/setup.bash
ros2 run my_cpp_pkg cpp_node
```

Saída esperada:

```
[INFO] [<tempo>] [cpp_test]: Hello World !
[INFO] [<tempo>] [cpp_test]: Hello World 0
[INFO] [<tempo>] [cpp_test]: Hello World 1
...
```

---
