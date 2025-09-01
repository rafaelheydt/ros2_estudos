# 02 - Criando um Workspace ROS 2

Este tutorial descreve como **criar e configurar um workspace ROS 2**, compilá-lo e testar com exemplos simples.

## 1. Estrutura do Workspace

Um workspace ROS 2 é uma pasta que contém os pacotes que você vai desenvolver:

```
ros2_ws/
├── src/     # Onde ficam seus pacotes
├── install/ # Arquivos gerados após a compilação
└── build/   # Diretório de build temporário
```

## 2. Criando a pasta do workspace

```bash
mkdir ros2_ws/src
cd ros2_ws
```

## 3. Compilando o workspace

Mesmo vazio:
```bash
colcon build
source install/setup.bash
```
Dica: adicione ```source ~/ros2_ws/install/setup.bash``` no ~/.bashrc.

## 4. Criando Pacotes

### 4.1 Pacote Python
Dentro do workspace (`ros2_ws/src`):
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_py_pkg
```
Podem ser adicionadas dependencias no momento de criação do pacote com:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclpy
```

Isso cria a estrutura:
```
my_py_pkg/
├── package.xml
├── setup.py
└── my_py_pkg/
    └── __init__.py
````

### 4.2 Pacote C++
Dentro do workspace (`ros2_ws/src`):
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake my_cpp_pkg
```
Podem ser adicionadas dependencias no momento de criação do pacote com:
```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclcpp
```

Estrutura Criada:
```
my_cpp_pkg/
├── CMakeLists.txt
├── package.xml
└── src/
``` 
## Compilando pacotes
```
cd ~/ros2_ws
colcon build --packages-select my_cpp_pkg my_py_pkg
source install/setup.bash
```


