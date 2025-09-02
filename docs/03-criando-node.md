# 03 - Criando Primeiros Pacotes ROS 2

Neste capítulo vamos criar dois pacotes de exemplo:

- **Python** (`my_py_pkg`)
- **C++** (`my_cpp_pkg`)

Ambos terão um nó simples que imprime "Hello World!" periodicamente.

---

## 1. Criando um pacote em Python

Dentro do workspace:

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_py_pkg --dependencies rclpy
```

Isso gera a seguinte estrutura:
```bash
my_py_pkg/
├── package.xml
├── setup.cfg
├── setup.py
├── resource/
├── my_py_pkg/
│   └── __init__.py
└── test/
```

## 2 Criando um Nó em Python

Agora vamos criar nosso **primeiro nó em Python** no ROS 2.  
Esse nó será bem simples: um contador que imprime mensagens no terminal a cada 1 segundo.  

### Código do nó em Python

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
### Arquivo `setup.py`

Para que o ROS 2 reconheça e instale nosso pacote Python, precisamos criar um arquivo chamado **`setup.py`** dentro da pasta do pacote (`my_py_pkg/`).  

Esse arquivo segue o padrão do **setuptools**, e informa ao ROS 2 quais scripts devem ser executáveis.  

Exemplo de `setup.py`:

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
### Compilando e executando o pacote

Após criar o nó Python (`my_first_node.py`) e configurar o `setup.py`, precisamos **compilar** o workspace.

1. Vá até a raiz do workspace (onde está a pasta `src/`):
   
```bash
cd ~/ros2_ws
```

2. Compile usando o colcon:
   
```bash
colcon build --packages-select my_py_pkg
```

3. Carregue as configurações do workspace compilado:
   
```bash
source install/setup.bash
```

4. Agora já é possível rodar o nó:

```bash
ros2 run my_py_pkg py_node
```

Se tudo estiver correto, você verá no terminal:
```less
[INFO] [1681234567.123456789] [py_test]: Hello World !
[INFO] [1681234568.123456789] [py_test]: Hello 0
[INFO] [1681234569.123456789] [py_test]: Hello 1
...
```
