#include <functional>
#include <stdexcept>
/*Paquete usado para generar varios hilos de ejecucion*/
#include <thread>

/*Paquetes usados para la iniclaizacion de nodo con ros*/
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
/*Paquete para los mensajes de geometria tipo TWIST*/
#include <geometry_msgs/msg/twist.hpp>


#include <signal.h>
/*Paquete usado para imprimir salidas de datos*/
#include <stdio.h>
#ifdef _WIN32
# include <windows.h>
#else
# include <termios.h>
# include <unistd.h>
#endif
/*Paquete que incluye el servicio de guardar el path*/
/*Paquete para el mensaje de tipo float que almacena los datos de los movimientos ejecutados*/

/* Variables que asignan las teclas a usar para movimiento del robot*/
static constexpr char KEYCODE_RIGHT = 0x43;
static constexpr char KEYCODE_LEFT = 0x44;
static constexpr char KEYCODE_UP = 0x41;
static constexpr char KEYCODE_DOWN = 0x42;
static constexpr char KEYCODE_Q = 0x71;

bool running = true;
/*Mensaje de tipo float usado para el txt que se guardara en el path*/


class KeyboardReader final
{
public:
  KeyboardReader()
  {
#ifdef _WIN32
    hstdin_ = GetStdHandle(STD_INPUT_HANDLE);
    if (hstdin_ == INVALID_HANDLE_VALUE)
    {
      throw std::runtime_error("Failed to get stdin handle");
    }
    if (!GetConsoleMode(hstdin_, &old_mode_))
    {
      throw std::runtime_error("Failed to get old console mode");
    }
    DWORD new_mode = ENABLE_PROCESSED_INPUT;  // for Ctrl-C processing
    if (!SetConsoleMode(hstdin_, new_mode))
    {
      throw std::runtime_error("Failed to set new console mode");
    }
#else
    // get the console in raw mode
    if (tcgetattr(0, &cooked_) < 0)
    {
      throw std::runtime_error("Failed to get old console mode");
    }
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &=~ (ICANON | ECHO);
    // Setting a new line, then end of file
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    raw.c_cc[VTIME] = 1;
    raw.c_cc[VMIN] = 0;
    if (tcsetattr(0, TCSANOW, &raw) < 0)
    {
      throw std::runtime_error("Failed to set new console mode");
    }
#endif
  }

  char readOne()
  {
    /*VAriable que guarda el evento de la tecla seleccionada*/
    char c = 0;

#ifdef _WIN32
    INPUT_RECORD record;
    DWORD num_read;
    switch (WaitForSingleObject(hstdin_, 100))
    {
        /*Revisa el caso para el cual se queda esperando respuesta y no se genera*/
    case WAIT_OBJECT_0:
      if (!ReadConsoleInput(hstdin_, &record, 1, &num_read))
      {
        throw std::runtime_error("Read failed");
      }
      /*Revisa que evento ha sido seleccionado de tal forma que se guarda en el char la tecla seleccionada*/
      if (record.EventType != KEY_EVENT || !record.Event.KeyEvent.bKeyDown) {
        break;
      }

      if (record.Event.KeyEvent.wVirtualKeyCode == VK_LEFT)
      {
        c = KEYCODE_LEFT;
      }
      else if (record.Event.KeyEvent.wVirtualKeyCode == VK_UP)
      {
        c = KEYCODE_UP;
      }
      else if (record.Event.KeyEvent.wVirtualKeyCode == VK_RIGHT)
      {
        c = KEYCODE_RIGHT;
      }
      else if (record.Event.KeyEvent.wVirtualKeyCode == VK_DOWN)
      {
        c = KEYCODE_DOWN;
      }
      else if (record.Event.KeyEvent.wVirtualKeyCode == 0x51)
      {
        c = KEYCODE_Q;
      }
      break;

    case WAIT_TIMEOUT:
      break;
    }
/*Protocolo de revision en caso de que falle el evento de las teclas*/
#else
    int rc = read(0, &c, 1);
    if (rc < 0)
    {
      throw std::runtime_error("read failed");
    }
#endif

    return c;
  }

  ~KeyboardReader()
  {
#ifdef _WIN32
    SetConsoleMode(hstdin_, old_mode_);
#else
    tcsetattr(0, TCSANOW, &cooked_);
#endif
  }

private:
#ifdef _WIN32
  HANDLE hstdin_;
  DWORD old_mode_;
#else
  struct termios cooked_;
#endif
};
/*CReaci'on de la clase turtleteleop con funciones propias para lectura, asignacion de variables*/
class TurtleBotTeleop final
{
public:
/*Declara el nodo del movimiento para ROS seg'un el parametro entregado 'rclcpp::Node::SharedPtr nh'*/
  TurtleBotTeleop(rclcpp::Node::SharedPtr nh)
  {
    rclcpp::Node::SharedPtr nh_ = nh;
    /*Se crea un mensaje de tipo twist que contiene info del vector lineal y angular para ser publicado en el topico turtlebot_cmdVel" */
    twist_pub_ = nh_->create_publisher<geometry_msgs::msg::Twist>("cmdVel", 1);
  }

  int keyLoop(float p_linear, float p_angular)
  {
    char c;
    /*MUESTRA mensaje en la consola para que el usuario sepa que se et'a ejecutando el loop de lectura de datos de las teclas*/

    puts("---------------------------");
    puts("\nReading from keyboard");
    puts("Use arrow keys to move the turtle.");
    puts("'Q' to quit.");

    while (running)
    {
      c = input_.readOne();
  
      double linear = 0.0;
      double angular = 0.0;

      switch(c)
      {
        /*Se verifica el evento seleccionado de tal forma que las variable angular y linear cambian de valores con las velocidades asignadas*/
      case KEYCODE_LEFT:
        angular = p_angular;
        /*Despues de asignar el cambio de valor,se introduce en el vector res un valor entre 1 a 4 identificado como el movimiento ejecutado*/
        /*los datos de res posteriormente van a ser procesados para identificar los movimientos hechos por el jugador*/
        break;
      case KEYCODE_RIGHT:
        angular = -p_angular;
        break;
      case KEYCODE_UP:
        linear = p_linear;
        break;
      case KEYCODE_DOWN:
        linear = -p_linear;
        break;
      case KEYCODE_Q:
        running = false;
        break;
      default:
        break;
      }
      /*E caso de que se este ejecutando los movimientos hechos por el jugador se hace lo siguiente:*/
      if (running && (linear != 0.0 || angular != 0.0))
      {
        /*Genera el mensaje twist*/
        geometry_msgs::msg::Twist twist;
        /*Se modifica los valores del vector angular en z y lineal en x*/
        twist.angular.z = angular;
        twist.linear.x = linear;
        /*Se publica el mensaje con los nuevos valores en el vector*/
        twist_pub_->publish(twist);
        /*Se vuelve a inicializar en cero el mensaje para que el robot no siga ejecutando la misma accion*/
        twist.angular.z = 0;
        twist.linear.x = 0;
        twist_pub_->publish(twist);
      }
    }

    return 0;
  }

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
  KeyboardReader input_;
};

#ifdef _WIN32
BOOL WINAPI quit(DWORD ctrl_type)
{
  (void)ctrl_type;
  running = false;
  return true;
}
#else
/*En caso de que el jugador no desee jugar mas o no quiera seleccionar mas teclas, se sale del bucle keyloop*/
void quit(int sig)
{
  (void)sig;
  running = false;
}
#endif
/*Los movimientos guardados en res son guardados en el path que el nodo accede*/

int main(int argc, char **argv)
{
  
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("turtle_bot_teleop");
  /*Se crean variables que guardaran las velocidades lineal y angular que el usuario coloque en la consola*/
  float linear;
  float angular;

  printf("Input the linear velocity: ");
  scanf("%f", &linear);
    
  printf("Input the angular velocity: ");
  scanf("%f", &angular);
  /*Inicializa el nodo*/
  TurtleBotTeleop teleop_turtle(node);
  /*en caso de que halla algun valor en el vector por un evento anterior, se vuelve vacio nuevamente*/
  /*Los dos priemros valores en el txt van a ser los datos ingresados por el usuario de la velcidad lineal y angular*/
  /*Se ejecuta el bucle para la lectura de las teclas seleccionadas y posteriormente publicarlo en el topico cmvel*/
  teleop_turtle.keyLoop(linear, angular);
  /*Se crea el servicio de guardar el path*/

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Finished");

  rclcpp::spin(node);
  rclcpp::shutdown();

}