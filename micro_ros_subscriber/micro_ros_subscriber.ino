// Dołączenie biblioteki transportowej Micro-ROS dla Arduino
#include <micro_ros_arduino.h>

// Dołączenie standardowych bibliotek C oraz bibliotek ROS 2 C
#include <stdio.h> // Standardowe wejście/wyjście C
#include <rcl/rcl.h> // Główna biblioteka ROS 2 C Client Library (RCL)
#include <rcl/error_handling.h> // Obsługa błędów w RCL
#include <rclc/rclc.h> // Biblioteka ROS 2 C Client Library C (RCLC) - warstwa upraszczająca RCL
#include <rclc/executor.h> // Obsługa egzekutora w RCLC (zarządzanie callbackami)

// Dołączenie standardowego typu wiadomości Int32
#include <std_msgs/msg/int32.h>

// Deklaracja globalnych zmiennych ROS 2
rcl_subscription_t subscriber; // Uchwyt do subskrybenta (umożliwia odbieranie wiadomości)
std_msgs__msg__Int32 msg; // Zmienna przechowująca otrzymaną wiadomość typu Int32
rclc_executor_t executor; // Egzekutor, który zarządza wywoływaniem funkcji zwrotnych (callbacków)
rclc_support_t support; // Struktura wsparcia dla inicjalizacji i zarządzania Micro-ROS
rcl_allocator_t allocator; // Alokator pamięci używany przez ROS 2
rcl_node_t node; // Uchwyt do węzła ROS 2
rcl_timer_t timer; // Uchwyt do timera (nieużywany w tym przykładzie, ale zadeklarowany)

// Definicja pinu Arduino, do którego podłączona jest dioda LED (zazwyczaj pin 13)
#define LED_PIN 13

// Makro do sprawdzania błędów funkcji ROS 2. Jeśli funkcja zwróci błąd, program przechodzi do pętli error_loop()
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
// Makro do "miękkiego" sprawdzania błędów. Jeśli funkcja zwróci błąd, jest on ignorowany (używane dla mniej krytycznych operacji)
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


// Funkcja pętli błędu: w przypadku krytycznego błędu, dioda LED miga, a program zatrzymuje się w tej pętli
void error_loop(){
  while(1){ // Nieskończona pętla
    digitalWrite(LED_PIN, !digitalRead(LED_PIN)); // Zmienia stan diody LED (włącza, jeśli wyłączona; wyłącza, jeśli włączona)
    delay(100); // Czeka 100 milisekund
  }
}

// Funkcja zwrotna (callback) subskrypcji: wywoływana, gdy nadejdzie nowa wiadomość na subskrybowanym temacie
void subscription_callback(const void * msgin) // Argument msgin to wskaźnik na otrzymaną wiadomość (typu void*, wymaga rzutowania)
{  
  // Rzutowanie wskaźnika msgin na właściwy typ wiadomości (std_msgs__msg__Int32 *)
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;
  // Sterowanie diodą LED na podstawie wartości otrzymanej wiadomości:
  // Jeśli msg->data (wartość liczbowa w wiadomości) jest równe 0, dioda LED jest wyłączana (LOW).
  // W przeciwnym przypadku (gdy msg->data jest różne od 0), dioda LED jest włączana (HIGH).
  digitalWrite(LED_PIN, (msg->data == 0) ? LOW : HIGH);  
}

// Funkcja setup() Arduino: uruchamiana raz przy starcie mikrokontrolera
void setup() {
  // Konfiguracja transportu Micro-ROS (np. przez port szeregowy)
  set_microros_transports(); 
  
  pinMode(LED_PIN, OUTPUT); // Ustawia pin LED_PIN jako wyjście cyfrowe
  digitalWrite(LED_PIN, HIGH); // Włącza diodę LED (sygnalizacja rozpoczęcia działania, może być później zmieniona przez callback)
  
  delay(2000); // Czeka 2 sekundy (daje czas na ustabilizowanie się systemu lub połączenia)

  allocator = rcl_get_default_allocator(); // Pobiera domyślny alokator pamięci dla ROS 2

  // Inicjalizacja struktury wsparcia Micro-ROS
  // Pierwszy argument to wskaźnik do struktury support.
  // Drugi (0) i trzeci (NULL) argument to liczba argumentów i same argumenty przekazywane do rcl_init (tutaj nieużywane).
  // Czwarty argument to alokator pamięci.
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // Tworzenie węzła ROS 2
  // Pierwszy argument to wskaźnik do struktury node.
  // Drugi argument to nazwa węzła ("micro_ros_arduino_node").
  // Trzeci argument to przestrzeń nazw (namespace) węzła (tutaj pusta).
  // Czwarty argument to wskaźnik do zainicjalizowanej struktury support.
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // Tworzenie subskrybenta
  // Pierwszy argument to wskaźnik do struktury subscriber.
  // Drugi argument to wskaźnik do węzła, do którego subskrybent będzie należał.
  // Trzeci argument to pobranie informacji o typie wiadomości (std_msgs/msg/Int32), którą subskrybent będzie odbierał.
  // Czwarty argument to nazwa tematu (topic), na którym subskrybent będzie nasłuchiwał wiadomości.
  RCCHECK(rclc_subscription_init_default(
    &subscriber, // Wskaźnik do subskrybenta
    &node, // Wskaźnik do węzła
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), // Typ oczekiwanej wiadomości
    "micro_ros_arduino_subscriber")); // Nazwa tematu do subskrypcji

  // Tworzenie egzekutora
  // Pierwszy argument to wskaźnik do struktury executor.
  // Drugi argument to wskaźnik do kontekstu ze struktury support.
  // Trzeci argument to liczba uchwytów (handles), jakie egzekutor będzie obsługiwał (tutaj 1, dla naszej subskrypcji).
  // Czwarty argument to alokator pamięci.
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Dodanie subskrypcji do egzekutora
  // Pierwszy argument to wskaźnik do egzekutora.
  // Drugi argument to wskaźnik do subskrybenta.
  // Trzeci argument to wskaźnik do zmiennej, gdzie będzie przechowywana otrzymana wiadomość (używane wewnętrznie przez RCLC).
  // Czwarty argument to funkcja zwrotna (callback), która zostanie wywołana po otrzymaniu nowej wiadomości.
  // Piąty argument (ON_NEW_DATA) określa, kiedy callback ma być wywołany (w tym przypadku, gdy pojawią się nowe dane).
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA));
}

// Główna pętla loop() Arduino: uruchamiana wielokrotnie po zakończeniu funkcji setup()
void loop() {
  delay(100); // Czeka 100 milisekund (zmniejsza obciążenie procesora, pozwala na inne operacje)
  // Wywołuje egzekutor, aby sprawdził i obsłużył oczekujące zdarzenia (np. wywołanie callbacku subskrypcji po otrzymaniu wiadomości)
  // RCL_MS_TO_NS(100) to maksymalny czas, przez który egzekutor będzie blokował, czekając na zdarzenia.
  // Użycie RCCHECK zamiast RCSOFTCHECK oznacza, że błąd w spin_some jest traktowany jako krytyczny.
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}