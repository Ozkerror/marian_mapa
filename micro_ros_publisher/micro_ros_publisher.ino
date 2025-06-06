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
rcl_publisher_t publisher; // Uchwyt do publishera (umożliwia publikowanie wiadomości)
std_msgs__msg__Int32 msg; // Zmienna przechowująca wiadomość typu Int32, która będzie publikowana
rclc_executor_t executor; // Egzekutor, który zarządza wywoływaniem funkcji zwrotnych (callbacków)
rclc_support_t support; // Struktura wsparcia dla inicjalizacji i zarządzania Micro-ROS
rcl_allocator_t allocator; // Alokator pamięci używany przez ROS 2
rcl_node_t node; // Uchwyt do węzła ROS 2
rcl_timer_t timer; // Uchwyt do timera, który pozwala na cykliczne wykonywanie zadań

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

// Funkcja zwrotna (callback) timera: wywoływana cyklicznie przez timer
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time); // Makro informujące kompilator, że zmienna last_call_time jest celowo nieużywana (zapobiega ostrzeżeniom)
  if (timer != NULL) { // Sprawdzenie, czy wskaźnik na timer jest poprawny
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL)); // Publikuje wiadomość przechowywaną w 'msg' za pomocą publishera 'publisher'. Błędy są ignorowane.
    msg.data++; // Inkrementuje (zwiększa o 1) wartość pola 'data' w wiadomości 'msg'
  }
}

// Funkcja setup() Arduino: uruchamiana raz przy starcie mikrokontrolera
void setup() {
  // Konfiguracja transportu Micro-ROS (np. przez port szeregowy)
  set_microros_transports(); 
  
  pinMode(LED_PIN, OUTPUT); // Ustawia pin LED_PIN jako wyjście cyfrowe
  digitalWrite(LED_PIN, HIGH); // Włącza diodę LED (sygnalizacja rozpoczęcia działania)
  
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

  // Tworzenie publishera
  // Pierwszy argument to wskaźnik do struktury publisher.
  // Drugi argument to wskaźnik do węzła, do którego publisher będzie należał.
  // Trzeci argument to pobranie informacji o typie wiadomości (std_msgs/msg/Int32).
  // Czwarty argument to nazwa tematu (topic), na którym będą publikowane wiadomości.
  RCCHECK(rclc_publisher_init_default(
    &publisher, // Wskaźnik do publishera
    &node, // Wskaźnik do węzła
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), // Typ wiadomości
    "micro_ros_arduino_node_publisher")); // Nazwa tematu

  // Tworzenie timera
  const unsigned int timer_timeout = 1000; // Okres timera w milisekundach (1000ms = 1s)
  // Pierwszy argument to wskaźnik do struktury timer.
  // Drugi argument to wskaźnik do struktury support.
  // Trzeci argument to okres timera w nanosekundach (konwersja z milisekund).
  // Czwarty argument to funkcja zwrotna (callback), która będzie wywoływana po upływie każdego okresu timera.
  RCCHECK(rclc_timer_init_default(
    &timer, // Wskaźnik do timera
    &support, // Wskaźnik do struktury support
    RCL_MS_TO_NS(timer_timeout), // Okres timera w nanosekundach
    timer_callback)); // Funkcja zwrotna timera

  // Tworzenie egzekutora
  // Pierwszy argument to wskaźnik do struktury executor.
  // Drugi argument to wskaźnik do kontekstu ze struktury support.
  // Trzeci argument to liczba uchwytów (handles), jakie egzekutor będzie obsługiwał (tutaj 1, dla naszego timera).
  // Czwarty argument to alokator pamięci.
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  // Dodanie timera do egzekutora, aby egzekutor mógł zarządzać jego wywołaniami
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0; // Inicjalizacja wartości pola 'data' w wiadomości 'msg' na 0
}

// Główna pętla loop() Arduino: uruchamiana wielokrotnie po zakończeniu funkcji setup()
void loop() {
  delay(100); // Czeka 100 milisekund (zmniejsza obciążenie procesora, pozwala na inne operacje)
  // Wywołuje egzekutor, aby sprawdził i obsłużył oczekujące zdarzenia (np. wywołanie callbacku timera)
  // RCL_MS_TO_NS(100) to maksymalny czas, przez który egzekutor będzie blokował, czekając na zdarzenia.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}