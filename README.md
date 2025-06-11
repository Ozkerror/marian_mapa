# marian_mapa
Korzystanie z ROS2 do stworzenia mapy przy użyciu LiDARa i ESP32 z microRosa na komputerze bez użycia Docker'a
## Wymagania
--Zainstalowany ROS2 Humble na Ubuntu 22.04

--Stworzony workspace ROS2

--zainstalowany microRos
## Instalacja ROS2 (dla wersji Linux 22.04)

Podążaj zgodnie z instrukcją z filmu:  
https://www.youtube.com/watch?v=-VGJy1QGDlA

oraz poradnikiem na stronie:  
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html

## Przydatny chat  
https://github.com/copilot/share/c04e510a-41e4-8c36-8011-5e4520d5412a  

## Micro_ROS  
1. Wejście do folderu workspace'u
   ```bash
   cd microros_ws/
   ```
2. Konfiguracja środowiska i uruchomienie agenta
   ```bash
   source install/local_setup.bash
   ```
3. Uruchomienie agenta microRosa poprzez WIFI
   ```bash
   ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888 -l 192.168.1.249 -v6
   ```
   A jeśli chcemy poprzez USB<br>*Tutaj sprawdzamy listę podłączonych urządzeń*  
   ```bash
   ls /dev/tty*
   ```
   ```bash
   ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
   ```
4. W nowym terminalu przelacz sprawdzamy czy pojawił się nowy topic
   ```bash
   ros2 topic list
   ```
5. Jeśli chcemy wysłać wiadmość do ESP32, to w nowym terminalu możemy użyć poniższego polecenia:
   ```bash
   ros2 topic pub --once /test_topic std_msgs/msg/Int32 {'data: 1'} 
   ```
   Lub data:0 (1 zapala LED, 0 gasi)
6. Aby sprawdzić, czy ESP32 odbiera wiadomości, możemy użyć:
   ```bash
   ros2 run micro_ros_setup build_agent.sh
   source install/local_setup.bash
   ```
10. Wlaczenie agenta
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
    ```
11. W nowym terminalu przelacz sie na roota
    ```bash
    sudo su
    ```
12. Konfiguracja srodowiska
    ```bash
    source /opt/ros/<twoja_dystrybucja_ros2>/setup.bash
    ```
    
    

  
## Do zrobienia
1. Ogarnąć podstawy microRosa
2. przekazać dane z arduino na odpowiedni topic (jakies odom czy cos w tym stylu)  
3. złożyć wszystko w całość i stworzyć mape  
4. stworzyć skrypt w bashu do odpalania całości  

KTO WIE TEN WIE:
https://www.facebook.com/photo.php?fbid=986893943477119&id=100064695668738&set=a.342504681249385

## LiDAR

### Podłączenie

1. Podłącz źródło zasilania **5V** prądu stałego, np. zasilacz laboratoryjny (max prąd ustawić na około 800 mA).
2. Połącz LiDAR z komputerem poprzez kabel miniUSB  
   _Masy w LiDARZE są połączone, więc można podłączyć masę z zasilacza do wyprowadzonego kabelka z masą od kabla miniUSB_

### Obsługa LiDARa za pomocą ROS2 i gotowej paczki

1. Znajdź port, do którego podłączony jest LiDAR. W tym celu wpisz:

   ```bash
   ls /dev/tty*
   sudo dmesg | grep tty
   ```

   W Bashu powinien wyświetlić się port, do którego podłączony jest LiDAR (np. `/dev/ttyACM0`).

2. Wklej poniższą linijkę z nazwą portu do którego jest podłączony LiDAR:
   Wcześniej należy wejść do folderu z tym plikiem config.yaml
   ```bash
   source /opt/ros/humble/setup.bash
   yq -i '.urg_node.ros__parameters.serial_port = "/dev/ttyACM0"' config.yaml
   ```

3. Odpalanie gałęzi dla lidaru:

   ```bash
   ros2 run urg_node urg_node_driver --ros-args --params-file config.yaml
   ```

4. Sprawdzenie, czy wszystko działa – wpisz w nowym terminal:

   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic echo /scan
   ```

### Wyświetlenie efektów w Rviz2

1. Wpisz w terminalu:

   ```bash
   rviz2
   ```

2. Ustaw `Fixed Frame`: `laser`
3. Kliknij `Add` w lewym dolnym rogu, następnie `By Topic`, potem `LaserScan`, a na końcu `OK`

# Tworzenie mapy
### Przydatne linki
https://wiki.ros.org/hokuyo_node/Tutorials/UsingTheHokuyoNode
https://www.youtube.com/watch?v=tKfVU1n5TjA&t=182s
##