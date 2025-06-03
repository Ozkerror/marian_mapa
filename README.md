# marian_mapa

## Przydatny chat  
https://github.com/copilot/share/c04e510a-41e4-8c36-8011-5e4520d5412a  

## Micro_ROS  
1. Docker - uruchomienie kontenera
   ```bash
  docker run -it --net=host -v /dev:/dev --privileged ros:humble
   ```
2. Konfiguracja srodowiska
  ```bash
  source /opt/ros/$ROS_DISTRO/setup.bash
  ```
3. Tworzenie Workspace'u i pobieranie narzedzi microRosa
   ```bash
  mkdir microros_ws
  cd microros_ws
  git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
  ```
4. instalacja zaleznosci
  ```bash
  sudo apt update && rosdep update
  rosdep install --from-paths src --ignore-src -y
  ```
5. instalacja pipa
   ```bash
   sudo apt-get install python3-pip
   ```
6. buildowanie narzedzi microRosa
   ```bash
   colcon build
   source install/local_setup.bash
   ```
7. instalacja agenta microRosa
   ```bash
   ros2 run micro_ros_setup create_agent_ws.sh
   ```
8. Buildowanie agenta
   ```bash
   ros2 run micro_ros_setup build_agent.sh
   source install/local_setup.bash
   ```
9. Wlaczenie agenta
    ```bash
    ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
    ```
    

  
## Do zrobienia
1. Ogarnąć podstawy microRosa

LINK BOZY
https://cps.unileoben.ac.at/install-micro-ros-on-esp32/  https://cps.unileoben.ac.at/install-micro-ros-on-esp32/



https://micro.ros.org/blog/2020/08/27/esp32/  
https://micro.ros.org/docs/tutorials/core/first_application_linux/  
https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/  
https://www.youtube.com/watch?v=fo5I9ZYbG5Q&list=PL1YH3iMfizDJge1nDCuEMvCvhBkKinIJ-  
https://www.youtube.com/watch?v=4JVdT523gfw&list=PL1YH3iMfizDLgcrTL1rj4NxXYKnPLLkby  
3. przekazać dane z arduino na odpowiedni topic (jakies odom czy cos w tym stylu)
4. złożyć wszystko do kupy i stworzyć mape
5. stworzyć skrypt w bashu do odpalania całości

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

   ```bash
   yq -i '.urg_node.ros__parameters.serial_port = "/dev/ttyACM[tutaj]"' config.yaml
   ```

3. Odpalanie gałęzi dla lidaru:

   ```bash
   ros2 run urg_node urg_node_driver --ros-args --params-file config.yaml
   ```

4. Sprawdzenie, czy wszystko działa – wpisz w nowym terminalu:

   ```bash
   ros2 topic echo /scan
   ```

### Wyświetlenie efektów w Rviz2

1. Wpisz w terminalu:

   ```bash
   rviz2
   ```

2. Ustaw `Fixed Frame`: `laser`
3. Kliknij `Add` w lewym dolnym rogu, następnie `By Topic`, potem `LaserScan`, a na końcu `OK`

### Instalacja ROS2 (dla wersji Linux 22.04)

Podążaj zgodnie z instrukcją z filmu:  
https://www.youtube.com/watch?v=-VGJy1QGDlA

oraz poradnikiem na stronie:  
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
