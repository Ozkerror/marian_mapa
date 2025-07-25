# marian_mapa
# **ZROB LAUNCH FILE**  


ODPALANIE NODEA Z TF:
```bash
ros2 run my_tf_broadcaster odom_to_tf
```
Trzeba wprowadzic zmiane zeby tworzylo sie jeszce polaczenie miedzy base_link i laser. Aktualnie to polaczenie robilem z palca:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 1 base_link laser
```
ZORIENTUJ SIE O CO CHODZI Z USE_SIM_TIME, BYC MOZE BRAK SYNCHRONIZACJI CZASOWEJ MIEDZY POSZCZEGOLNYMI NODEAMI JEST PROBLEMEM!!!!  
STWORZ LAUNCH FILE!!!
  


-------------------------------------------------------------------------------------------
Uzytkownicy zarowno w Dockerze jak i poza nim musza byc tacy sami aby mozliwa byla komunikacja.  
W przyszlosci jesli uda sie stworzyc wlasnego Dockera to warto zainteresowac sie docker compose.  

## Przydatny chat  
https://github.com/copilot/share/c04e510a-41e4-8c36-8011-5e4520d5412a  

## Micro_ROS  
1. Podlacz ESP32 z wgranym programem do komputera
2. Sprawdz do ktorego portu jest podlaczone
   ```bash
   ls /dev/ttyUSB*
   ```
3. odpal dockera z microRosem
   ```bash
   docker run -it --rm --net=host --device=/dev/ttyUSB0 microros/micro-ros-agent:humble serial --dev /dev/ttyUSB0
   ```
   Jesli przy sprawdzaniu portu w pkt. 2 wyskoczyl inny port niz ttyUSB0, nalezy to zmienic w dwoch miejscach w powyzszej komendzie
4. W NOWYM TERMINALU mozna sprawdzic czy wszystko sie ladnie wysyla
   ```bash
   source /opt/ros/humble/setup.bash
   ros2 topic echo /odom
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
2. przekazać dane z arduino na odpowiedni topic (jakies odom czy cos w tym stylu)  
3. złożyć wszystko do kupy i stworzyć mape  
4. stworzyć skrypt w bashu do odpalania całości  

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
   source /opt/ros/humble/setup.bash
   yq -i '.urg_node.ros__parameters.serial_port = "/dev/ttyACM[tutaj]"' config.yaml
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

### Instalacja ROS2 (dla wersji Linux 22.04)

Podążaj zgodnie z instrukcją z filmu:  
https://www.youtube.com/watch?v=-VGJy1QGDlA

oraz poradnikiem na stronie:  
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html
