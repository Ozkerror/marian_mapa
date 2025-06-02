# marian_mapa

## Do zrobienia
1. Ogarnąć podstawy microRosa  
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
