# marian_mapa
## LiDAR
### Podłączenie
1. Podłącz źródło zasilania ***5V*** prądu stałego, np. zasilacz labolatoryjny(max prąd ustawić na około 800mA)
2. Połącz LiDAR z komputerem poprzez kabel miniUSB  
*Masy w LiDARZE są połączone, więc można podłączyć mase z zasilacza do wyprowadzonego kabelka z masą od kabla miniUSB*
### Obsługa LiDARa za pomocą ROS2 i gotowej paczki
1. Znajdz port do którego podłączony jest LiDAR, w tym celu
ls /dev/tty*
sudo dmesg | grep tty
w bashu powinien wyswietlic sie port do którego podłączony jest LiDAR
2. Wklej poniższą linijkę z nazwą portu do którego jest podłączony LiDAR
yq -i '.urg_node.ros__parameters.serial_port = "/dev/ttyACM[tutaj]"' config.yaml
3. odpalanie gałęzi dla lidara
ros2 run urg_node urg_node --ros-args --params-file config.yaml
4. Sprawdzenie czy wszystko dziala
ros2 topic echo /scan
### Wyświetlenie efektów w Rviz2
1. Wpisz w bashu
rviz2
2. Ustaw FixedFrame: laser
3. kliknij add w lewym dolnym rogu, następnie By Topic, następnie LaserScan, OK
### Instalacja ROS2 (DLA WERSJI LINUXA 22.04)
Podążaj zgodnie z instrukcją z filmu:  
https://www.youtube.com/watch?v=-VGJy1QGDlA  
oraz poradnikiem na stronie:  
https://docs.ros.org/en/humble/Installation/Alternatives/Ubuntu-Development-Setup.html  

