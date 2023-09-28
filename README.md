# Программное обеспечение гусенечного робота (Crawler Bot Software)
Пакет экосистемы Robot Operating System 2 (ROS2) для гусенечной трекер платформы. 
## Введение
ПО предназначено для управления с верхнего программного уровня. В качестве платформы для развертывания на роботе
установлен одноплатный микрокомпьютер NVIDIA Jetson Nano с операционной системой Ubuntu 22.04.03 LTS (Jammy Jellyfish). 
Весь функционал реализован в популярной среде функционирования роботов ROS2 Humble. Реализовано 3 режима работы 
платформы:
* Отладочный режим (Debug mode) - режим, в котором управление роботом осуществляется через терминал Ubuntu путем 
обращения к параметрам ROS2 и изменению их. Ниже будут перечислены все доступные параметры взаимодействия с роботом. 
* Режим ручного управления (Manual mode) - режим, в котором управление роботом осуществляется посредством контроллера. 
В данный момент реализована поддержка контроллера Playstation 4 (Dualshock 4, DS4). Схема управления при помощи геймпада 
приведена ниже.
* Автоматический режим (Auto mode) - беспилотный режим управления, в котором робот выполняет задачу трекинга объекта 
на изображении с его камеры и следует за этим объектом, держась на определенном расстоянии. В данный момент реализовано 
распознавание человека и следование за ним, используя предобученную нейронную сеть. (В будущем планируется 
распознавание и следование за логотипом РЦР ДГТУ).
> Пакет лишь задаёт параметры управления роботом, напрямую с исполнительными органами (моторами, актуаторами, 
> сервоприводами и т.д.) не взаимодействует. При этом он также не обеспечивает связь верхнего уровня (компьютер) и 
> нижнего (микроконтроллер). Подразумевается лишь передача топиков, которые ПО передает на нижний уровень через microROS
> (запускается отдельно). Получение изображения с камеры внутри пакета также требует заранее отправленного изображения
> в соответсвующий топик экосистемы.
## Установка
Предварительно необходимо создать локальную среду ROS2 на компьютере, если у вас ее еще нет:  
*
   ``` 
   mkdir -p ~/dev_ws/src
  ```
Далее:
1. Перейти в папку локальной среды ROS2
    ```
    cd ~/dev_ws/src
   ```
2. Клонировать репозиторий
    ```
    git clone https://github.com/RCR-DSTU/crawler-bot-software.git
   ```
3. Вернуться в рабочую среду ROS2
    ```
    cd ~/dev_ws
   ```
4. Забилдить проект
    ``` 
    colcon build
   ```
5. Засорсить глобальную среду ROS2 в текущую оболочку
    ```
    source /opt/ros/humble/setup.bash
   ```
6. Засорсить локальную среду ROS2 в текущую оболочку
    ```
    . install/setup.bash
   ```
## Запуск
Запуск программы осуществляется запуском ноды с названием `crawler_bot`:
* 
   ```
   ros2 run crawler_bot crawler_bot_run
  ```
Запуск означает, что программа начнет попытки подключения к контроллеру DS4, топикам, содержащим изображения с камеры 
цвета и глубины, создаст топики для отправки в среду microROS. Изначально следует запустить драйвер камеры, чтобы была 
возможность в любой момент включить изображение или автоматический режим.
## API
### Получаемые топики
| Название                     | Тип сообщения     | Описание                                                                    |
|------------------------------|-------------------|-----------------------------------------------------------------------------|
| /camera/color/image_raw      | sensor_msgs/Image | Цветное изображение с камеры для детектирования объекта следования          |
| /camera/depth/image_rect_raw | sensor_msgs/Image | Изображение глубины с камеры для измерения расстояния до объекта следования |
### Отправляемые топики
| Название           | Тип сообщения       | Описание                                    |
|--------------------|---------------------|---------------------------------------------|
| /crawler_bot/twist | geometry_msgs/Twist | Линейные и угловые скорости движения робота |
### Основные параметры
Параметры, которые можно изменять в любой момент работы робота, по большей части управляются через терминал. 

| Название       | Тип переменной | Описание                                                          |
|----------------|----------------|-------------------------------------------------------------------|
| operating_mode | integer        | 3 режима соответствуют значениям: DEBUG - 0, MANUAL - 1, AUTO - 2 |
| use_camera     | bool           | Активация работы камеры                                           |  

Пример изменения параметра в терминале: 
*   ```
    ros2 param set crawler_bot operating_mode 1
    ```
### Описание режимов
* Отладочный (DEBUG). Работа в этом режиме позволяет изменять скорость движения робота путем задания параметров:
    - `crawler_bot/linear_velocity` - движение робота вдоль линейной оси x, диапазон значений -1.0 ... 1.0 
    - `crawler_bot/angular_velocity` - движение робота вокруг оси z, диапазон значений -1.0 ... 1.0
* Ручной (MANUAL). Работа в этом режиме позволяет изменять скорость движения робота путем отклонения правого 
стика геймпада, причем отклонения влево-вправо изменяют улговую скорость, вверх-вниз - линейную. Так же в этом режиме 
доступно переключение на любой другой режим путем нажатия стрелок вверз (AUTO), вниз (DEBUG). Кнопка Share позволяет 
включать и выключать камеру. На рисунке ниже 
приведена схема управления геймпадом.  
![Схема управления Dualshock 4](https://ae01.alicdn.com/kf/HTB18ArNXcfrK1Rjy0Fmq6xhEXXaH.jpg)
* Автоматический (AUTO). Работа в этом режиме позволяет в автономном режиме роботу следовать за объектом, который 
распознается на изображении камеры
