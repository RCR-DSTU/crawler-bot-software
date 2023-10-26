from ament_index_python import get_package_prefix


imageWidth = int(1280 / 4)
imageHeight = int(720 / 4)


# Ниже определены 3 режима работы робота:
#     0 - (DEBUG) Режим отладки, работает только управление с компьютера через ROS2
#     1 - (MANUAL) Режим работы ручного управления, когда подключен геймпад
#     2 - (AUTO) Автоматический режим работы
DEBUG = 0
MANUAL = 1
AUTO = 2
# Период таймера в секундах всех РОС2 таймеров (20 раз в секунду)
TIMER_PERIOD = 0.02  # sec
# Таймаут для выключения автономного режима, если не поступает изображения с камеры в миллисекундах
AUTO_MODE_TIMEOUT = 10000  # msec


#  Максимальные и минимальные значения скоростей для управления роботом
max_linear_velocity = 1.0
max_angular_velocity = 1.0
min_linear_velocity = -1.0
min_angular_velocity = -1.0
#  Среднее значение ускорения робота
averageAcceleration = 0.01
# Коэффициенты регуляторов в автономном режиме
pLinearRatio = 1.5
pAngularRatio = 1.5
# Класс объектов, которые необходимо отслеживать, после распознавания нейронной сети
detectingClass = 0  # Integer
# Название логотипа распознаваемого нейронной сетью
detectingLogoName = 'RCR Logo'  # String
# Тип YOLOV8 модели нейронной сети
nnModel = 'yolov8n.yaml'
# Путь к весам нейронной сети
nnPath = f"{get_package_prefix('crawler_bot')}/share/crawler_bot/yolov8/model.pt"
# nnPath = f"{get_package_prefix('crawler_bot')}/lib/python3.10/site-packages/crawler_bot/yolov8/model.pt"
# Интерфейс подключения геймпада
gamepadInterface = "/dev/input/js0"
# Картинка для тестирования модуля распознавания
testImagePath = 'media/StreetPhoto.jpg'
# Переменная, хранящая текущий режим работы из параметров
operatingMode = AUTO  # Integer
# Флаг испльзования камеры
usingCamera = True  # Boolean
# Пустая переменная для экземпляра ноды, чтобы можно было использовать ноду из всех скриптов, к которым подлючен
# этот файл
mainNode = None  # None

