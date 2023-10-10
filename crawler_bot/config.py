import datetime
import logging

from ament_index_python import get_package_prefix

# Параметры логгирования всех скриптов в пакете
loggingLevel = logging.INFO
loggingFormat = '%(asctime)s %(levelname)s: %(module)s->%(funcName)s:%(message)s'
loggingTime = str(datetime.datetime.now()).split(':')[:2]


loggingHandlers = [  # logging.FileHandler(f"./logs/logo_follower_{':'.join(logging_time)}.log", mode="w"),
    logging.StreamHandler(),
]

logging.basicConfig(level=logging.INFO,
                    handlers=loggingHandlers,
                    format=loggingFormat,
                    )


imageWidth = 1280
imageHeight = 720


# Ниже определены 3 режима работы робота:
#     0 - (DEBUG) Режим отладки, работает только управление с компьютера через ROS2
#     1 - (MANUAL) Режим работы ручного управления, когда подключен геймпад
#     2 - (AUTO) Автоматический режим работы
DEBUG = 0
MANUAL = 1
AUTO = 2


# Класс объектов, которые необходимо отслеживать
detectingClass = 0
# Тип YOLOV8 модели нейронной сети
nnModel = 'yolov8n.yaml'
# Путь к весам нейронной сети
nnPath = f"{get_package_prefix('crawler_bot')}/share/crawler_bot/yolov8/model.pt"
# Основной логгер для всех скриптов в пакете
commonLogger = logging.getLogger("LogoFollowerLogger")
# Интерфейс подключения геймпада
gamepadInterface = "/dev/input/js0"
# Картинка для тестирования модуля распознавания
testImagePath = 'media/StreetPhoto.jpg'
# Переменная, хранящая текущий режим работы из параметров
operatingMode = MANUAL
# Флаг испльзования камеры
usingCamera = False
# Пустая переменная для экземпляра ноды, чтобы можно было использовать ноду из всех скриптов, к которым подлючен
# этот файл
mainNode = None
realsenseNode = None


