from ament_index_python import get_package_prefix


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
# nnPath = f"{get_package_prefix('crawler_bot')}/lib/python3.10/site-packages/crawler_bot/yolov8/model.pt"
# Интерфейс подключения геймпада
gamepadInterface = "/dev/input/js0"
# Картинка для тестирования модуля распознавания
testImagePath = 'media/StreetPhoto.jpg'
# Переменная, хранящая текущий режим работы из параметров
operatingMode = AUTO
# Флаг испльзования камеры
usingCamera = True
# Пустая переменная для экземпляра ноды, чтобы можно было использовать ноду из всех скриптов, к которым подлючен
# этот файл
mainNode = None
# Период таймера всех РОС2 таймеров (20 раз в секунду)
TIMER_PERIOD = 0.05



