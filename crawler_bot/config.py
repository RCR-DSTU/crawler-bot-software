import datetime
import logging


logging_level = logging.INFO
logging_format = '%(asctime)s %(levelname)s: %(module)s->%(funcName)s:%(message)s'
logging_time = str(datetime.datetime.now()).split(':')[:2]

# logging_handlers = [logging.FileHandler(f"./logs/logo_follower_{':'.join(logging_time)}.log",
#                                         mode="w"),
#                     logging.StreamHandler(),
#                     ]

logging_handlers = [
                    logging.StreamHandler(),
                    ]

logging.basicConfig(level=logging.INFO,
                    handlers=logging_handlers,
                    format=logging_format,
                    )


gamepad_interface = "/dev/input/js0"


common_logger = logging.getLogger("LogoFollowerLogger")


test_image = 'media/StreetPhoto.jpg'


use_camera = False


"""
3 Режима работы робота:
    1 - Режим отладки, работает только управление с компьютера через ROS2
    2 - Режим работы ручного управления, когда подключен геймпад
    3 - Автоматический режим работы
"""
OperatingMode = 0

DEBUG = 0
MANUAL = 1
AUTO = 2
