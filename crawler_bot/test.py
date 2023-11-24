import cv2
import numpy as np

from crawler_bot import config, logo


image1 = cv2.imread('./media/IMG_0219.jpg')
image2 = cv2.imread('./media/IMG_0219(1).jpg')
image3 = cv2.imread('./media/StreetPhoto.jpg')

color1 = logo.get_dominant_color(image1)
color2 = logo.get_dominant_color(image2)
color3 = logo.get_dominant_color(image3)

delta1 = sum(abs(color1 - color2))
delta2 = sum(abs(color1 - color3))

print(delta1, delta2)
