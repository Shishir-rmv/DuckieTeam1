import numpy as np
from PIL import Image

im = Image.open('RoadTest.jpg').convert('LA')

width, height = im.size
im = im.crop((width*0.1,height*0.8, width, height))
pix = im.getpixel((700,100))
pixedge = im.getpixel((650, 100))
print(pix)
print(pixedge)

im.show()