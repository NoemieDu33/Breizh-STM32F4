from PIL import Image
import numpy as np
import pickle

class ST7565Converter:
    def __init__(self, img):
        self._img = Image.open(img)
        self.matrix = np.zeros((64,128),np.int8)
    
    def convert(self):
        w,h = self._img.size
        if w>128 or h>64:
            self._img = self._img.resize((128,64))
        thresh = 50
        w,h = self._img.size
        fn = lambda x : 255 if x > thresh else 0
        self._img = self._img.convert('L').point(fn, mode='1')
        for L in range(h):
            for C in range(w):
                print(self._img.getpixel((C,L)))
                if self._img.getpixel((C,L))>128:
                    self.matrix[L][C] = 1
        with open("output.txt","w") as f:
            msg = "(\n("
            for y in self.matrix:
                for x in y:
                    msg+=f"{x},"
                msg+="),\n("
            msg+=")\n)"
            f.write(msg)
            f.close()
                    

        print(self.matrix)

if __name__=="__main__":
    ST7565Converter("vivian.png").convert()