from glob import glob                                                           
import cv2 

# Drag this program to where you want to convert your files and then just run it on the terminal!
def convert():
    pngs = glob('./*.png')
    for i in pngs:
        img = cv2.imread(i)
        cv2.imwrite(i[:-3] + 'jpg', img)

if __name__ == "__main__":
    convert()
