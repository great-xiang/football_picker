import image,sensor,lcd
import KPU as kpu
import gc
import Maix

#总内存 = 固件大小 + 静态变量申请内存 + 其它功能动态申请(比如摄像头 屏幕显示申请的缓冲区, RGB888 缓冲区, Micropython 的GC 内存块) + 模型使用 + 剩余内存, 模型使用的内存在使用nncase转换的时候会有输出, V3 输出了 main memory usage, V4 输出了working memory usage.
threshold=[(100,255)]
ROI = (0,20,32,12)
kernel_size = 1
kernel = [-1, -1, -1,
          -1, +8, -1,
          -1, -1, -1]
lcd.init()
img = image.Image("/sd/test.jpg").resize(32,32).to_grayscale()

img.morph(kernel_size, kernel)
img.binary(threshold)
img.erode(1, threshold = 2)
line = img.get_regression([(100,255)], roi=ROI,robust = True)
rho_err = abs(line.rho())-img.width()/2
img.draw_line(line.line())
print(rho_err,line.magnitude(),rho_err)
lcd.display(img)







#lcd.display(img)
#print(img.find_edges(image.EDGE_CANNY))

#newimg=[[[0]*3]*64]*64
#for x in range(0,64):
    #for y in range(0,64):
        #newimg[x][y]=img.get_pixel(x,y)
#print(newimg)

#bi = img.to_bytes()
#del img
#img = image.Image(size=(224, 224))
#for x in range(224):
    #for y in range(224):
        #i=x*224+y
        #r5 = (bi[i] & 0xF8) >> 3
        #g6 = ((bi[i] & 0x07) << 3) | ((bi[i] & 0xE0)>>5)
        #b5 = bi[i] & 0x1F
        #r8 = (r5 << 3) | (r5 >> 2)
        #g8 = (g6 << 2) | (g6 >> 4)
        #b8 = (b5 << 3) | (b5 >> 2)
        #img.set_pixel(x,y,(r8,g8,b8))

#lcd.display(img)

