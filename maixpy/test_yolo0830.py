import image,sensor
import KPU as kpu
import gc
import Maix

#总内存 = 固件大小 + 静态变量申请内存 + 其它功能动态申请(比如摄像头 屏幕显示申请的缓冲区, RGB888 缓冲区, Micropython 的GC 内存块) + 模型使用 + 剩余内存, 模型使用的内存在使用nncase转换的时候会有输出, V3 输出了 main memory usage, V4 输出了working memory usage.


labels = ['people', 'football', 'goal']
people = 0
football = 1
goal = 2
anchors = [4.92, 3.31, 0.69, 2.53, 1.12, 0.97, 0.47, 0.53, 1.09, 3.36]
task = None
task = kpu.load("/sd/model-11247.kmodel")
kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)
obs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]



img = image.Image("/sd/4777052.jpg")
bi = img.to_bytes()
del img
img = image.Image(size=(224, 224))
for x in range(224):
    for y in range(224):
        i=x*224+y
        r5 = (bi[i] & 0xF8) >> 3
        g6 = ((bi[i] & 0x07) << 3) | ((bi[i] & 0xE0)>>5)
        b5 = bi[i] & 0x1F
        r8 = (r5 << 3) | (r5 >> 2)
        g8 = (g6 << 2) | (g6 >> 4)
        b8 = (b5 << 3) | (b5 >> 2)
        img.set_pixel(x,y,(r8,g8,b8))

print(img)
#objects = kpu.run_yolo2(task,img)
#obs = []
#if objects:
    #for i in objects:
        #obs.append([i.classid(), i.value(), i.y() - i.h(), i.x() - 112])
#print(obs)
