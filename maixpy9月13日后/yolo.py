import KPU as kpu

# ————————————————————————————————————————————————————————————————————————————————————————————————————
# 目标检测参数设置
# ——————————————————————————————————————————————————————————————————————————————————————————————————————

labels = ['people', 'goal', 'football']
people = 0
football = 2
goal = 1
anchors = [0.44, 0.47, 5.34, 3.62, 2.28, 1.94, 0.75, 1.06, 0.81, 2.97]
task = None
## 显示堆内存
#print(Maix.utils.heap_free() / 1024)
## 显示栈内存
#print(gc.mem_free() / 1024)
task = kpu.load("/sd/model-11393.kmodel")
kpu.init_yolo2(task, 0.5, 0.3, 5, anchors)
## 显示堆内存
#print(Maix.utils.heap_free() / 1024)
## 显示栈内存
#print(gc.mem_free() / 1024)
obs = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
