from Maix import utils
import machine

print(utils.gc_heap_size())

utils.gc_heap_size(1024*1024) # 1MiB
machine.reset()
