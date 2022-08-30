import mmap, time, cv2,threading
from PIL import Image
from io import BytesIO
import numpy as np
def read_csharp():
    random=0
    while(1):
        try:
            shmem = mmap.mmap(0, 100000, "img", mmap.ACCESS_READ)
            Done=str(shmem.read(1))
            if(Done[3]!="x"):
                this_random= str(shmem.read(4))
                if(random!=this_random):
                    random = this_random
                    img_size = int(shmem.read(5).decode('ASCII'))
                    img = Image.open(BytesIO(shmem.read(img_size)))
                    img_line = cv2.cvtColor(np.asarray(img), cv2.COLOR_RGB2BGR)
                shmem.close()
                break
            else:
                shmem.close()
        except Exception as re:
            print(re)
            pass
        time.sleep(0.1)
    return img,img_line, Done


def send_csharp(angle,speed):
    angle =str(angle).encode('ASCII')
    speed = str(speed).encode('ASCII')
    order=angle+speed
    shmem = mmap.mmap(0, 16, 'order', mmap.ACCESS_WRITE)
    shmem.write(order)
    time.sleep(0.4)
    return 0

def dectet_line(img):
    lower_white = np.array([0, 0, 225])
    upper_white = np.array([180, 30, 255])
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_white, upper_white)
    zero = np.zeros_like(mask)
    zero = cv2.fillPoly(zero, np.array([[[1, 154], [93, 121], [139, 121], [219, 174]]]), color=255)
    res = cv2.bitwise_and(mask, zero)
    white = cv2.countNonZero(res)
    m = cv2.moments(res, False)
    size = img.shape
    w = size[0]  # 宽度
    h = size[1]  # 高度
    try:
        Cx, Cy = m['m10'] / m['m00'], m['m01'] / m['m00']
    except ZeroDivisionError:
        Cx, Cy = h / 2, w / 2
    turn_info = (Cx - w / 2.0) / 10
    # 直角转弯
    # if (Cy > 67 * h / 100):
    #     turn_info = turn_info * 16.0 + 5
    #     return ("%8.2f" %(turn_info))

    if(white < 500 and abs(turn_info) < 0.5):
        turn_info = -0.5
    elif(turn_info > 2.0):
        turn_info = 0.5
    elif(turn_info < -3.5):
        turn_info = -3.5
    elif(turn_info > -1 and turn_info < 0.0):
        turn_info = -3.0
    return ("%8.2f" %(turn_info))

for i in range(1000):
    img,img_line, Done = read_csharp()
    angle = dectet_line(img_line)
    print(angle)
    speed=("%8.2f"%1)
    send_csharp(angle,speed)

