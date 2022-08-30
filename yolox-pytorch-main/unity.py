# -*- coding: <encoding name> -*- : # -*- coding: utf-8 -*-
import mmap, time, cv2, os, parl, paddle
from PIL import Image
from io import BytesIO
import numpy as np
from yolo import YOLO
import paddle.nn as nn
import paddle.nn.functional as F
from paddle.distribution import Normal

# 初始化yolo目标检测模型
yolo = YOLO()
# 强化学习策略网络输入维度，输出维度
obs_dim = 24
act_dim = 2
# 强化学习策略网络学习率,默认1e-3
LEARNING_RATE = 1e-3


# 读取c#传入数据
# mmap释放不了C#创建的共享内存；mmap读取共享内存会上锁，导致C#读取不了
def read_csharp():
    while (1):
        try:
            shmem2 = mmap.mmap(0, 10, "unitywrite", mmap.ACCESS_READ)
            unitywrite = shmem2.read(1).decode("ASCII")
            shmem2.close()
            # 即使unitywrite不存在，mmap也会读到数据，只不过为空，必须判断是否为数字，才可以跳出循环
            if (unitywrite.isdigit()):
                break
        except Exception as re:
            print(re)
            pass
        time.sleep(0.01)
    # 在读取unitywrite标志后再读取图片数据，是为了保证数据完整性。
    shmem1 = mmap.mmap(0, 100000, "img", mmap.ACCESS_READ)
    Done = int(shmem1.read(1).decode("ASCII"))
    img_size = int(shmem1.read(5).decode('ASCII'))
    img = Image.open(BytesIO(shmem1.read(img_size)))
    shmem1.close()
    return img, Done


# 向C#发送指令
# mmap创建的共享内存一旦调用close方法会释放内存，或者send_csharp结束调用也会释放内存，导致C#读取不了
def send_csharp(angle, speed, Done):
    angle = str(angle).encode('ASCII')
    speed = str(speed).encode('ASCII')
    Done = str(0).encode('ASCII')
    order = angle + speed + Done
    shmem3 = mmap.mmap(0, 20, 'order', mmap.ACCESS_WRITE)
    shmem3.write(order)
    # 创建一个C#读取标志，共享内存必须写入数据才会被创建
    shmem4 = mmap.mmap(0, 1, "pythonwrite", mmap.ACCESS_WRITE)
    shmem4.write(str("1").encode('ASCII'))
    time.sleep(0.1)
    # sleep结束后，共享内存会被释放
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

    if (white < 500 and abs(turn_info) < 0.5):
        turn_info = -0.5
    elif (turn_info > 2.0):
        turn_info = 0.5
    elif (turn_info < -3.5):
        turn_info = -3.5
    elif (turn_info > -1 and turn_info < 0.0):
        turn_info = -3.0
    return ("%8.2f" % (turn_info))


class Model(parl.Model):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        # 中间层为输出层的10倍
        hid1_size = act_dim * 10
        # 策略网络定义
        self.fc1 = nn.Linear(obs_dim, hid1_size)
        self.fc2 = nn.Linear(hid1_size, act_dim)

    # 策略网络前向传播
    def forward(self, obs):
        out = F.tanh(self.fc1(obs))
        out = F.tanh(self.fc2(out)) * 2  # 参考国外示例代码
        return out


class PolicyGradient(parl.Algorithm):
    def __init__(self, model, lr=None):
        self.model = model
        assert isinstance(lr, float)
        self.optimizer = paddle.optimizer.Adam(learning_rate=lr, parameters=model.parameters())

    def predict(self, obs):
        return self.model(obs)

    def learn(self, obs, act_log_prob, reward):
        print("进行梯度更新")
        # loss = paddle.mean(-1 * act_log_prob * reward)
        print("本回合的act_log_prob:", act_log_prob)
        print("本回合的reward:", reward)
        # loss = -1 * act_log_prob * reward
        loss = 0
        for t in range(len(act_log_prob)):
            loss += -act_log_prob[t] * reward[t]
        # Debug避免空回合报错
        try:
            self.optimizer.clear_grad()
            loss.backward()
            self.optimizer.step()
        except:
            pass
        return loss


class Agent(parl.Agent):
    def __init__(self, algorithm, obs_dim, act_dim):
        super().__init__(algorithm)
        self.obs_dim = obs_dim
        self.act_dim = act_dim

    def sample(self, obs):
        obs = paddle.to_tensor(obs, dtype='float32')
        mu = self.alg.predict(obs)
        # 正态分布，mu是神经网络预测结果，标准正太分布标准差为1
        distribution = Normal(mu, paddle.to_tensor([0.1]))
        act = distribution.sample([1])
        log_prob = distribution.log_prob(act[0])
        # 返回act[0]是因为act为[[]]
        return act[0], log_prob

    def predict(self, obs):
        obs = paddle.to_tensor(obs, dtype='float32')
        act = self.alg.predict(obs)
        return act

    def learn(self, obs, act, reward):
        # act = np.expand_dims(act, axis=-1)
        # reward = np.expand_dims(reward, axis=-1)
        obs = paddle.to_tensor(obs, dtype='float32')
        act = paddle.to_tensor(act, dtype='float32')
        reward = paddle.to_tensor(reward, dtype='float32')
        loss = self.alg.learn(obs, act, reward)
        return 0
        # return loss.numpy()[0]


# 根据parl框架构建agent
model = Model(obs_dim, act_dim)
alg = PolicyGradient(model, lr=LEARNING_RATE)
agent = Agent(alg, obs_dim=obs_dim, act_dim=act_dim)


# 目标检测，返回物体识别结果和得分
def yolo_detect(img):
    # 返回结果物体种类，置信度，bottom，(left+right)/2,从左到右，从上到下
    my_result = yolo.detect_image(img, save_img=False)
    # 重新排列返回结果
    obs = np.zeros(24, dtype='float32')
    try:
        obs[0: 4] = my_result[my_result.index(0):my_result.index(0) + 4]
        del my_result[my_result.index(0):my_result.index(0) + 4]
    except:
        pass
    try:
        obs[4: 8] = my_result[my_result.index(2):my_result.index(2) + 4]
        del my_result[my_result.index(2):my_result.index(2) + 4]
    except:
        pass
    for i in range(len(my_result)):
        obs[8 + i] = my_result[0 + i]
    return obs


def get_reward(obs):
    # reward定义，让reward变大,obs格式：物体种类，置信度，bottom，(left+right)/2,从左到右，从上到下
    reward = 0
    if (obs[1] != 0):
        reward = -10 * obs[2] + 10 * abs(obs[3] - 112)
    for i in range(1, 6):
        if (obs[i * 4 + 1] != 0):
            reward += obs[i * 4 + 2] + abs(obs[i * 4 + 3] - 112)
    return reward



# 运行一个回合
def run_episode():
    print("新回合开始")
    obs_list, action_list, reward_list, act_log_prob_list = [], [], [], []
    while (1):
        img, Done = read_csharp()
        obs = yolo_detect(img)
        reward = get_reward(obs)
        python_Done = 0
        # 判断是否结束，Done==1抓到球了，Done==2碰到障碍物
        if (Done != 0):
            print("回合结束")
            if (Done == 1):
                reward += 10000
            if (Done == 2):
                reward -= 10000
            angle, speed = ("%8.2f" % 0), ("%8.2f" % 0)
            send_csharp(angle, speed, python_Done)
            break

        obs_list.append(obs)
        reward_list.append(reward)
        action, act_log_prob = agent.sample(obs)
        action_list.append(action)
        act_log_prob_list.append(act_log_prob)
        angle, speed = ("%8.2f" % action[0]), ("%8.2f" % action[1])
        print("图片的obs:", obs)
        print("reward:", reward, "angle:", angle, "speed:", speed)

        # 训练策略网络
        # if(float(speed)<0):
        #     Done=2
        #     send_csharp(angle, speed, Done)
        #     break
        send_csharp(angle, speed, python_Done)
    return obs_list, action_list, reward_list, act_log_prob_list


# 根据一个episode的每个step的reward列表，计算每一个Step的Gt
def calc_reward_to_go(reward_list, gamma=1.0):
    for i in range(len(reward_list) - 2, -1, -1):
        # G_t = r_t + γ·r_t+1 + ... = r_t + γ·G_t+1
        reward_list[i] += gamma * reward_list[i + 1]  # Gt
    return np.array(reward_list)


for i in range(100):
    # 导入策略网络参数
    if os.path.exists('./model.ckpt'):
        agent.restore('./model.ckpt')
    obs_list, action_list, reward_list, act_log_prob_list = run_episode()
    # 输出训练进度
    # parl.utils.logger.info("Episode {}, Reward Sum {}.".format(i, sum(reward_list)))
    batch_obs = np.array(obs_list)
    batch_action = np.array(action_list)
    batch_reward = calc_reward_to_go(reward_list)
    batch_act_log_prob = np.array(act_log_prob_list)
    # 策略网络更新
    agent.learn(batch_obs, batch_act_log_prob, batch_reward)
    agent.save('./model.ckpt')

print("大吉大利")
