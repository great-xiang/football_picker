# -*- coding: <encoding name> -*- : # -*- coding: utf-8 -*-
import mmap, time, cv2, os, parl, paddle, random, math
from PIL import Image
from io import BytesIO
import numpy as np
from yolo import YOLO
import paddle.nn as nn
import paddle.nn.functional as F
from paddle.distribution import Normal
from visdom import Visdom

# 强化学习策略网络输入维度，输出维度
obs_dim = 18
act_dim = 2
# 强化学习策略网络学习率,默认1e-3
LEARNING_RATE = 1e-3


class Model(parl.Model):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        # 中间层为输出层的10倍
        hid1_size = act_dim * 4
        # 策略网络定义
        self.fc1 = nn.Linear(obs_dim, hid1_size)
        self.fc2 = nn.Linear(hid1_size, act_dim)

    # 策略网络前向传播
    def forward(self, obs):
        out = F.tanh(self.fc1(obs))
        out = F.tanh(self.fc2(out))  # 参考国外示例代码
        return out


class PolicyGradient(parl.Algorithm):
    def __init__(self, model, lr=None):
        self.model = model
        assert isinstance(lr, float)
        self.optimizer = paddle.optimizer.Adam(learning_rate=lr, parameters=model.parameters())

    def predict(self, obs):
        return self.model(obs)

    def learn(self, obs, act_log_prob, reward):
        loss = 0
        for t in range(len(act_log_prob)):
            loss += -act_log_prob[t] * reward[t]
        self.optimizer.clear_grad()
        loss.backward()
        self.optimizer.step()

        return loss


class Agent(parl.Agent):
    def __init__(self, algorithm, obs_dim, act_dim):
        super().__init__(algorithm)
        self.obs_dim = obs_dim
        self.act_dim = act_dim

    def sample(self, obs):
        obs = paddle.to_tensor(obs, dtype='float32')
        mu = self.alg.predict(obs)
        distribution = Normal(mu, paddle.to_tensor([0.05]))
        act = distribution.sample([1])
        log_prob = distribution.log_prob(act[0])
        return act[0], log_prob

    def predict(self, obs):
        obs = paddle.to_tensor(obs, dtype='float32')
        act = self.alg.predict(obs)
        return act

    def learn(self, obs, act, reward):
        obs = paddle.to_tensor(obs, dtype='float32')
        act = paddle.to_tensor(act, dtype='float32')
        reward = paddle.to_tensor(reward, dtype='float32')
        self.alg.learn(obs, act, reward)
        return 0


# 根据parl框架构建agent
model = Model(obs_dim, act_dim)
alg = PolicyGradient(model, lr=LEARNING_RATE)
agent = Agent(alg, obs_dim=obs_dim, act_dim=act_dim)
# 导入策略网络参数
if os.path.exists('./model.ckpt'):
    agent.restore('./model.ckpt')
# 创建窗口并初始化
#viz = Visdom()
#viz.line([[0., 0.]], [0], win='reward', opts=dict(title='reward和角度差', legend=['reward', '角度差']))
avr=0
for i in range(100000):
    obs = np.zeros(18, dtype='float32')
    obs_list, action_list, reward_list, act_log_prob_list = [], [], [], []
    reward=0
    temp_reward=0
    for t in range(8):
        bottom = random.uniform(0, 112)/224  # 纵坐标
        abscissa = (random.uniform(0, 224)-112)/224  # 横坐标
        obs[0:3] = [0, bottom, abscissa]
        action, act_log_prob = agent.sample(obs)
        Angle = math.degrees(math.atan(bottom / abscissa))
        reward = -abs(Angle - float(action[0]*90))
        obs_list.append(obs)
        act_log_prob_list.append(act_log_prob)
        reward_list.append(reward)
        temp_reward+= reward
        #angle, speed = ("%8.2f" % action[0]), ("%8.2f" % action[1])
    avr+=temp_reward/8
    agent.learn(obs_list, act_log_prob_list, reward_list)
    if (i % 100 == 0):
        frequency = int(i / 100)
        avr =avr/100
        print("过了{}次,平均rewad:{}".format(frequency,avr))
        avr = 0
        #print("angle:", angle, "speed:", speed, "Angle:", int(Angle), "角度差：", int(Angle - float(action[0] * 90)),"reward:", int(reward))
        #viz.line([[int(reward), int(Angle - float(action[0]) * 90)]], [frequency], win='reward',update='append')
        agent.save('./model.ckpt')

print("大吉大利")
