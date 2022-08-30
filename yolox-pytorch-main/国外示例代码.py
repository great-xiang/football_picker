import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions.normal import Normal  # continuous distribution
import numpy as np
import gym
import math
import matplotlib.pyplot as plt
import keyboard


class Agent(nn.Module):
    def __init__(self, lr):
        super(Agent, self).__init__()
        self.fc1 = nn.Linear(3, 64)
        self.fc2 = nn.Linear(64, 32)
        self.fc3 = nn.Linear(32, 1)  # neural network with layers 3,64,32,1

        self.optimizer = optim.Adam(self.parameters(), lr=lr)

    def forward(self, x):
        x = torch.relu(self.fc1(x))  # relu and tanh for output
        x = torch.relu(self.fc2(x))
        x = torch.tanh(self.fc3(x)) * 2
        return x


env = gym.make('Pendulum-v1')
agent = Agent(0.01)  # hyperparameters
SIGMA = 0.2
DISCOUNT = 0.99
total = []

for e in range(1000):
    log_probs, rewards = [], []
    done = False
    state = env.reset()
    while not done:
        mu = agent.forward(torch.from_numpy(state).float())
        distribution = Normal(mu, SIGMA)
        action = distribution.sample().clamp(-2.0, 2.0)
        log_probs.append(distribution.log_prob(action))
        print("log_probs:",log_probs)
        state, reward, done, info = env.step([action.item()])
        # reward = abs(state[1])
        rewards.append(reward)

    total.append(sum(rewards))

    cumulative = 0
    d_rewards = np.zeros(len(rewards))
    for t in reversed(range(len(rewards))):  # get discounted rewards
        cumulative = cumulative * DISCOUNT + rewards[t]
        d_rewards[t] = cumulative
    d_rewards -= np.mean(d_rewards)  # normalize
    d_rewards /= np.std(d_rewards)

    loss = 0
    for t in range(len(rewards)):
        loss += -log_probs[t] * d_rewards[t]  # loss is - log prob * total reward

    agent.optimizer.zero_grad()
    loss.backward()  # update
    agent.optimizer.step()

    if e % 10 == 0:
        print(e, sum(rewards))
        plt.plot(total, color='blue')  # plot
        plt.pause(0.0001)
        if keyboard.is_pressed("space"):  # holding space exits training
            raise Exception("Exited")


def run(i):  # to visualize performance
    for _ in range(i):
        done = False
        state = env.reset()
        while not done:
            env.render()
            distribution = Normal(agent.forward(torch.from_numpy(state).float()), SIGMA)
            action = distribution.sample()
            state, reward, done, info = env.step([action.item()])
        env.close()


#
#
# def normal(self, x, mu, sigma_sq):                                # 计算动作x在policy net定义的高斯分布中的概率值
#     a = ( -1 * (Variable(x)-mu).pow(2) / (2*sigma_sq) ).exp()
#     b = 1 / ( 2 * sigma_sq * self.pi.expand_as(sigma_sq) ).sqrt()      # pi.expand_as(sigma_sq)的意义是将标量π扩展为与sigma_sq同样的维度
#     return a*b
#
# def select_action(self, state):
#     # mu, sigma_sq = self.model(Variable(state).cuda())
#     mu, sigma_sq = self.model(Variable(state))
#     sigma_sq = F.softplus(sigma_sq)
#
#     eps = torch.randn(mu.size())                                  # 产生一个与动作向量维度相同的标准正态分布随机向量
#     # action = (mu + sigma_sq.sqrt()*Variable(eps).cuda()).data
#     action = (mu + sigma_sq.sqrt()*Variable(eps)).data            # 相当于从N(μ,σ²)中采样一个动作
#     prob = self.normal(action, mu, sigma_sq)                      # 计算动作概率
#     entropy = -0.5*( ( sigma_sq + 2 * self.pi.expand_as(sigma_sq) ).log()+1 ) # 高斯分布的信息熵，参考https://blog.csdn.net/raby_gyl/article/details/73477043
#
#     log_prob = prob.log()                                         # 对数概率
#     return action, log_prob, entropy
#
# def update_parameters(self, rewards, log_probs, entropies, gamma):# 更新参数
#     R = torch.zeros(1, 1)
#     loss = 0
#     for i in reversed(range(len(rewards))):
#         R = gamma * R + rewards[i]                                # 倒序计算累计期望
#         # loss = loss - (log_probs[i]*(Variable(R).expand_as(log_probs[i])).cuda()).sum() - (0.0001*entropies[i].cuda()).sum()
#         loss = loss - (log_probs[i]*(Variable(R).expand_as(log_probs[i]))).sum() - (0.0001*entropies[i]).sum()
#     loss = loss / len(rewards)
#
#     self.optimizer.zero_grad()
#     loss.backward()
#     utils.clip_grad_norm(self.model.parameters(), 40)             # 梯度裁剪，梯度的最大L2范数=40
#     self.optimizer.step()
