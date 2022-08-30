import paddle
import paddle.nn as nn
import paddle.nn.functional as F
import parl
import numpy as np
import gym
from parl.utils import logger
from paddle.distribution import Categorical

LEARNING_RATE = 1e-3


class Model(parl.Model):
    def __init__(self, obs_dim, act_dim):
        super().__init__()
        hid1_size = act_dim * 10
        self.fc1 = nn.Linear(obs_dim, hid1_size)
        self.fc2 = nn.Linear(hid1_size, act_dim)

    def forward(self, obs):
        out = F.tanh(self.fc1(obs))
        out = F.softmax(self.fc2(out))
        return out


class PolicyGradient(parl.Algorithm):
    def __init__(self, model, lr=None):
        self.model = model
        assert isinstance(lr, float)
        self.optimizer = paddle.optimizer.Adam(learning_rate=lr, parameters=model.parameters())

    def predict(self, obs):
        return self.model(obs)

    def learn(self, obs, act, reward):
        # act_prob = self.model(obs)
        # log_prob = F.cross_entropy(act_prob, act)
        # loss = log_prob.mean()
        # self.optimizer.clear_grad()
        # loss.backward()
        # self.optimizer.step()
        prob = self.model(obs)
        log_prob = Categorical(prob).log_prob(act)
        print("act:",act)
        print("Categorical(prob):",Categorical(prob))
        print("log_prob:",log_prob)
        loss = paddle.mean(-1 * log_prob * reward)

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
        act_prob = self.alg.predict(obs)
        act_prob = np.squeeze(act_prob, axis=0)

        act = np.random.choice(range(self.act_dim), p=act_prob.numpy())
        return act

    def predict(self, obs):
        obs = paddle.to_tensor(obs, dtype='float32')
        act_prob = self.alg.predict(obs)
        act = np.argmax(act_prob)
        return act

    def learn(self, obs, act, reward):
        act = np.expand_dims(act, axis=-1)
        reward = np.expand_dims(reward, axis=-1)

        obs = paddle.to_tensor(obs, dtype='float32')
        act = paddle.to_tensor(act, dtype='int32')
        reward = paddle.to_tensor(reward, dtype='float32')
        loss = self.alg.learn(obs, act, reward)
        return loss.numpy()[0]


def run_episode(env, agent):
    obs_list, action_list, reward_list = [], [], []
    obs = env.reset()
    while True:
        obs_list.append(obs)
        action = agent.sample(obs)  # 采样动作
        action_list.append(action)

        obs, reward, done, info = env.step(action)
        reward_list.append(reward)

        if done:
            break
    return obs_list, action_list, reward_list


# 评估 agent, 跑 5 个episode，总reward求平均
def evaluate(env, agent, render=False):
    eval_reward = []
    for i in range(5):
        obs = env.reset()
        episode_reward = 0
        while True:
            action = agent.predict(obs)  # 选取最优动作
            obs, reward, isOver, _ = env.step(action)
            episode_reward += reward
            if render:
                env.render()
            if isOver:
                break
        eval_reward.append(episode_reward)
    return np.mean(eval_reward)


# 根据一个episode的每个step的reward列表，计算每一个Step的Gt
def calc_reward_to_go(reward_list, gamma=1.0):
    for i in range(len(reward_list) - 2, -1, -1):
        # G_t = r_t + γ·r_t+1 + ... = r_t + γ·G_t+1
        reward_list[i] += gamma * reward_list[i + 1]  # Gt
    return np.array(reward_list)


# 创建环境
env = gym.make('CartPole-v0')
obs_dim = env.observation_space.shape[0]
act_dim = env.action_space.n
logger.info('obs_dim {}, act_dim {}'.format(obs_dim, act_dim))

# 根据parl框架构建agent
model = Model(obs_dim, act_dim)
alg = PolicyGradient(model, lr=LEARNING_RATE)
agent = Agent(alg, obs_dim=obs_dim, act_dim=act_dim)

# 加载模型
# if os.path.exists('./model.ckpt'):
#     agent.restore('./model.ckpt')
#     run_episode(env, agent, train_or_test='test', render=True)
#     exit()

for i in range(1000):
    obs_list, action_list, reward_list = run_episode(env, agent)
    if i % 10 == 0:
        logger.info("Episode {}, Reward Sum {}.".format(
            i, sum(reward_list)))

    batch_obs = np.array(obs_list)
    batch_action = np.array(action_list)
    batch_reward = calc_reward_to_go(reward_list)

    agent.learn(batch_obs, batch_action, batch_reward)
    if (i + 1) % 100 == 0:
        total_reward = evaluate(env, agent, render=False)  # render=True 查看渲染效果，需要在本地运行，AIStudio无法显示
        logger.info('Test reward: {}'.format(total_reward))

# 保存模型到文件 ./model.ckpt
agent.save('./model.ckpt')