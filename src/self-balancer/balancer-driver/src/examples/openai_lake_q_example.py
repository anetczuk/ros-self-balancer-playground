#!/usr/bin/env python3

import gym
import numpy as np
import operator
from IPython.display import clear_output
from time import sleep
import random
import itertools
import tqdm
import pprint


tqdm.monitor_interval = 0


def create_random_policy(env):
    policy = {}
    for key in range(0, env.observation_space.n):
        #current_end = 0
        p = {}
        for action in range(0, env.action_space.n):
            p[action] = 1 / env.action_space.n
        policy[key] = p
    return policy


def create_state_action_dictionary(env, policy):
    Q = {}
    for key in policy.keys():
        Q[key] = {a: 0.0 for a in range(0, env.action_space.n)}
    return Q


def run_game(env, policy, display=True):
    env.reset()
    episode = []
    finished = False

    while not finished:
        s = env.env.s
        if display:
            clear_output(True)
            env.render()
            sleep(0.2)

        timestep = []
        timestep.append(s)
        n = random.uniform(0, sum(policy[s].values()))
        top_range = 0
        for prob in policy[s].items():
            top_range += prob[1]
            if n < top_range:
                action = prob[0]
                break
        state, reward, finished, info = env.step(action)
        timestep.append(action)
        timestep.append(reward)

        episode.append(timestep)

    if display:
        clear_output(True)
        env.render()
        sleep(0.2)
    return episode


def test_policy(policy, env):
    wins = 0
    r = 100
    for i in range(r):
        wresult = run_game(env, policy, display=False)
#         wresult = run_game(env, policy, display=False)
        w = wresult[-1][-1]
        if w == 1:
            wins += 1
    return wins / r


def monte_carlo_e_soft(env, episodes=100, policy=None, epsilon=0.01):
    if not policy:
        policy = create_random_policy(env)  # Create an empty dictionary to store state action values
    Q = create_state_action_dictionary(env, policy)  # Empty dictionary for storing rewards for each state-action pair
    returns = {}  # 3.

    for _ in range(episodes):  # Looping through episodes
        G = 0  # Store cumulative reward in G (initialized at 0)
        episode = run_game(env=env, policy=policy, display=False)  # Store state, action and value respectively

        # for loop through reversed indices of episode array.
        # The logic behind it being reversed is that the eventual reward would be at the end.
        # So we have to go back from the last timestep to the first one propagating result from the future.

        for i in reversed(range(0, len(episode))):
            s_t, a_t, r_t = episode[i]
            state_action = (s_t, a_t)
            G += r_t  # Increment total reward by reward on current timestep

            if state_action not in [(x[0], x[1]) for x in episode[0:i]]:  #
                if returns.get(state_action):
                    returns[state_action].append(G)
                else:
                    returns[state_action] = [G]

                Q[s_t][a_t] = sum(returns[state_action]) / len(returns[state_action])  # Average reward across episodes

                Q_list = list(map(lambda x: x[1], Q[s_t].items()))  # Finding the action with maximum value
                indices = [i for i, x in enumerate(Q_list) if x == max(Q_list)]
                max_Q = random.choice(indices)

                A_star = max_Q  # 14.

                for a in policy[s_t].items():  # Update action probability for s_t in policy
                    if a[0] == A_star:
                        policy[s_t][a[0]] = 1 - epsilon + (epsilon / abs(sum(policy[s_t].values())))
                    else:
                        policy[s_t][a[0]] = (epsilon / abs(sum(policy[s_t].values())))

    return policy


env = gym.make("FrozenLake8x8-v0")
#env.reset()
print("observation space:", env.observation_space.n)
print("action space:", env.action_space.n)

policy = monte_carlo_e_soft(env, episodes=5000)

wins = test_policy(policy, env)

print("win factor:", wins)
print("policy:")
pprint.pprint(policy)
