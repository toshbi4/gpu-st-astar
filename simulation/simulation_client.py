#!/usr/bin/env python3

import argparse
from gworld import *
from visualize import *
import operator
import yaml

import socket
import sys
import threading
import time
import tkinter as tk
import os
import re

class Simulation:

    a = 0
    threads = []
    agents_can_move = []
    update_path_vis = []

    actions = []
    paths = []
    nogui = False
    replanning_count = 0

    def __init__(self) -> None:
        pass
    
    def parallel_agent(self, agent_id):
        while True:
            if self.agents_can_move[agent_id]:
                if len(self.actions[agent_id]) > 0:
                    self.a.parallel_step(agent_id+1, self.actions[agent_id].pop())
                self.agents_can_move[agent_id] = False
            time.sleep(0.01)

    def exit_button(self, vis):
        vis.root().quit()
        vis.root().destroy()
        os._exit(1)
        
        print("Button clicked!")

    def start(self, map, schedule, aNogui=False):
        
        self.nogui = aNogui
        self.a = GridWorld(25, 25)

        has_cargo = []
        for o in map["map"]["obstacles"]:
            self.a.add_rocks([(o[0], o[1])])
        for d, i in zip(map["agents"], range(0, len(map["agents"]))):
            self.a.add_agents([(d["start"][0], d["start"][1], d["goal"][0], d["goal"][1])])
            has_cargo.append(False)

        # print(self.a.get_goals()[1][0])

        if not self.nogui:
            vis = Visualize(self.a)
            vis.draw_world()
            vis.draw_agents()
            vis.canvas.pack()
            vis.canvas.update()
            vis.canvas.after(1000)
            button = tk.Button(vis.root(), command=lambda: self.exit_button(vis), text="Exit")
            button.place(relx=0, rely=0, anchor="nw")
        else:
            vis = None

        path = []
        for agent_name in schedule["schedule"]:
            agent = schedule["schedule"][agent_name]
            path = []
            for coord in agent:
                path.append((coord['x'], coord['y']))
            self.paths.append(path)

        # print('Paths before actions: ', paths)

        for num, path in enumerate(self.paths, 1):
            self.actions.append(list(reversed(self.a.path_to_action(num, path))))
            if not self.nogui:
                vis.draw_path(path, num)

        # print('Actions: ', actions)

        for i in range(len(self.paths)):
            self.agents_can_move.append(True)
            t = threading.Thread(target=self.parallel_agent, args=(i,))
            self.threads.append(t)
            t.start()

        finished = False
        time = 0

        stop = False
        while not finished:

            if all(elem == False for elem in self.agents_can_move):
                time += 1
                for agent_id in range(len(self.agents_can_move)):
                    
                    if len(self.actions[agent_id]) == 3:
                        t = threading.Thread(target=self.create_new_task, args=(agent_id, time+4, has_cargo, vis))
                        t.start()

                    if len(self.paths[agent_id]) > 2:
                        path_point = self.paths[agent_id].pop(0)
                        current_coord = self.a.get_pos(agent_id+1)
                        if not current_coord == self.paths[agent_id][0]:
                            print("Agent: ", agent_id)
                            print("Current coord: ", current_coord)
                            print("Path: ", self.paths[agent_id][0])
                            stop = True
                        if not self.nogui:
                            vis.clear_cell(path_point)

                    if not self.nogui:
                        vis.update_agent_vis(agent_id+1)                    
                if not self.nogui:
                    vis.canvas.update()

                if stop:
                    while 1:
                        pass
                
                for i, agent in enumerate(self.update_path_vis):
                    if not self.nogui:
                        vis.draw_path(self.paths[agent], agent+1)
                    self.update_path_vis.pop(i)

                agents, collision  = self.a.check_collision()
                if collision:
                    print("Collision!")
                    print("Agents: ", agents)
                    print("Time: ", time)
                    print("Replanning Count: ", self.replanning_count)

                self.agents_can_move = [True] * len(self.agents_can_move)

        if not self.nogui:
            vis.canvas.after(3000)


    def create_new_task(self, agent_id, time, has_cargo, vis=None):
        
#        print('**************************')
#        print('Paths before new task: ', self.paths)
#        print('---------------')
#        print('Actions before new task: ', self.actions)

        HOST = '127.0.0.1'
        PORT = 12345

        # создаем сокет
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        try:
            # подключаемся к серверу
            sock.connect((HOST, PORT))
        except:
            return 0
        
        x = str(self.a.get_goals()[agent_id+1][0])
        y = str(self.a.get_goals()[agent_id+1][1])
        t = str(40.0 * time)
        # has_cargo[agent_id] = not has_cargo[agent_id]
        cargo = "1" if has_cargo[agent_id] else "0"

        message = x + " " + y + " " + t + " " + cargo
        sock.sendall(message.encode())

        # получаем ответ от сервера
        data = sock.recv(1024)

        # выводим ответ сервера
        print(f'Сервер ответил: {data.decode()}')

        sock.close()

        pattern = r"- x: (?P<x>\d+)\n\s+y: (?P<y>\d+)\n\s+t: (?P<t>\d+)"
        matches = re.finditer(pattern, data.decode())

        new_path = []

        # Вывод координат
        for match in matches:
            x = match.group("x")
            y = match.group("y")
            new_path.append((int(x), int(y)))
            # print(f"x: {x}, y: {y}")

        goal = (new_path[-1][0], new_path[-1][1])

        # print("New path: ", new_path)

        # actions = []
        # for num, path in enumerate(self.paths, 1):
        # path = self.paths[agent_id].copy()
        action = list(reversed(self.a.path_to_action(agent_id+1, new_path)))
        print(action)
        self.paths[agent_id].extend(new_path[1:])
        self.actions[agent_id] = action + self.actions[agent_id]
        # if not self.nogui:
            # vis.draw_path(self.paths[agent_id], agent_id+1)

        self.update_path_vis.append(agent_id)
        self.a.change_goal(agent_id+1, goal)

#        print('**************************')
#        print('Paths after new task: ', self.paths)
#        print('---------------')
#        print('Actions after new task: ', self.actions)
        # return actions

    def get_env(self, grid, agent_id, paths, time, has_cargo, nogui, vis=None):

        HOST = '127.0.0.1'
        PORT = 12345

        # создаем сокет
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

        # подключаемся к серверу
        sock.connect((HOST, PORT))

        x = str(grid.get_goals()[agent_id][0])
        y = str(grid.get_goals()[agent_id][1])
        t = str(40.0 * time)
        has_cargo[agent_id-1] = not has_cargo[agent_id-1]
        cargo = "1" if has_cargo[agent_id-1] else "0"

        message = "start 0 end"
        sock.sendall(message.encode())

        # получаем ответ от сервера
        data = sock.recv(1024)

        # выводим ответ сервера
        print(f'Сервер ответил: {data.decode()}')

        sock.close()

        pattern = r"- x: (?P<x>\d+)\n\s+y: (?P<y>\d+)\n\s+t: (?P<t>\d+)"
        matches = re.finditer(pattern, data.decode())

        new_path = []

        # Вывод координат
        for match in matches:
            x = match.group("x")
            y = match.group("y")
            new_path.append((int(x), int(y)))
            # print(f"x: {x}, y: {y}")

        goal = (new_path[-1][0], new_path[-1][1])

        # print("New path: ", new_path)

        paths[agent_id-1] = new_path

        # check_crossings(paths)

        actions = []
        for num, path in enumerate(paths, 1):
            action = list(reversed(grid.path_to_action(num, path)))
            # print(action)
            actions.append(action)
            # if not nogui:
            #    vis.draw_path(path, num)
            self.update_path_vis.append[num-1]

        grid.change_goal(agent_id, goal)

        # print('**************************')
        # print('Paths after new task: ', paths)
        # print('---------------')
        # print('Actions after new task: ', actions)
        return actions


if __name__ == '__main__':

    parser = argparse.ArgumentParser()
    parser.add_argument("map", help="input file containing map")
    parser.add_argument("schedule", help="schedule for agents")
    parser.add_argument('--gui', dest='nogui', action='store_false')
    parser.add_argument('--nogui', dest='nogui', action='store_true')
    parser.set_defaults(nogui=False)
    args = parser.parse_args()

    with open(args.map) as map_file:
        map = yaml.safe_load(map_file)

    with open(args.schedule) as states_file:
        schedule = yaml.safe_load(states_file)
    
    sim = Simulation()
    sim.start(map, schedule, args.nogui)
