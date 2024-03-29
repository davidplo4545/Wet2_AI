from Agent import Agent, AgentGreedy
from WarehouseEnv import WarehouseEnv, manhattan_distance
import random


def shortest_distance_package(env, robot):
    # Returns the distance to package with the closest distance
    # to its position and the destination
    packages = env.packages[:2]
    first_md = manhattan_distance(robot.position, packages[0].position) + manhattan_distance(packages[0].position,packages[0].destination)
    second_md = manhattan_distance(robot.position, packages[1].position) + manhattan_distance(packages[1].position, packages[1].destination)
    if first_md < second_md:
        return packages[0], first_md
    return packages[1], second_md

def shortest_distance_station(env, robot):
    # Returns the distance to the Charge Station with the closest distance
    charge_stations = env.charge_stations
    first_md = manhattan_distance(robot.position, charge_stations[0].position)
    second_md = manhattan_distance(robot.position, charge_stations[1].position)
    if first_md < second_md:
        return charge_stations[0], first_md
    return charge_stations[1], second_md

def smart_heuristic(env: WarehouseEnv, robot_id: int):
    # case: no package and enough battery to pick up and deliver
    # if !package and min(md(pos, package_pos) + md(package_pos, package_dest) + 2) <= battery
    # then h =  battery * 1000 - (md(pos, package_pos) + md(package_pos, package_dest))
    # case: no package and not enough better to pick up and deliver
    # if !package and min(md(pos, package_pos) + md(package_pos, package_dest) + 2) > battery
    # then h =  credit * 1000 - md(pos, closest_station)

    # case: has package and enough battery to deliver
    # if package and md(pos, package_dest) + 1) <= battery
    # then h = battery * 1000 - md(pos, package_dest)

    # case: has package and not enough battery to deliver
    # if package and md(pos, package_dest) + 1) > battery
    # then h =  credit * 1000 - md(pos, closest_station)

    # if r0.row == r1.row == closest_package.col and r0.col > r1.col > closest_package.col
    robot = env.get_robot(robot_id)
    other_robot = env.get_robot((robot_id + 1) % 2)
    import time
    h_value = 0
    closest_station, station_distance = shortest_distance_station(env, robot)
    closest_package,shortest_distance = shortest_distance_package(env, robot)

    if robot.battery > other_robot.battery and robot.credit > other_robot.credit:
        h_value = robot.credit * 1000 + robot.battery * 100 - shortest_distance
    elif robot.package is None:
        if shortest_distance + 2 <= robot.battery:
            h_value = robot.battery * 1000 - shortest_distance
        else:
            h_value = robot.credit * 1000 - station_distance
    else:
        curr_package = robot.package
        if manhattan_distance(robot.position, curr_package.destination) + 1 <= robot.battery:
            h_value = robot.battery * 1000 - manhattan_distance(robot.position, curr_package.destination)
        else:
            h_value = robot.credit * 1000 - manhattan_distance(robot.position, closest_station.position)
    return h_value




class AgentGreedyImproved(AgentGreedy):
    def heuristic(self, env: WarehouseEnv, robot_id: int):
        return smart_heuristic(env, robot_id)

import time

# AgentAlphaBeta

class AgentAlphaBeta(Agent):
    def __init__(self):
        self.ended = False
    # TODO: section b : 1

    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        start_time = time.monotonic()
        self.ended = False
        finish_time = start_time + time_limit - 0.1

        best_move = env.get_legal_operators(agent_id)[0]
        curr_depth = 1
        while True:
            if time.monotonic() > finish_time:
                return best_move
            _, curr_move = self.alpha_beta(env, agent_id, curr_depth, agent_id, finish_time, float("-inf"), float("inf"))
            if not self.ended and curr_move != None:
                best_move = curr_move
            else:
                break
            curr_depth += 1
        return best_move

    def succ(self, state, turn):
        children = []
        operators = state.get_legal_operators(turn)
        children = [(state.clone(), op) for op in operators]
        for child, op in children:
            child.apply_operator(turn, op)
        return children

    def alpha_beta(self, state, agent_id, depth, turn, finish_time, alpha, beta):
        if time.monotonic() > finish_time:
            self.ended = True
            return 0, None
        if depth == 0 or state.done():
            return smart_heuristic(state, agent_id), None
        children = self.succ(state, turn)
        if turn == agent_id:
            cur_max = float("-inf")
            best_action = None
            for c, operator in children:
                v, _ = self.alpha_beta(c, agent_id, depth - 1, (turn+1)%2, finish_time, alpha, beta)
                if time.monotonic() > finish_time:
                    self.ended = True
                    return 0, None
                if cur_max < v:
                    best_action = operator
                    cur_max = v
                # alpha-beta
                alpha = max(cur_max, alpha)
                if cur_max >= beta:
                    return float("inf"), best_action
            return cur_max, best_action
        else:
            cur_min = float("inf")
            best_action = None
            for c, operator in children:
                v, _ = self.alpha_beta(c, agent_id, depth - 1, (turn+1)%2, finish_time, alpha, beta)
                if time.monotonic() > finish_time:
                    self.ended = True
                    return 0, None
                if v < cur_min:
                    best_action = operator
                    cur_min = v
                # alpha-beta
                beta = min(cur_min, beta)
                if cur_min <= alpha:
                    return float("-inf"), best_action
            return cur_min, best_action

class AgentMinimax(Agent):
    # TODO: section c : 1
    def __init__(self):
        self.ended = False
    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        start_time = time.monotonic()
        self.ended = False
        finish_time = start_time + time_limit - 0.1

        best_move = env.get_legal_operators(agent_id)[0]
        curr_depth = 1
        while True:
            if time.monotonic() > finish_time:
                return best_move
            _, curr_move = self.minmax(env, agent_id, curr_depth, agent_id, finish_time)
            if not self.ended and curr_move != None:
                best_move = curr_move
            else:
                break
            curr_depth += 1
        return best_move

    def succ(self, state, turn):
        children = []
        operators = state.get_legal_operators(turn)
        children = [(state.clone(), op) for op in operators]
        for child, op in children:
            child.apply_operator(turn, op)
        return children

    def minmax(self, state, agent_id, depth, turn, finish_time):
        if time.monotonic() > finish_time:
            self.ended = True
            return 0, None
        if depth == 0 or state.done():
            return smart_heuristic(state, agent_id), None
        children = self.succ(state, turn)
        if turn == agent_id:
            cur_max = float("-inf")
            best_action = None

            for c, operator in children:
                v, _ = self.minmax(c, agent_id, depth - 1, (turn+1)%2, finish_time)
                if time.monotonic() > finish_time:
                    self.ended = True
                    return 0, None

                if cur_max < v:
                    best_action = operator
                    cur_max = v

            return cur_max, best_action
        else:
            cur_min = float("inf")
            best_action = None

            for c, operator in children:
                v, _ = self.minmax(c, agent_id, depth - 1, (turn+1)%2, finish_time)
                if time.monotonic() > finish_time:
                    self.ended = True
                    return 0, None
                if v < cur_min:
                    best_action = operator
                    cur_min = v
            return cur_min, best_action



class AgentExpectimax(Agent):
    # TODO: section d : 1
    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        raise NotImplementedError()


# here you can check specific paths to get to know the environment
class AgentHardCoded(Agent):
    def __init__(self):
        self.step = 0
        # specifiy the path you want to check - if a move is illegal - the agent will choose a random move
        self.trajectory = ["move north", "move east", "move north", "move north", "pick_up", "move east", "move east",
                           "move south", "move south", "move south", "move south", "drop_off"]

    def run_step(self, env: WarehouseEnv, robot_id, time_limit):
        if self.step == len(self.trajectory):
            return self.run_random_step(env, robot_id, time_limit)
        else:
            op = self.trajectory[self.step]
            if op not in env.get_legal_operators(robot_id):
                op = self.run_random_step(env, robot_id, time_limit)
            self.step += 1
            return op

    def run_random_step(self, env: WarehouseEnv, robot_id, time_limit):
        operators, _ = self.successors(env, robot_id)

        return random.choice(operators)