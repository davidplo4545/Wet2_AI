from WarehouseEnv import WarehouseEnv, manhattan_distance
import random


class Agent:
    # returns the next operator to be applied - i.e. takes one turn
    def run_step(self, env: WarehouseEnv, agent_id, time_limit):
        raise NotImplementedError()

    # returns list of legal operators and matching list of states reached by applying them
    def successors(self, env: WarehouseEnv, robot_id: int):
        operators = env.get_legal_operators(robot_id)
        children = [env.clone() for _ in operators]
        for child, op in zip(children, operators):
            child.apply_operator(robot_id, op)
        return operators, children

    def heuristic(self, env: WarehouseEnv, robot_id: int):
        robot = env.get_robot(robot_id)
        other_robot = env.get_robot((robot_id + 1) % 2)
        return robot.credit - other_robot.credit


# picks random operators from the legal ones
class AgentRandom(Agent):
    def run_step(self, env: WarehouseEnv, robot_id, time_limit):
        operators, _ = self.successors(env, robot_id)
        return random.choice(operators)


class AgentGreedy(Agent):
    def run_step(self, env: WarehouseEnv, robot_id, time_limit):
        operators = env.get_legal_operators(robot_id)
        children = [env.clone() for _ in operators]
        for child, op in zip(children, operators):
            child.apply_operator(robot_id, op)
        children_heuristics = [self.smart_heuristic(child, robot_id) for child in children]
        max_heuristic = max(children_heuristics)
        index_selected = children_heuristics.index(max_heuristic)
        return operators[index_selected]
    
    def shortest_distance_package(self, env, robot):
        # Returns the distance to package with the closest distance
        # to its position and the destination
        packages = env.packages[:2]
        first_md = manhattan_distance(robot.position, packages[0].position) + manhattan_distance(packages[0].position,packages[0].destination)
        second_md = manhattan_distance(robot.position, packages[1].position) + manhattan_distance(packages[1].position, packages[1].destination)
        if first_md < second_md:
            return first_md
        return second_md
    
    def shortest_distance_station(self, env, robot):
        # Returns the distance to the Charge Station with the closest distance
        charge_stations = env.charge_stations
        first_md = manhattan_distance(robot.position, charge_stations[0].position) 
        second_md = manhattan_distance(robot.position, charge_stations[1].position) 
        if first_md < second_md:
            return first_md
        return second_md
    
    def smart_heuristic(self, env: WarehouseEnv, robot_id: int):
        robot = env.get_robot(robot_id)
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
        h_value = 0
        if robot.package is None:
            shortest_distance = self.shortest_distance_package(env, robot)
            if shortest_distance + 2 <= robot.battery:
                h_value = robot.battery * 1000 - shortest_distance
            else:
                h_value = robot.credit * 1000 - self.shortest_distance_station(env, robot)
        else:
            curr_package = robot.package
            if manhattan_distance(robot.position, curr_package.destination) + 1 <= robot.battery:
                h_value = robot.battery * 1000 - manhattan_distance(robot.position, curr_package.destination)
            else:
                h_value = robot.credit * 1000 - manhattan_distance(robot.position, self.shortest_distance_station(env, robot))
        return h_value


