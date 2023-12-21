from cbs_mapf.planner import Planner
import time
import yaml

def load_scenario(fd):
    with open(fd, 'r') as f:
        global GRID_SIZE, ROBOT_RADIUS, RECT_OBSTACLES, START, GOAL
        data = yaml.load(f, Loader=yaml.FullLoader)
        GRID_SIZE = data['GRID_SIZE']
        ROBOT_RADIUS = data['ROBOT_RADIUS']
        RECT_OBSTACLES = data['RECT_OBSTACLES']
        START = data['START']
        GOAL = data['GOAL']
    RECT_OBSTACLES 

def vertices_to_obsts(obsts):
        def drawRect(v0, v1):
            o = []
            base = abs(v0[0] - v1[0])
            side = abs(v0[1] - v1[1])
            for xx in range(0, base, 30):
                o.append((v0[0] + xx, v0[1]))
                o.append((v0[0] + xx, v0[1] + side - 1))
            o.append((v0[0] + base, v0[1]))
            o.append((v0[0] + base, v0[1] + side - 1))
            for yy in range(0, side, 30):
                o.append((v0[0], v0[1] + yy))
                o.append((v0[0] + base - 1, v0[1] + yy))
            o.append((v0[0], v0[1] + side))
            o.append((v0[0] + base - 1, v0[1] + side))
            return o
        static_obstacles = []
        for vs in obsts.values():
            static_obstacles.extend(drawRect(vs[0], vs[1]))
        for i in range(len(static_obstacles)):
            static_obstacles[i] = (static_obstacles[i][0] + 600, static_obstacles[i][1] + 600)
        for i in range(len(START)):
            START[i] = (START[i][0] + 600, START[i][1] + 600)
            GOAL[i] = (GOAL[i][0] + 600, GOAL[i][1] + 600)
        return static_obstacles

def find_path():
        static_obstacles = vertices_to_obsts(RECT_OBSTACLES)
        planner = Planner(GRID_SIZE, ROBOT_RADIUS, static_obstacles)
        # before = time.time()
        path = planner.plan(START, GOAL, debug=False)
        # after = time.time()
        # print('Find Path Time elapsed:', "{:.4f}".format(after-before), 'second(s)')
        d = dict()
        for i, path in enumerate(path):
            d[i] = path
        path = d
        write_path_to_txt(path)

def write_path_to_txt(path):
    for k in range(len(path)):
        with open('txt_data/00{}.txt'.format(k), 'w') as file:
            for i in range(len(path[k])):
                file.write(str(path[k][i][0] - 600) + ' ' + str(path[k][i][1] -600) + '\n')

if __name__ == '__main__':
    load_scenario("path_ctrl\scenario1.yaml")
    find_path()

