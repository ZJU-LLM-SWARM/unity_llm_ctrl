import json
class AgentState:
    def __init__(
        self, type, id, position, rotation, velocity, target, distance_to_target
    ):
        self.type = type  # a uav or car
        self.id = id
        self.position = position  # a list of [x, y, z]
        self.rotation = rotation  # a list of [x, y, z]
        self.velocity = velocity  # a list of [x, y, z]
        self.target = target  # a bool
        self.distance_to_target = distance_to_target  # a float

        if self.type == "car":
            self.car_manual_control = [
                False,
                False,
                False,
                False,
                False,]  # 前进，后退，左转，右转，刹车
        else:
            self.drone_speed_control = [0, 0, 0]  # xyz速度

    def encode_state_json(self):
        # 将状态信息编码为JSON格式
        if self.type == "car":
            state = {
                "type": self.type,
                "id": int(self.id),
                "position": self.position,
                "rotation": self.rotation,
                "velocity": self.velocity,
                "target": int(self.target),  # 将布尔值转换为整数
                "distance_to_target": self.distance_to_target,
                "car_manual_control": self.car_manual_control,
                #新加的状态量在这里加入 但是注意c#端同步对齐
            }
        elif self.type == "uav":
            state = {
                "type": self.type,
                "id": int(self.id),
                "position": self.position,
                "rotation": self.rotation,
                "velocity": self.velocity,
                "target": int(self.target),  # 将布尔值转换为整数
                "distance_to_target": self.distance_to_target,
                "drone_speed_control": self.drone_speed_control,
                #新加的状态量在这里加入 但是注意c#端同步对齐
            }
        return json.dumps(state)

def get_agent_list():
    agents_state = []
    with open("json_data/agentsettings.json", "r") as f:
        settings = json.load(f)
    agents = settings["agents"]
    for agent in agents:
        type = agent["type"]
        id = agent["id"]
        position = agent["position"]
        rotation = agent["rotation"]
        velocity = agent["velocity"]
        target = agent["target"]
        distance_to_target = agent["distance_to_target"]
        agent_state = AgentState(
            type, id, position, rotation, velocity, target, distance_to_target
        )
        agents_state.append(agent_state)
    return agents_state
