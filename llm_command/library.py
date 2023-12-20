# import LLM_env.Assets.LLM_ENV.gpt_create_path_planning as pcf
from __init__ import start

def track(agent_list, target_position, expected_distance_list, track_direction_list):
    pass

def search(agent_list, target_area, search_style):
    pass

def simple_formation(target_position):
    # Open the file in write mode
    with open('txt_data/000.txt', 'w') as file:
        # 编队目标点定义
        file.write(str(target_position[0]) +  ' ' + str(target_position[2]))
    start.main_control(1)

