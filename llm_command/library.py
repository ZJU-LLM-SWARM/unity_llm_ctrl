# import LLM_env.Assets.LLM_ENV.gpt_create_path_planning as pcf
from __init__ import start
import subprocess
def track(agent_list, target_position, expected_distance_list, track_direction_list):
    pass

def search(agent_list, target_area, search_style):
    pass

def simple_formation(target_position):
    # 根据输入的目标点为每个智能体集群的领导者分配目标点
    # 还需要补充

    # 现在是在yaml中指定目标点，将生成的路径输出到txt中
    # 启动一个新的Python进程来运行另一个Python文件
    process = subprocess.Popen(['python', 'path_ctrl/cbs_find_path.py'])
    # 等待上一个进程结束
    process.wait()
    start.socket_main(1)

