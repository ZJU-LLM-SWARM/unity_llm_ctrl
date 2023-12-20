import os
import sys

# 获取当前文件的父目录
parent_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

# 将父目录添加到sys.path中
sys.path.insert(0, parent_dir)