#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""加载独立 launch 文件并运行 (ros2 launch 不认绝对路径 py 文件的 workaround)

用法: python3 launch_runner.py /home/nvidia/nav2_nav/nav2_launch.py [launch args...]
"""
import importlib.util
import sys

from launch import LaunchService

path = sys.argv[1]
spec = importlib.util.spec_from_file_location('target_launch', path)
mod = importlib.util.module_from_spec(spec)
spec.loader.exec_module(mod)
ld = mod.generate_launch_description()

ls = LaunchService(argv=sys.argv[2:])
ls.include_launch_description(ld)
ls.run()
