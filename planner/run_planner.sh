#!/bin/bash
# 保存为 debug_planner.sh

# 设置环境变量
export LD_LIBRARY_PATH="/home/yang/Planning/PCT_planner/planner/lib/3rdparty/gtsam-4.1.1/install/lib:$LD_LIBRARY_PATH"
export LD_LIBRARY_PATH="/home/yang/Planning/PCT_planner/planner/lib/build/src/common/smoothing:$LD_LIBRARY_PATH" 
export LD_LIBRARY_PATH="/home/yang/Planning/PCT_planner/planner/lib:$LD_LIBRARY_PATH"
export PYTHONPATH="/home/yang/Planning/PCT_planner/planner/lib:$PYTHONPATH"

cd /home/yang/Planning/PCT_planner/planner/scripts

# 使用GDB调试
gdb -ex "set environment LD_LIBRARY_PATH=$LD_LIBRARY_PATH" \
    -ex "set environment PYTHONPATH=$PYTHONPATH" \
    -ex "run plan.py --scene Spiral" \
    -ex "bt" \
    -ex "quit" \
    python3