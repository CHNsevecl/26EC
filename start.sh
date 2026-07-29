#!/bin/bash

echo "🚀 启动 ZMQ 图像传输服务..."

# C++程序路径
CPP_PATH="/home/sevecl/Desktop/C/Example/QD4310控制模版/build/qd"

# Python脚本路径
PY_SCRIPT="/home/sevecl/Desktop/PY/Zero_MQ/单目测距/clinet.py"

# 启动C++程序
echo "✅ C++程序已启动"
"$CPP_PATH" &

# 等待2秒
sleep 1

# 启动Python程序（虚拟环境）
echo "✅ Python服务已启动"
/home/sevecl/Desktop/PY/Zero_MQ/zmq_env/bin/python "$PY_SCRIPT" &

echo "✅ 全部启动完成"
echo "🌐 访问地址: http://$(hostname -I | awk '{print $1}'):8080"

# 等待所有后台进程
wait
