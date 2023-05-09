# RMDS motor driver

RMDS 系列电机驱动，需要 ros1 下的 ros-$ROS_DISTRO-serial 包支持

## Prerequests

```bash
sudo apt install ros-$ROS_DISTRO-serial
```

## Compile

```bash
cd path/to/your/workspace/
catkin_make
```

### Compile example
```bash
cd path/to/example/
cmake -S . -B build
cmake --build build
# 可执行文件就是 build/main
```

## Usage

示例代码在 `example/main.cpp` 中

### 初始化

```cpp
std::string dev("电机设备路径");
int id; // 电机 id，跟电机上的按钮配置有关，如果没动过就是 1
Motor::RMDS motor(dev, 1);
```

### 控制电机动作

```cpp
double degree; // 电机动作角度
double vel; // 电机动作角速度

// 旋转到指定位置，电机有绝对编码器，零点位置有专门的配置软件
// 可以指定旋转速度，默认 180°/s
// 如果之前指定过旋转速度，则后面再不指定速度，都按照先前设定的速度旋转
motor.rotateTo(degree);
motor.rotateTo(degree, vel);

// 旋转到指定位置 + degree 角度，电机有绝对编码器，零点位置有专门的配置软件
motor.rotateMore(degree);
motor.rotateMore(degree, vel);

// 按照指定速度旋转
motor.rotate(vel);
motor.rotate(); // 如果不指定速度则按照先前设定的速度旋转

// 获取当前角度
motor.getCurrentPose();

// 暂停工作
motor.pause();

// 减速电机通常需要绝对编码器转过多圈，电机的转子才能旋转一圈，因此
// 设计了适用于减速电机的控制指令
motor.rotateMTo(degree);
motor.rotateMTo(degree, vel);
motor.rotateMMore(degree);
motor.rotateMMore(degree, vel);
motor.getCurrentMPose();
```