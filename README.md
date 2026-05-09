# robot_status_reporter

  

## 一、项目简介

  

`robot_status_reporter` 是一个用于机器人状态采集与标准化发布的 ROS 2 功能包。

  

它的主要作用是把外部 SLAM、定位、底盘或传感器层提供的原始状态，整理成上层决策系统可以直接使用的标准消息，尤其是持续发布机器人状态信息，供后续规划与控制节点使用。

  

---

  

## 二、功能介绍

  

本模块主要完成以下工作：

  

1. 订阅外部状态输入，例如电量、药量、TF 等信息。

2. 将机器人当前位置、姿态、资源状态等信息统一封装。

3. 对外发布标准 ROS 2 消息，便于其他模块直接订阅。

4. 为上层系统提供稳定的状态输入接口。

  

---

  

## 三、当前实现说明

当前代码中，核心节点是 `status_aggregator`，它会：

1. **带参启动**：支持配置全局系、机体系以及报告频率。
2. **状态订阅**：订阅高频物理传感话题（`/battery_state` 和 `/medicine_inventory`）。
3. **坐标监听**：通过 TF 查询 `map -> base_link`。
4. **防呆保护机制**：如果上游定位模块没有就绪（即取不到 TF），本节点会安全挂起，并每 5 秒提示等待有效位姿，严禁向外发送全 0 位姿迷惑下游路径规划模块。
5. **状态对齐与下发**：当定位恢复，节点以固定频率打包发布 `/robot_integrated_status`。

---

  

## 四、目录结构

```text
robot_status_reporter/
├── CMakeLists.txt
├── package.xml
├── launch/
│   └── status_reporter.launch.py
├── msg/
│   └── DeliveryStatus.msg
└── robot_status_reporter/
    ├── __init__.py
    └── status_aggregator.py
```

  

---

  

## 五、依赖环境

  

- ROS 2

- Python 3

- `rclpy`

- `tf2_ros`

- `sensor_msgs`

- `std_msgs`

- `geometry_msgs`

  

---

  

## 六、编译

  

在工作空间根目录执行：

  

```bash

colcon build --packages-select robot_status_reporter

source install/setup.bash

```

  

如果是首次编译整个工作空间，也可以直接执行：

  

```bash

colcon build

source install/setup.bash

```

  

---

  

## 七、启动与联调测试方式

为了完整测试该模块向外发送信息的机制，你需要模拟真实的上下游数据流。请打开多个终端严格按照以下顺序执行：

### 1. 启动状态聚合节点（主站）

**终端 1** 中执行：

```bash
source install/setup.bash
ros2 launch robot_status_reporter status_reporter.launch.py
```
> *注意：此时如果系统内没有给它发定位数据，终端会进入安全保护状态，每 5 秒提示一次 `Waiting for valid robot pose before publishing integrated status...`*

### 2. 模拟 SLAM 模块发送定位 TF（解锁下发机制）

打开 **终端 2** 执行：

```bash
source install/setup.bash
ros2 run tf2_ros static_transform_publisher 1.0 2.0 0.0 0.0 0.0 0.0 map base_link
```
> *执行后，你应该能看到 终端 1 的等待报错立刻消失，它开始默默在底层向外发布综合状态了。*

### 3. 查看最终对外发送的综合结构

打开 **终端 3** 执行：

```bash
source install/setup.bash
ros2 topic echo /robot_integrated_status
```
> *此时你会看到打包好的坐标位置源源不断地输出。*

### 4. （可选）模拟发送底盘资源数据

如果想看其余数值的变化，可以打开 **终端 4** 模拟发送假数据，然后看 终端 3 的打印变化：

```bash
# 发布 85% 电量
source install/setup.bash
ros2 topic pub /battery_state sensor_msgs/msg/BatteryState "{percentage: 0.85}"

# 发布当前药盒里的余量 (如 10)
source install/setup.bash
ros2 topic pub /medicine_inventory std_msgs/msg/Int32 "{data: 10}"
```

---

## 八、参数与话题说明

### 启动参数 (Parameters)

- `global_frame_id`：全局坐标系名称（默认: `map`）
- `robot_frame_id`：机器人坐标系名称（默认: `base_link`）
- `update_rate`：向外打包下发的频率（默认: `5.0` Hz）

### 输入话题

  

- `/battery_state`：电量信息，类型为 `sensor_msgs/BatteryState`

- `/medicine_inventory`：药量信息，类型为 `std_msgs/Int32`

- `/tf`：TF 变换树，用于获取 `map -> base_link`

  

### 输出话题

  

- `/robot_integrated_status`：聚合后的机器人状态，类型为自定义消息 `DeliveryStatus`

  

---

  

## 九、与上层模块（如 Decision）的通信联调方式

上游决策模块接收并使用本节点状态时，开发人员需要执行以下修改或订阅操作才能真正“接收信息”：

1. **导入依赖报文格式**  
   上层功能包（例如 `multi_agent_decision-main`）的 `package.xml` 中，需要增加对本包的强依赖：
   ```xml
   <depend>robot_status_reporter</depend>
   ```

2. **Python 节点导入实体**  
   在想要拿到综合状态的决策规划节点（如 `task_planning_node.py` 或 `fleet_manager_node.py`）文件头部，添加导入指令：
   ```python
   from robot_status_reporter.msg import DeliveryStatus
   ```

3. **创建订阅并在回调中解析信息**  
   在类初始化函数中创建针对本节点打包信息的 Topic 监听，并编写 `callback` 以实时读取位置、电量与装载量：
   ```python
   # 示例：上层某节点的监听设定
   self.status_sub = self.create_subscription(
       DeliveryStatus,
       '/robot_integrated_status',
       self.status_receive_callback,
       10
   )

   def status_receive_callback(self, msg: DeliveryStatus):
       # 此时就可以直接调用解包使用这些值
       self.get_logger().info(
           f'最新位置: ({msg.current_pose.position.x:.2f}, {msg.current_pose.position.y:.2f}) '
           f'剩余电量: {msg.battery_level}% '
           f'载药量:   {msg.medicine_count}'
       )
       # 然后拿这批数据作判断，当电量不足时自动触发返航状态机等。
   ```

如果在联调中你发现话题一直发不出数据，请回头参考 **第七节** ，确保外部系统里 SLAM/自动导航系统 正常建立起了 `map` 从而让本模块被成功唤醒。

---

  

## 十、常见问题

  

### 1. `ros2 launch` 提示找不到 launch 文件

  

请确认你执行的是下面这条命令：

  

```bash

ros2 launch robot_status_reporter status_reporter.launch.py

```

  

### 2. 节点启动了但没有数据

  

这是因为上游话题还没有发布，或者 TF 树里还没有 `map -> base_link`。

  

### 3. 看不到输出话题

  

请先检查：

  

- 是否执行了 `source install/setup.bash`

- 上游传感器节点是否启动

- `/battery_state`、`/medicine_inventory`、`/tf` 是否存在

  

---

  

## 十一、总结

  

`robot_status_reporter` 的核心作用是：

  

- 接入外部状态数据

- 统一整理机器人状态

- 对外发布可被上层模块直接使用的结果

  

它是一个状态适配与聚合模块，不负责路径规划和决策。
