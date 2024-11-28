# Step 1: 创建工作空间并克隆 mj2rviz 仓库 Create a workspace and clone the mj2rviz repository

确保你已经在 ROS2 工作空间的 src 文件夹下。例如：
Make sure you're in the src folder of your ROS2 workspace. For example:

    mkdir -p ~/ros2_ws/src
    cd ~/ros2_ws/src
    
克隆 mj2rviz 仓库：
Clone the mj2rviz repository:

    git clone https://github.com/YunhaoTsao10/mujoco-rviz2.git mj2rviz
    
# Step 2: 编辑 .bashrc 文件 Edit the .bashrc file

打开 .bashrc 文件进行编辑：
Open the .bashrc file for editing:

    nano ~/.bashrc
    
在文件的最后一行添加以下内容：
Add the following line at the end of the file:

    export CONFIG_FILE_PATH="[absolute path to the paths.yaml file]"
    
保存并退出：
Save and exit:

在 nano 中，按下 Ctrl+O 保存文件，按下 Enter 确认，接着按 Ctrl+X 退出。
In nano, press Ctrl+O to save, press Enter to confirm, and Ctrl+X to exit.

刷新环境变量：
Refresh the environment variables:

    source ~/.bashrc
    
# Step 3: 配置 config 文件夹里的 paths.yaml 文件 Configure the paths.yaml file in the config folder

将 [ ] 中的内容替换为当前文件的绝对路径，old_mesh_dir 行内容请勿更改！
Replace the contents inside [ ] with the absolute path to the current file. Do not modify the old_mesh_dir line!

### Step 4: 运行 config 文件夹里的 formater.py Run the formater.py script in the config folder

### Step 5: 运行 ROS 2 Humble 版本的环境设置脚本 Run the ROS 2 Humble environment setup script

如果此命令之前没有添加到 ~/.bashrc：
If this command has not been added to ~/.bashrc before:

    source ~/ros2_ws/install/setup.bash
    
# Step 6: 安装缺少的包和依赖 Install packages and dependencies

    sudo apt update
    sudo apt install ros-humble-joint-state-publisher-gui
    pip install -r requirements.txt
    
# Step 7: 编译 mj2rviz 包 Build the mj2rviz package

    colcon build --packages-select mj2rviz
    
# Step 8: 运行 ROS2 启动命令 Run the ROS2 launch command

    ros2 launch mj2rviz state_pub_launch.py
    
## 注意 (Note):请确保所有的项目依赖已正确安装！Make sure all project dependencies are correctly installed!

# Node Diagram 节点图
![rosgraph](https://github.com/user-attachments/assets/e8f3aba1-f23f-4146-bda1-f0f6779cb85b)

# Demo with Unitree H1 宇树h1示例
![demo](https://github.com/user-attachments/assets/6b5b5a60-3bc3-4cb6-b123-2d8a5c511551)

# Demo with my Exoskeleton Project 我的外骨骼项目示例 The source code for this project will NOT be open-sourced
![demo_exoskeleton](https://github.com/user-attachments/assets/c9d5dab1-6348-43e8-80ac-43bc81c67d14)

### 在使用自己项目的模型时生成urdf与对应xml文件的过程是少不了的，然而自己生成的模型描述文件与开源的Unitree h1的可能会有差别。下面我会对我进行urdf文件生成的过程进行简单描述： The process of generating URDF and XML files for your project’s model is essential. Compared to open-source models like Unitree H1, there might be differences in the model description files. Below is a step-by-step guide to creating a URDF file for your project:

## 1.机械模型的设计与创建 Design and creation of the mechanical model

机械模型的设计与创建可以使用 Catia, Creo, UG, SolidWorks, Fusion360 等工业设计软件。这里推荐 SolidWorks 或 Fusion360，因为这两款软件的插件库里有支持装配文件转 urdf 文件的快捷插件。
You can use industrial design software such as Catia, Creo, UG, SolidWorks, or Fusion360 for designing and creating the mechanical model. We recommend SolidWorks or Fusion360 because they have convenient plugins for exporting assembly files to URDF format.
    
此项目中，我全程使用的是 Fusion360 进行零部件装配与导出。可以在 Fusion360 的官方插件库中搜索 “urdf” 关键词找到插件。安装完插件后如下：
In this project, I used Fusion360 throughout for component assembly and export. You can find the plugin by searching for "urdf" in the Fusion360 official plugin library. After installation, it looks like this:

<img width="342" alt="1" src="https://github.com/user-attachments/assets/22fac213-d48a-41db-b685-7b5022ca624b">

## 2.装配模型时的注意事项 Precautions during model assembly

1. 模型的站立方向与世界坐标系的 Z 方向一致 Ensure the model's standing direction aligns with the Z-axis of the world coordinate system.
2. 每一个关联关节不能有嵌套关系。链接方式必须是 a->b->c->d，不可以是 a->(b->c->d)。 Each joint connection must not have nested relationships. The linking must be a->b->c->d, not a->(b->c->d).
3. 每个关节的自由度不能大于 2，自用全程采用 revolute joint。 Each joint must have no more than 2 degrees of freedom. I used revolute joints throughout.
4. **重要！装配后的模型必须高于 Z0 平面，否则导出 URDF 文件时会报错。** **Critical! Ensure the assembled model is above the Z0 plane; otherwise, exporting to a URDF file will cause errors.**
   
完成的装配模型大致如下：
The completed assembly model looks like this:
<img width="640" alt="2" src="https://github.com/user-attachments/assets/93994f0d-e353-479d-a936-fd8679d5493e">

## 3.导出 URDF 文件 Export the URDF file

装配完成后，使用先前安装的插件导出 URDF 文件：
After assembly, use the installed plugin to export the URDF file:
<img width="318" alt="3" src="https://github.com/user-attachments/assets/c11b8afe-faa1-45e9-a893-5b3363fd5128">

## 4. STL 文件转化与路径修改 (STL to DAE Conversion and Path Modification)

导出 URDF 文件后，打开文件检查 filename 标签引用的文件路径。默认情况下，URDF 文件中引用的是 .stl 格式文件。
After exporting the URDF file, open it and check the filename tag for referenced file paths. By default, the URDF file references .stl files.

为了增强兼容性，需要将 meshes 文件夹中的 .stl 模型转换为 .dae 格式，同时在 URDF 文件中将对应路径更新为 .dae 文件路径。
For better compatibility, convert the .stl models in the meshes folder to .dae format, and update the corresponding file paths in the URDF file to reference the .dae files.

注意：保留原始的 .stl 模型文件，**不要删除**。后续 XML 文件需要使用这些 .stl 文件。可以使用工具（例如 Blender 或 Meshlab）完成文件格式转换和模型编辑，**Mujoco对于单个stl模型的要求是面数不能高于100000**。
Note: Keep the original .stl model files; **do not delete them**. These .stl files will be required later for the XML file.
You can use tools such as Blender or Meshlab for file format conversion and stl model editing, **Mujoco only allows a single stl model with faces fewer than 100000**.

## 5. 修改根基零件的惯性描述 (Modify Base Component's Inertia Description)

找到 URDF 文件中的根基零件。它的名字通常是 base_link，但在您的项目中可能有其他命名，例如 "EB"。
Locate the base component in the URDF file. Its name is typically base_link, but in your project, it might have a different name, such as "EB".

关键点：

根基零件是所有其他零件的父级零件，决定了机器人相对于世界坐标系的位置和关系。
The base component is the parent of all other components and determines the robot's position and relationship to the world coordinate system.
**删除根基零件的 <inertia> 部分，仅保留 <visual> 和 <collision> 部分。
Remove the <inertia> section for the base component and retain only the <visual> and <collision> sections.**
如果未删除 <inertia>，RViz 无法加载该 URDF 文件。
If <inertia> is not removed, RViz will fail to load the URDF file.

## 6. 使用 Mujoco 转换为 XML 并添加控制器 (Convert to XML with Mujoco and Add Controllers)

将修改完成的 URDF 文件通过 Mujoco 提供的 compile 脚本转换为 XML 文件：
Use Mujoco's compile script to convert the modified URDF file into an XML file:

<path_to_mujoco_bin>/compile <path_to_urdf_file> <output_xml_file>
转换完成后，请按以下步骤完善 XML 文件：
After conversion, complete the XML file with the following steps:

### 添加控制器 (<actuator>)
为机器人模型的关节添加控制器（例如电机）。具体配置方法请参考 Mujoco 官方文档。
Add actuators (e.g., motors) for the robot's joints. Refer to the Mujoco documentation for detailed configuration.

### 忽视间隙碰撞 (<contact exclude>)
在 XML 文件中，可以通过 <contact> 标签使用 <exclude> 子标签忽视因模型间隙引发的非正常碰撞问题。
Use <contact> with <exclude> tags in the XML file to ignore abnormal collisions caused by model gaps.
手动添加根基零件的 <body> 标签
默认情况下，URDF 转换生成的根基零件只有 <geom> 描述，而没有 <body> 标签。为了明确根基零件相对于世界坐标系的初始位置，您需要手动添加 <body> 标签，并将其设置为 6 自由度的 freejoint：
By default, the URDF-to-XML conversion generates only a <geom> description for the base component without a <body> tag. To define the base component's initial position relative to the world coordinate system, manually add a <body> tag and set it as a 6-DOF freejoint:

<img width="734" alt="截屏2024-11-28 20 27 17" src="https://github.com/user-attachments/assets/e480874c-8525-4a3a-931e-9661acf11472">

  
例如，在此项目中，其位置为 (0m, 0m, 1.6m)。
In this project, its position is (0m, 0m, 1.6m).

## 节点代码部分 (Node Code Section)

pub_test 节点
pub_test 节点的内容无需更改。
No changes are required for the pub_test node.
simulation 节点
simulation 节点需要注意以下事项：
For the simulation node, ensure the following:
发布的 /mujoco_joint_state 话题的数据结构（包括关节数量和名称）必须与 joint_state_publisher 节点发布的 /joint_state 话题完全一致。
The /mujoco_joint_state topic published by this node must have the same structure (including joint count and names) as the /joint_state topic published by the joint_state_publisher node.
/mujoco_tf 话题的数据结构应与 joint_state_publisher 节点的 /tf 话题一致。此外，还需要增加根基零件与世界坐标系 world 的 TF 关系。
The /mujoco_tf topic should have the same structure as the /tf topic from the joint_state_publisher node. Additionally, include the TF relationship between the base component and the world coordinate frame.
如果发现不一致，需根据项目的 XML 模型结构修改 simulation.py 文件中的相关代码。
If inconsistencies are found, update the relevant code in simulation.py based on your project's XML model structure.

# 调试工具 (Debugging Tools)

问：如何查看 ROS2 项目运行时的所有话题及其内容结构？
Q: How can I check all topics and their structures during a ROS2 project run?

答：

使用以下命令列出所有话题：
Use the following command to list all topics:

    ros2 topic list
    
使用以下命令查看指定话题的内容和结构（仅打印一次）：
Use the following command to inspect the content and structure of a specific topic (print once):

    ros2 topic echo <topic_name> --once
    
### 祝你成功完成项目移植！:)
### Wishing you success with your project migration! :)







