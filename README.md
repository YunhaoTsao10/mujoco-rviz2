Step 1: 创建工作空间并克隆 mj2rviz 仓库 Create a workspace and clone the mj2rviz repository

    确保你已经在 ROS2 工作空间的 src 文件夹下。例如： Make sure you're in the src folder of your ROS2 workspace. For example:
    
        mkdir -p ~/ros2_ws/src/mj2rviz
        cd ~/ros2_ws/src/mj2rviz
        
    克隆 mj2rviz 仓库： Clone the mj2rviz repository:
    
        git clone https://github.com/YunhaoTsao10/mujoco-rviz2.git mj2rviz

Step 2: 编辑 .bashrc 文件 Edit the .bashrc file

    打开 .bashrc 文件进行编辑：
    Open the .bashrc file for editing:

        nano ~/.bashrc  

    在文件的最后一行添加以下内容：
    Add the following line at the end of the file:

        export CONFIG_FILE_PATH="[absolute path to the paths.yaml file ]"

    保存并退出：
    Save and exit:
    
        在 nano 中，按下 Ctrl+O 保存文件，按下 Enter 确认，接着按 Ctrl+X 退出。
        In nano, press Ctrl+O to save, press Enter to confirm, and Ctrl+X to exit.
    
    刷新环境变量：
    Refresh the environment variables:

        source ~/.bashrc

Step 3: 配置 config 文件夹里的 paths.yaml 文件 Configure the paths.yaml file in the config folder

    将 [ ] 中的内容替换为当前文件的绝对路径，old_mesh_dir 行内容请勿更改！
    Replace the contents inside [ ] with the absolute path to the current file. Do not modify the old_mesh_dir line!

Step 4: 运行 config 文件夹里的 formater.py Run the formater.py script in the config folder

Step 5: 编译 mj2rviz 包 Build the mj2rviz package

    colcon build --packages-select mj2rviz

Step 6: 运行 ROS2 启动命令 Run the ROS2 launch command

    ros2 launch mj2rviz state_pub_launch.py

Node diagram:
    ![rosgraph](https://github.com/user-attachments/assets/e8f3aba1-f23f-4146-bda1-f0f6779cb85b)

Demo:
    ![demo](https://github.com/user-attachments/assets/6b5b5a60-3bc3-4cb6-b123-2d8a5c511551)






