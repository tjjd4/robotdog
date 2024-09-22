# ROS 2 Robot Dog Project

Welcome to the ROS 2 Robot Dog project! This repository provides a comprehensive guide to building, running, and extending a ROS 2-based robot dog simulation using Docker. Whether you're a beginner or an experienced ROS 2 developer, this guide will help you set up your environment, create new ROS 2 nodes, and interact with the robot dog simulation seamlessly.

## Table of Contents

- [Introduction](#introduction)
- [Prerequisites](#prerequisites)
- [Building the Docker Image](#building-the-docker-image)
  - [Build the Image](#build-the-image)
  - [Run the Container](#run-the-container)
- [Attaching to a Running Container](#attaching-to-a-running-container)
  - [Using VSCode Dev Container Extension](#using-vscode-dev-container-extension)
  - [Initializing the Environment Inside the Container](#initializing-the-environment-inside-the-container)
- [Testing ROS 2 Commands](#testing-ros-2-commands)
- [Creating a New ROS 2 Node](#creating-a-new-ros-2-node)
  - [Create a New Package](#create-a-new-package)
  - [Add Node Script](#add-node-script)
  - [Configure `setup.py`](#configure-setuppy)
  - [Build the Workspace](#build-the-workspace)
  - [Run the New Node](#run-the-new-node)
- [Example Output](#example-output)
- [Troubleshooting](#troubleshooting)
- [License](#license)

---

## Introduction

This project leverages ROS 2 (Robot Operating System 2) to simulate a robot dog with integrated camera and control nodes. Docker is used to containerize the environment, ensuring consistency across different development setups. The Flask web server provides a user-friendly interface to interact with the robot dog, enabling commands such as movement directions.

---

## Prerequisites

Before you begin, ensure you have the following installed on your system:

- **Docker**: [Install Docker](https://docs.docker.com/get-docker/)
- **Visual Studio Code (VSCode)**: [Download VSCode](https://code.visualstudio.com/)
- **VSCode Dev Containers Extension**: [Install Extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)
- **Git**: [Install Git](https://git-scm.com/downloads)
- **ROS 2**: This guide uses ROS 2 Humble. Refer to the [ROS 2 Installation Guide](https://docs.ros.org/en/humble/Installation.html) for details.

---

## Building the Docker Image

Docker is used to create an isolated environment for your ROS 2 robot dog project. Follow the steps below to build and run the Docker image.

### Build the Image

1. **Navigate to the Project Directory**

   Open your terminal and navigate to the root directory of the project where the `Dockerfile` is located.

   ```bash
   cd /path/to/ros2_robot_dog_project
   ```

2. **Build the Docker Image**

   Use the following command to build the Docker image. Replace `{Your User Name}` with your actual Docker Hub username.

   ```bash
   docker build -t {Your User Name}/ros2_robot_dog:latest .
   ```

   **Example:**

   ```bash
   docker build -t john_doe/ros2_robot_dog:latest .
   ```

   This command builds the Docker image and tags it as `ros2_robot_dog:latest` under your Docker Hub username.

### Run the Container

Once the image is built, you can run a container from it.

```bash
docker run -it --name ros2_robot_dog --privileged -p 5000:5000 {Your User Name}/ros2_robot_dog:latest
```

**Parameters Explained:**

- `-it`: Runs the container in interactive mode with a terminal.
- `--name ros2_robot_dog`: Names the container `ros2_robot_dog` for easy reference.
- `--privileged`: Grants extended privileges to the container.
- `-p 5000:5000`: Maps port `5000` of the container to port `5000` on the host machine, allowing access to the Flask server.
- `{Your User Name}/ros2_robot_dog:latest`: Specifies the image to run.

**Example:**

```bash
docker run -it --name ros2_robot_dog --privileged -p 5000:5000 john_doe/ros2_robot_dog:latest
```

---

## Attaching to a Running Container

To develop inside the Docker container using VSCode, follow these steps.

### Using VSCode Dev Container Extension

1. **Install the Dev Containers Extension**

   Ensure you have the **Dev Containers** extension installed in VSCode. You can install it from the [VSCode Marketplace](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers).

2. **Attach to the Running Container**

   - Open VSCode.
   - Press `Ctrl + Shift + P` to open the command palette.
   - Type `Attach to Running Container` and select it.
   - Choose the `ros2_robot_dog` container from the list.

### Initializing the Environment Inside the Container

After attaching to the container, you need to initialize the ROS 2 environment.

```bash
source /opt/venv/bin/activate

source /opt/ros/${ROS_DISTRO}/setup.bash
source /ros2_ws/install/setup.bash

export PYTHONPATH="/opt/venv/lib/python3.10/site-packages:$PYTHONPATH"
```

**Explanation:**

- `source /opt/venv/bin/activate`: Activates the Python virtual environment.
- `source /opt/ros/${ROS_DISTRO}/setup.bash`: Sources the ROS 2 setup script. Replace `${ROS_DISTRO}` with your ROS 2 distribution (e.g., `humble`).
- `source /ros2_ws/install/setup.bash`: Sources the workspace setup script.
- `export PYTHONPATH=...`: Updates the `PYTHONPATH` to include the virtual environment's site-packages.

---

## Testing ROS 2 Commands

Ensure that ROS 2 is correctly set up and accessible within the Docker container.

1. **Test ROS 2 Help Command**

   ```bash
   ros2 --help
   ```

2. **Expected Output**

   ```bash
   root@f8c1e78dddd7:/ros2_ws# ros2 --help
   usage: ros2 [-h] [--use-python-default-buffering] Call `ros2 <command> -h` for more detailed usage. ...

   ros2 is an extensible command-line tool for ROS 2.

   options:
     -h, --help            show this help message and exit
     --use-python-default-buffering
                           Do not force line buffering in stdout and instead use the python default buffering, which might be affected by
                           PYTHONUNBUFFERED/-u and depends on whatever stdout is interactive or not

   Commands:
     action     Various action related sub-commands
     bag        Various rosbag related sub-commands
     component  Various component related sub-commands
     daemon     Various daemon related sub-commands
     doctor     Check ROS setup and other potential issues
     interface  Show information about ROS interfaces
     launch     Run a launch file
     lifecycle  Various lifecycle related sub-commands
     multicast  Various multicast related sub-commands
     node       Various node related sub-commands
     param      Various param related sub-commands
     pkg        Various package related sub-commands
     run        Run a package specific executable
     security   Various security related sub-commands
     service    Various service related sub-commands
     topic      Various topic related sub-commands
     wtf        Use `wtf` as alias to `doctor`

     Call `ros2 <command> -h` for more detailed usage.
   ```

---

## Creating a New ROS 2 Node

Extend the functionality of your robot dog by creating new ROS 2 nodes. Follow the steps below to create, build, and run a new node.

### Create a New Package

1. **Navigate to the Source Directory**

   ```bash
   cd /ros2_ws/src
   ```

2. **Create the Package**

   Use the `ros2 pkg create` command to generate a new ROS 2 package.

   ```bash
   ros2 pkg create --build-type ament_python --node-name my_new_node my_package
   ```

   **Parameters Explained:**

   - `--build-type ament_python`: Specifies that the package uses Python and the `ament_python` build system.
   - `--node-name my_new_node`: Names the initial node `my_new_node`.
   - `my_package`: The name of the new package.

3. **Package Structure**

   After creation, your package should have the following structure:

   ```
   my_package/
   ├── package.xml
   ├── setup.py
   ├── resource
   │   └── my_package
   └── my_package
       ├── __init__.py
       └── my_new_node.py
   ```

### Add Node Script

1. **Edit `my_new_node.py`**

   Open the `my_new_node.py` file and add the following content to create a simple publisher node.

   ```python
   # my_package/my_new_node.py

   import rclpy
   from rclpy.node import Node
   from std_msgs.msg import String

   class MyNewNode(Node):
       def __init__(self):
           super().__init__('my_new_node')
           self.publisher_ = self.create_publisher(String, 'chatter', 10)
           timer_period = 1.0  # 1 second
           self.timer = self.create_timer(timer_period, self.timer_callback)
           self.i = 0
           self.get_logger().info('MyNewNode start！')

       def timer_callback(self):
           msg = String()
           msg.data = f'Hello, ROS 2! {self.i}'
           self.publisher_.publish(msg)
           self.get_logger().info(f'Publish: "{msg.data}"')
           self.i += 1

   def main(args=None):
       rclpy.init(args=args)
       node = MyNewNode()
       try:
           rclpy.spin(node)
       except KeyboardInterrupt:
           pass
       finally:
           node.destroy_node()
           if rclpy.ok():
               rclpy.shutdown()

   if __name__ == '__main__':
       main()
   ```

   **Explanation:**

   - **Publisher Node**: The node publishes a `String` message to the `chatter` topic every second.
   - **Logging**: Logs are printed to indicate the node's activity.

### Configure `setup.py`

1. **Open `setup.py`**

   Edit the `setup.py` file to include necessary dependencies and entry points.

   ```python
   # my_package/setup.py

    from setuptools import find_packages, setup

    package_name = 'my_package'

    setup(
        name=package_name,
        version='0.0.0',
        packages=find_packages(exclude=['test']),
        data_files=[
            ('share/ament_index/resource_index/packages',
                ['resource/' + package_name]),
            ('share/' + package_name, ['package.xml']),
        ],
        install_requires=['setuptools'],
        zip_safe=True,
        maintainer='root',
        maintainer_email='root@todo.todo',
        description='TODO: Package description',
        license='TODO: License declaration',
        tests_require=['pytest'],
        entry_points={
            'console_scripts': [
                'my_new_node = my_package.my_new_node:main'
            ],
        },
    )

   ```

   **Key Sections:**

   - **`install_requires`**: Lists Python dependencies. Initially, only `setuptools` is required.
   - **`entry_points`**: Defines the executable script for the node.

2. **Ensure `package.xml` is Correct**

   Although not explicitly modified here, ensure that your `package.xml` includes necessary dependencies, especially if you add more in the future.

### Build the Workspace

1. **Navigate to the Workspace Root**

   ```bash
   cd /ros2_ws
   ```

2. **Install Dependencies**

   Use `rosdep` to install any system dependencies required by your packages.

   ```bash
   rosdep update
   rosdep install --from-paths src --ignore-src -r -y
   ```

3. **Build the Workspace**

   Compile your ROS 2 workspace using `colcon`.

   ```bash
   colcon build
   ```

4. **Source the Setup Script**

   After building, source the setup script to overlay this workspace on top of your environment.

   ```bash
   source install/setup.bash
   ```

### Run the New Node

Execute your newly created ROS 2 node to verify it's working correctly.

```bash
ros2 run my_package my_new_node
```

**Expected Output:**

```bash
[INFO] [my_new_node]: MyNewNode start！
[INFO] [my_new_node]: Publish: "Hello, ROS 2! 0"
[INFO] [my_new_node]: Publish: "Hello, ROS 2! 1"
...
```

---

## Example 

### Run
```bash
ros2 launch robot_dog_launch robot_dog_launch.py
```

1. **Starting the Container and Nodes**

   ```bash
   [INFO] [launch]: All log files can be found below /root/.ros/log/2024-09-22-13-23-19-995206-7dc5ad1b7137-26325
   [INFO] [launch]: Default logging verbosity is set to INFO
   [INFO] [camera-1]: process started with pid [28964]
   [INFO] [machine_dog_control-2]: process started with pid [28966]
   [INFO] [simulator-3]: process started with pid [28968]
   [machine_dog_control-2] [INFO] [1727011642.962666386] [machine_dog_controller]: DogController node has been started.
   [machine_dog_control-2] [INFO] [1727011642.968206613] [machine_dog_controller]: Flask server is running.
   ```

2. **Stopping the Nodes**

   Press `CTRL+C` to terminate the nodes gracefully.

   ```bash
   ^C[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)
   [machine_dog_control-2] [INFO] [1727011644.843123384] [machine_dog_controller]: Shutting down Flask server...
   [machine_dog_control-2] [INFO] [1727011644.972764390] [machine_dog_controller]: Flask server shut down.
   [INFO] [simulator-3]: process has finished cleanly [pid 28968]
   [INFO] [camera-1]: process has finished cleanly [pid 28964]
   [INFO] [machine_dog_control-2]: process has finished cleanly [pid 28966]
   ```

   **Note:** You might encounter warnings like `Failed to publish log message to rosout: publisher's context is invalid`. These can typically be ignored as they occur during the shutdown sequence.

---

## Troubleshooting

### Common Issues

1. **`ros2: command not found`**

   **Cause:** The ROS 2 environment is not properly sourced.

   **Solution:**

   ```bash
   source /opt/ros/${ROS_DISTRO}/setup.bash
   source /ros2_ws/install/setup.bash
   export PYTHONPATH="/opt/venv/lib/python3.10/site-packages:$PYTHONPATH"
   ```

   Replace `${ROS_DISTRO}` with your ROS 2 distribution (e.g., `humble`).

2. **`ModuleNotFoundError: No module named 'flask'`**

   **Cause:** Flask is not installed in the package's Python environment.

   **Solution:**

   Ensure `Flask` is listed in `setup.py` under `install_requires` and rebuild the workspace.

   ```bash
   export PYTHONPATH="/opt/venv/lib/python3.10/site-packages:$PYTHONPATH"
   ```

   Then rebuild:

   ```bash
   cd ~/ros2_ws
   rm -rf build/ install/ log/
   colcon build
   source install/setup.bash
   ```

3. **Flask Server Not Accessible**

   **Cause:** Port `5000` might not be exposed or mapped correctly.

   **Solution:**

   Ensure the Docker container is run with port `5000` exposed.

   ```bash
   docker run -it --name ros2_robot_dog --privileged -p 5000:5000 {Your User Name}/ros2_robot_dog:latest
   ```

4. **Flask Server Crashing on Shutdown**

   **Cause:** Improper handling of thread shutdown.

   **Solution:**

   Ensure the Flask server is gracefully shut down by managing its lifecycle in the node's `destroy_node` method, as demonstrated in the node code above.

### Additional Tips

- **Check Logs:** Review the logs located in `/root/.ros/log/` for detailed error messages.
- **Validate Dependencies:** Ensure all Python dependencies are installed in the virtual environment and declared in `setup.py`.
- **Use Virtual Environments:** Always activate the Python virtual environment before building or running ROS 2 nodes.

## License

This project is licensed under the [Apache License 2.0](LICENSE).


## Acknowledgements

- [ROS 2](https://index.ros.org/doc/ros2/)
- [Docker](https://www.docker.com/)
- [Flask](https://flask.palletsprojects.com/)
- [Visual Studio Code](https://code.visualstudio.com/)

---

*Happy Coding!*