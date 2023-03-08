# Using process similar to MFI_AMR WORKSPACE but building with ROS2 Humble Instead

## Setup

1. Configure `nvidia-container-runtime` as the default runtime for Docker.

   Using your text editor of choice, add the following items to `/etc/docker/daemon.json`.

    ```json
    {
        ...
        "runtimes": {
            "nvidia": {
                "path": "nvidia-container-runtime",
                "runtimeArgs": []
            }
        },
        "default-runtime": "nvidia"
        ...
    }
    ```

2. Then, restart Docker:

   ```bash
   sudo systemctl daemon-reload && sudo systemctl restart docker
   ```

3. Install [Git LFS](https://git-lfs.github.com/) in order to pull down all large files:  

    ```bash
    sudo apt-get install git-lfs
    ```  

    ```bash
    git lfs install
    ```

4. Navigate to `src` 
   > **Note:** Remember to replace <path-to-mfi_amr_ws> with the correct path in the following command.
    ```bash
    cd <path-to-mfi_amr_ws>/src
    ``` 
5. Install [vcstool](https://github.com/dirk-thomas/vcstool) -
   ```bash 
   curl -s https://packagecloud.io/install/repositories/dirk-thomas/vcstool/script.deb.sh | sudo bash
   sudo apt-get update
   sudo apt-get install python3-vcstool
   ```
5. Clone all required repos using `vcstool`- 
   1. isaac_ros_common
   <SKIP> 2. realsense (4.51.1)
   ```bash 
   vcs import < ros2.repos
   ```
_________________________________________ SKIP ______________________________________________________________

7. Clone the `librealsense` repo setup udev rules. Remove any connected relasense cameras when prompted:
   ```bash
    cd /tmp && \
    git clone https://github.com/IntelRealSense/librealsense && \
    cd librealsense
    ``` 
    ```bash 
    sudo ./scripts/setup_udev_rules.sh
    ```
_______________________________________________________________________________________________________________

9. Configure the container created by `isaac_ros_common/scripts/run_dev.sh` to include `librealsense`. Create the `.isaac_ros_common-config` file in the `isaac_ros_common/scripts` directory:
    > **Note:** Remember to replace the **two** instances of <path-to-mfi_amr_ws> with the correct path in the following command.
    ```bash
    cd <path-to-mfi_amr_ws>/src/isaac_ros_common/scripts && \
    touch .isaac_ros_common-config && \
    echo CONFIG_IMAGE_KEY=humble.nav2 > .isaac_ros_common-config && \
    echo CONFIG_DOCKER_SEARCH_DIRS="(<path-to-mfi_amr_ws>/docker)" >> .isaac_ros_common-config
    ```

10. Create alias to launch the docker container
   > **Note:** Remember to replace the **two** instances of <path-to-mfi_amr_ws> with the correct path.
   
   Edit the `/etc/bash.bashrc` file using `sudo nano /etc/bash.bashrc` and add an entry as follows - .
   ```bash
   alias ros2_humble_docker="cd <path-to-mfi_amr_ws>/src/isaac_ros_common && ./scripts/run_dev.sh <path-to-mfi_amr_ws>"
   ```
   Then, run this command to source the bashrc 
   ```bash
   source /etc/bash.bashrc
   ```

12. Launch the docker container using the `run_dev.sh` script. If you run this command twice then the script will just attach a new shell to the existing container.

   ```bash
   ros2_humble_docker
   ```

13. Build workspace using `colcon build`
   ```bash
   colcon build --packages-skip-regex isaac_ros*
   ```
   Then, run this command -
   ```bash
   source install/setup.bash
   ```
   
   <ADD INSTALLATION FOR NEOBOTIX IF REQUIRED>
   
## Testing Installation
14. Testing Velodyne VLP16 <br>
Run the following command in the container -
   ```bash
   ros2 launch velodyne velodyne-all-nodes-VLP16-launch.py
   ```
Run the following command to check the frequency of the point cloud topic. You should see an average rate of 10 if everything goes well.
   ```bash
   ros2 topic hz /velodyne_points
   ```

## Jetson Xavier AGX SD card Setup
To automount the SD card, an entry with a reference to the device path or UUID of the device has to be added in fstab ( File System Table ) file.

Run this command to identify the attached device and note its UUID.
   ```bash
   sudo blkid
   ```
Edit the fstab file by executing the following command in the terminal. Add an entry similar to the one below after changing the UUID.
   ```bash
   sudo nano /etc/fstab
   ```
   ```
   UUID=<put-UUID-here> /mnt  ext4    defaults    0   0
   ```
# VSCode Setup
### Prerequisites

You should already have Docker and VSCode with the remote containers plugin installed on your system.

* [docker](https://docs.docker.com/engine/install/)
* [vscode](https://code.visualstudio.com/)
* [vscode remote containers plugin](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers)

Steps to connect to container - 
1. Launch the container using `ros2_foxy_docker`
2. Launch VS code and click on `Open a Remote Window` towards the bottom left corner
3. Choose `Attach to a Running Container`
4. Select the appropriate container (`isaac_ros_dev*`)
5. Set working directory to `/workspaces/isaac_ros-dev` in VS Code

# Development Workflow  
## Using open-source ROS2 packages
If you require installing any packages add them to the Dockerfile.
If you require building them then add them to ros2.repos and use `vcstool` to manage them 
## Updating Docker
- Every time we execute the `ros2_foxy_docker`, it will automatically build the Docker image if any modifications are detected in the dockerfile.
- If there is a need to install external dependencies, by adding those to the Dockerfile (docker/Dockerfile.mfi_amr) and pushing the updates to github, we can ensure that these dependencies are accessible to all development environments.
- You can follow the template present in Dockerfile as a starting point to add any dependencies.
## Git workflow 
We can use tree types of branches - master, devel, feat, hotfix as follows ([Reference](https://www.atlassian.com/git/tutorials/comparing-workflows/gitflow-workflow))
- A `devel` branch is created from `master` 
- `feat` branches are created from `devel` 
- When a `feat` is complete it is merged into the `devel` branch 
- After milestone demonstrations,`devel` is merged into `master` 
- If an issue in `master` is detected a `hotfix` branch is created from `master` 
- Once the `hotfix` is complete it is merged to both `devel` and `master`

Good practices related to commits - 
- Include the ROS2 package names in the commit message e.g. "[mapping] some change"
- If possible, include only changes to a single package in a commit
- Break down large changes into smaller commits
