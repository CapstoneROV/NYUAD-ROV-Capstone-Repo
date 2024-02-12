# NYUAD ROV Capstone Repo

![Image of Version](https://img.shields.io/badge/version-latest-blue)
![ROS Ubuntu 18.04 Docker](https://img.shields.io/badge/docker-ROS%20Ubuntu%2018.04-blue)
![ROS Melodic](https://img.shields.io/badge/ROS-Melodic-brightgreen)
![License](https://img.shields.io/badge/license-Open-blue.svg)

This is the docker environment, loaded out of the box with Ubuntu 18.04, ROS Melodic, Python, supporting both versions 2.7 and 3, to ensure compatibility with a wide range of existing tools and scripts, for the NYUAD Capstone Underwater RoV project.

This is intended for a straightforward and practical approach to setting up a development environment for our underwater robot. 

This container supports Python both versions 2.7 and 3, to ensure compatibility with a wide range of existing tools and scripts, with a variety of Python packages like numpy and scipy, essential for data analysis and algorithm development. Additionally, the setup facilitates GUI applications, crucial for visual tools like RViz, gazebo out of the box.

We ensure the ROS environment is always ready by sourcing it in the bashrc file, and we set our working directory to '/capstonerov' in mounted docker container to keep our project organized. 

## Running Docker Container with GUI support (Linux) - working
To build docker image. Please rerun this command when you make any edits to source(code) files and run the command after:
```
make build
```

To run image and create container if necessary. Please note that when making any edits to the launch files on your local file system, you need to run this command again:
```
make run
```

To run simulation: (Run this in docker container)
```
roslaunch bluerov2_bringup bringup_ardusub_sitl_custom.launch gazebo:=true
```
then run in a separate terminal:
```
docker exec -it capstonerov /bin/bash -c "sim_vehicle.py -v ArduSub -l 55.60304,12.808937,0,0 --console"
```

# Using GUI

To run GUI applications in docker container (e.g. gazebo, rviz), you need to configure the container to use the X server of your host machine.

```
xhost +local:docker
```

## Contributing
Contributions are welcome. Please adhere to this project's code of conduct.

## License
This project is licensed under the MIT License - see the LICENSE.md file for details.

## Contact
For any queries or suggestions, please reach out to Rami Richani at rir8190@nyu.edu or Pi Ko at pk2269@nyu.edu.
