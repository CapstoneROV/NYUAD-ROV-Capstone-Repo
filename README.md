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

```
make run
```

To run roscore as background, run

```
roscore &
```

Then launch anything you want, such as 

```
rviz
```


# Using GUI

To run GUI applications in docker container (e.g. gazebo, rviz), you need to configure the container to use the X server of your host machine.

```
xhost +local:docker
```

## Notes for Pi

After building image, you need to tag then push to dockerhub
```
pi@swarmic:~/Documents/GitHub/CapstoneRoV$ sudo docker tag ros-ubuntu18.04 piko314159/ros-ubuntu18.04
pi@swarmic:~/Documents/GitHub/CapstoneRoV$ sudo docker push piko314159/ros-ubuntu18.04
```

## Contributing
Contributions are welcome. Please adhere to this project's code of conduct.

## License
This project is licensed under the MIT License - see the LICENSE.md file for details.

## Contact
For any queries or suggestions, please reach out to Pi Ko at pk2269@nyu.edu.
