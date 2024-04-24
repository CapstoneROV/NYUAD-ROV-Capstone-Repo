IMAGE_NAME=ros-ubuntu18.04
CONTAINER_NAME=capstonerov
USER=ardupilot
pwd := $(shell pwd)

init_submodule:
	git submodule update --init --recursive

# Build dockerfile in same directory
.PHONY: build
build:
	docker build -t $(IMAGE_NAME) .

.PHONY: build_if_not_exists
build_if_not_exists:
	if [ -z "$$(sudo docker images -q $(IMAGE_NAME))" ]; then \
	  echo "Image $(IMAGE_NAME) not found, building."; \
	  make build; \
	else \
	  echo "Image $(IMAGE_NAME) found, skipping build."; \
	fi

# Copy files into docker container/do not remove any new files in container
# We use a temp directory to speed this up mutliple factors
# Exclude .git and src folder (this is bind mounted anyways)
.PHONY: copy
copy:
	mkdir -p temp_dir && \
	rsync -avq --exclude='.git' --exclude='src' --exclude='build' --exclude='devel' --exclude='install' . temp_dir && \
	docker cp -a temp_dir/. $(CONTAINER_NAME):/home/$(USER)/$(CONTAINER_NAME) && \
	rm -rf temp_dir

# Run docker container with user ardupilot
# Mount X11 socket for GUI
# Bind mount src folder to container
.PHONY: run_container
run_container:
	sudo docker run -it --gpus all --user $(USER) --network host --privileged --cap-add SYS_ADMIN --device /dev/fuse \
	--env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name $(CONTAINER_NAME) \
	--volume="$(pwd)/src:/home/$(USER)/$(CONTAINER_NAME)/src" \
	--volume="$(pwd)/include/ardupilot_gazebo/src:/home/$(USER)/$(CONTAINER_NAME)/include/ardupilot_gazebo/src" \
	-w /home/$(USER)/$(CONTAINER_NAME) $(IMAGE_NAME)

# Reuse created container
.PHONY: reuse
reuse:
	sudo docker start $(CONTAINER_NAME)
	sudo docker exec -it --ufser $(USER) $(CONTAINER_NAME) /bin/bash
	sudo docker stop $(CONTAINER_NAME)

# Run docker container
# Init submodules
# Delete container from scratch and run new container
# Build if does not exist
.PHONY: rerun
rerun:
	make init_submodule && \
	sudo docker ps -aq --filter "name=^/$(CONTAINER_NAME)$\" | \
	xargs -r sudo docker rm && \
	make build_if_not_exists && \
	make run_container 

# Run docker container
# If container already exists, start the container (DONT DELETE CONTAINER)
.PHONY: run
run:
	make init_submodule && \
	make build_if_not_exists && \
	(make copy || rm -rf temp_dir) && \
	make run_container || \
	echo "Container $(CONTAINER_NAME) found, starting container." && \
	sudo docker start -i $(CONTAINER_NAME)

# Stop docker container
.PHONY: stop
stop:
	sudo docker stop $(CONTAINER_NAME)

# Make catkin workspace add source devel/setup.bash to bashrc
.PHONY: catkin_make
catkin_make:
	sudo docker exec -it $(CONTAINER_NAME) /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	cd /$(CONTAINER_NAME) && \
	catkin_make" && \
	echo "source /$(CONTAINER_NAME)/devel/setup.bash" >> ~/.bashrc


# DOCKER MAKEFILE COMMANDS
# ----------
# SIMULATION
# ----------
.PHONY: launch_sim
launch_sim:
	roslaunch $(pwd)/launch/sim.launch

.PHONY: launch_sitl
launch_sitl:
	sim_vehicle.py -j6 -v ArduSub -f gazebo-bluerov2 -l 55.60304,12.808937,0,0 --console \
	--out=udpout:0.0.0.0:14550 --out=udpout:0.0.0.0:14551 --out=udpout:0.0.0.0:14552 --out=udpout:0.0.0.0:14553 \
	--out=udpout:0.0.0.0:14554 --out=udpout:0.0.0.0:14555 --add-param-file=$(pwd)/src/bluerov2_sim/bluerov2_ardusub/config/vectored6dof.param
# -------
# TOPSIDE
# -------
.PHONY: mav_proxy 
mav_proxy:
	mavproxy.py --master=udpout:192.168.2.2:14550 \
	--out 127.0.0.1:14540 --out 127.0.0.1:14550 \
	--out 127.0.0.1:14551 --out 127.0.0.1:14552

.PHONY: mavros
mavros:
	roslaunch mavros apm.launch fcu_url:=udp://0.0.0.0:14550@