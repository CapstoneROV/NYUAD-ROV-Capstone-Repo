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

build_if_not_exists:
	if [ -z "$$(sudo docker images -q $(IMAGE_NAME))" ]; then \
	  echo "Image $(IMAGE_NAME) not found, building."; \
	  make build; \
	else \
	  echo "Image $(IMAGE_NAME) found, skipping build."; \
	fi
# Copy files into docker container/do not remove any new files in container
copy:
	docker cp -a . $(CONTAINER_NAME):/home/$(USER)/$(CONTAINER_NAME)

# Run docker container with user ardupilot
run_container:
	sudo docker run -it --user $(USER) --privileged --cap-add SYS_ADMIN --device /dev/fuse \
	--env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name $(CONTAINER_NAME) \
	-w /home/$(USER)/$(CONTAINER_NAME) $(IMAGE_NAME)

# Run docker container
# Init submodules
# Delete container from scratch and run new container
# Build if does not exist
rerun:
	make init_submodule && \
	sudo docker ps -aq --filter "name=^/$(CONTAINER_NAME)$\" | \
	xargs -r sudo docker rm && \
	make build_if_not_exists && \
	make run_container 

# Run docker container
# If container already exists, start the container (DONT DELETE CONTAINER)
run:
	make init_submodule && \
	make build_if_not_exists && \
	sudo docker ps -aq --filter "name=^/$(CONTAINER_NAME)$\" | \
	xargs -r sudo docker start -i $(CONTAINER_NAME) && \
	echo "Container $(CONTAINER_NAME) found, starting container." || \
	make copy && \
	docker start -i $(CONTAINER_NAME) || \
	make run_container

# Stop docker container
stop:
	sudo docker stop $(CONTAINER_NAME)

# Make catkin workspace add source devel/setup.bash to bashrc
catkin_make:
	sudo docker exec -it $(CONTAINER_NAME) /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	cd /$(CONTAINER_NAME) && \
	catkin_make" && \
	echo "source /$(CONTAINER_NAME)/devel/setup.bash" >> ~/.bashrc


# Launch sim.launch
launch_sim:
	sudo docker exec -it $(CONTAINER_NAME) /bin/bash -c "source /opt/ros/melodic/setup.bash && \
	cd /$(CONTAINER_NAME) && \
	roslaunch capstone sim.launch"
