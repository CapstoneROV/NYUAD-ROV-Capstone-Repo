IMAGE_NAME=ros-ubuntu18.04
CONTAINER_NAME=capstonerov
pwd := $(shell pwd)

init_submodule:
	git submodule update --init --recursive

# Build dockerfile in same directory
build:
	docker build -t $(IMAGE_NAME) .

build_if_not_exists:
	if [ -z "$$(sudo docker images -q $(IMAGE_NAME))" ]; then \
	  echo "Image $(IMAGE_NAME) not found, building."; \
	  make build; \
	else \
	  echo "Image $(IMAGE_NAME) found, skipping build."; \
	fi

run_container:
	sudo docker run -it --env="DISPLAY" --env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" --name $(CONTAINER_NAME) \
	-v $(pwd):/$(CONTAINER_NAME) $(IMAGE_NAME)

# Run docker container
# Init submodules
# Delete container from scratch and run new container
# Build if does not exist
run:
	make init_submodule && \
	sudo docker ps -aq --filter "name=^/$(CONTAINER_NAME)$\" | \
	xargs -r sudo docker rm && \
	make build_if_not_exists && \
	make run_container 

# Run docker container
# If container already exists, start the container (DONT DELETE CONTAINER)
rerun:
	sudo docker ps -aq --filter "name=^/$(CONTAINER_NAME)$\" | \
	xargs -r sudo docker start -i $(CONTAINER_NAME) && \
	echo "Container $(CONTAINER_NAME) found, starting container." || \
	sudo docker start -i $(CONTAINER_NAME)

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
