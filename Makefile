# Define image and container names
IMAGE_NAME = ros-image
CONTAINER_NAME = ros-container

# Build the Docker image
build:
	docker build -t $(IMAGE_NAME) .

# Run the Docker container
run:
	docker run \
		--name $(CONTAINER_NAME) \
		--hostname $(CONTAINER_NAME) \
	    -it $(IMAGE_NAME)

# Execute a command inside the running container
exec:
	docker exec -it $(CONTAINER_NAME) bash

# Stop the running container
stop:
	docker stop $(CONTAINER_NAME)

# Start the stopped container
start:
	docker start $(CONTAINER_NAME)

# Rebuild the Docker image and recreate the container
rebuild: stop build run

# Stop and remove the container and image
clean:
	-docker stop $(CONTAINER_NAME)
	-docker rm $(CONTAINER_NAME)
	-docker rmi $(IMAGE_NAME)

# Default target
all: build run