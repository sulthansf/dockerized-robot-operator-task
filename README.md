Your task is to create a containerized system where on start it launches two ROS nodes. Each ROS node should publish a string message to a topic once a second and the other ROS node should subscribe to that topic and print the string message it receives from its subscription. In the end they should both be publishing and subscribing to each other over two ROS topics. Once you have set up this system, think about and implement something creative that the nodes could do. There's no right answer here!

The container should be running ROS Noetic. The preferred language is Python. We recommend you use Docker for containerization, but if there is another tool you prefer for containerization, then feel free to use that.

Please prepare a README that makes it easy to understand what you have setup and how to run the system. Go ahead and replace these instructions with your README.

Some starting materials if you are not familiar with ROS or Docker:
* [ROS in Python example](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29)
* [Docker intro](https://docker-curriculum.com/)
* [OSRF's Dockerhub](https://hub.docker.com/r/osrf/ros/tags)
