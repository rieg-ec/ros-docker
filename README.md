# ROS-Docker-dev

### Steps to run:

1. `xhost +local:docker` in terminal
2. clone source code of packages into `src/`
3. `docker-compose up`
4. `docker exec -it <container id> /bin/bash` -container id can be found by running `docker ps`-
