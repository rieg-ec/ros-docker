# ROS-Docker-dev

### Steps to run:

1. install XQuartz
2. setup xquartz (follow first steps [here](https://sourabhbajaj.com/blog/2017/02/07/gui-applications-docker-mac/) and stop in the _Run XQuartz_ section
3. clone source code of packages into `src/`
4. add exec permission to launch script: `chmod +x launch.sh`
5. run launch script: `./launch.sh`
6. `docker exec -it <container id> /bin/bash` -container id can be found by running `docker ps`-
