## Robotica Movil IIC2685


### GPU acceleration:

**HOST OS**:
1. `distribution=$(. /etc/os-release;echo $ID$VERSION_ID)`
2. `curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -`
3. `curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list`
4. `sudo apt-get update && sudo apt-get install -y nvidia-container-toolkit`
5. `sudo systemctl restart docker`
### TODO:

1. control 1 (chat pub/sub)
