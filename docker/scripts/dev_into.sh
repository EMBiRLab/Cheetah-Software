EMBIR_ROOT_DIR="$(pwd)/../.."

xhost +local:docker

docker run -v ${EMBIR_ROOT_DIR}:/Quadruped-Software \
           --privileged \
           -w /Quadruped-Software \
           --env="DISPLAY" \
           --net host \
           -it embir-quad 
           
           