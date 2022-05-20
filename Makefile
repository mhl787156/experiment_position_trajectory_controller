MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/buildtools/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

NETWORK?=bridge
ENV?=
BUILD_ARGS?=


all: position_trajectory_controller

position_trajectory_controller:
	$(BAKE) position-trajectory-controller

# This mybuilder needs the following lines to be run:
# docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
# docker buildx create --name mybuilder
# docker buildx use mybuilder
# docker buildx inspect --bootstrap
local-build-push:
	docker buildx bake --builder mybuilder -f $(BAKE_SCRIPT) --push 4pl-controller

run: position_trajectory_controller
	docker run -it --rm --net=$(NETWORK) $(ENV) -e USE_SIMULATED_TIME=true uobflightlabstarling/position-trajectory-controller:latest

run_bash: position_trajectory_controller
	docker run -it --rm --net=$(NETWORK) -e USE_SIMULATED_TIME=true uobflightlabstarling/position-trajectory-controller:latest bash

.PHONY: all position_trajectory_controller run run_bash