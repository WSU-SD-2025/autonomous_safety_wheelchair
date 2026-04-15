#!/bin/bash

# Usage: ./enter_container.sh <container_id>
# If no container ID is provided, the default id is "efb5343782ad"
# You can find the container ID by running `docker ps -a`
# The container must be started before running this script.

$CONTAINER_ID=${1:-"efb5343782ad"}

docker exec -it $CONTAINER_ID /bin/bash