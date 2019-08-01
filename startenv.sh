#!/bin/bash

## stops script on error or unbound variable (undefined/unset)
set -eu

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"


tmpfile=$(mktemp start.catkin.ws.XXXXXX.sh --tmpdir)


cat > $tmpfile <<EOL
## set bash prompt prefix
PS1="(ROS) \$PS1"
export PS1
source $SCRIPT_DIR/devel/setup.bash
if [ \$? -ne 0 ]; then
    echo -e "Unable to activate virtual environment, exiting"
    exit 1
fi 
echo "ros package paths:" \$ROS_PACKAGE_PATH
exec </dev/tty 
EOL


echo "Starting virtual env"

bash -i <<< "source $tmpfile"
