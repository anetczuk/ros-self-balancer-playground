#!/bin/bash

set -eu


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"


SRC_DIR=$SCRIPT_DIR/src


mkdir -p $SRC_DIR

catkin_init_workspace $SRC_DIR
