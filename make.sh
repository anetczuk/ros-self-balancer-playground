#!/bin/bash

set -eu


SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" >/dev/null && pwd )"


cd $SCRIPT_DIR


if [[ $* == *"--clear"* ]]; then
    ## remove old build directory
    build_dir=$SCRIPT_DIR/build
    echo "Deleting: $build_dir"
    rm -rf $build_dir
fi


### calling "catkin_make" is similar to following commands:
###     mkdir $SCRIPT_DIR/build
###     cd $SCRIPT_DIR/build
###     cmake $SCRIPT_DIR/src

catkin_make
