#!/bin/bash


## works both under bash and sh
SCRIPT_DIR=$(dirname "$(readlink -f "$0")")



$SCRIPT_DIR/notrailingwhitespaces.sh

exit_code=$?
if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi


$SCRIPT_DIR/codecheck.sh

exit_code=$?
if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi


$SCRIPT_DIR/doccheck.sh

exit_code=$?
if [ $exit_code -ne 0 ]; then
    exit $exit_code
fi
