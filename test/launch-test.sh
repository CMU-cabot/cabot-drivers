#!/bin/bash


trap ctrl_c INT QUIT TERM

function ctrl_c() {
    docker compose down > /dev/null 2>&1
    exit
}
function red {
    echo -en "\033[31m"  ## red
    echo $@
    echo -en "\033[0m"  ## reset color
}

pwd=`pwd`
scriptdir=`dirname $0`
cd $scriptdir
scriptdir=`pwd`
cd $scriptdir/../

if [[ ! -z $1 ]]; then
    models=$1
else
    pushd cabot_base/config > /dev/null 2>&1
    models=$(ls cabot*.yaml | grep -v common | cut -d. -f1)
    popd > /dev/null 2>&1
fi

for model in ${models[@]}; do
    echo -n "$model: "
    export CABOT_MODEL=$model
    export CABOT_TOUCH_PARAMS=[128,64,32] 
    timeout 10s docker compose up driver > /tmp/cabot-driver-test.log 2>&1
    EXIT_STATUS=$?

    if [ $EXIT_STATUS -eq 124 ]; then
	echo "Looks okay, shutdown the docker"
	docker compose down > /dev/null 2>&1
    else
	echo "Something is wrong: $EXIT_STATUS"
	red $(grep ERROR /tmp/cabot-driver-test.log)
    fi
done
