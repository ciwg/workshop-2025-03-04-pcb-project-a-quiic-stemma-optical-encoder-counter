#!/bin/bash

go run main.go &
pid=$!

trap "kill $pid; exit 0" EXIT INT TERM HUP 

while true
do inotifywait -r -e modify *
    padsp signalgen -t 100m sin 444
    sleep 1
    grok-commit -A
    git push
    if [ $? -ne 0 ]
    then
        padsp signalgen -t 100m sin 300
        continue
    fi
    padsp signalgen -t 100m sin 666
    echo -------------------------------------------------------------
done

