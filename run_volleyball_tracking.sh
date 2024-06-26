#!/bin/bash

#gnome-terminal -- /bin/bash -c "cd /home/toenuc8/volleyball/build; exec bash"
cd /home/toenuc8/volleyball/build
while true; 
do
    ./Volleyball_Vision
    if [[ "$?" -ne 0 ]]; then
        echo "启动失败， $?. 再次尝试..." >&2
    else
        break
    fi
done

