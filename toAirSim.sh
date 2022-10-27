#!/bin/bash

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )
listArray=($(grep -nri 'map' ${parent_path}/a4vai/setup.py | cut -f1 -d:))

for i in ${listArray[@]}
do
    strVal=$(sed -n -e ${i}p ${parent_path}/a4vai/setup.py)
    if [[ ${strVal} == *"packages="* ]];
    then
        strRepVal=${strVal//, map_queue/}
        echo ${strRepVal}
        sed -i "s/${strVal}/${strRepVal}/g" ${parent_path}/a4vai/setup.py
    else
        sed "${i}" ${parent_path}/a4vai/setup.py
    fi
done


sed -i "s/from .map_service import MapService/# from .map_service import MapService/g" ${parent_path}/a4vai/a4vai/controller/controller.py
sed -i "s/from .map_service import MapService/# from .map_service import MapService/g" ${parent_path}/a4vai/a4vai/controller/controller.py

tgtStart=($(grep -nr 'if self.map_generation_flag is True :' ${parent_path}/a4vai/a4vai/controller/controller.py | cut -f1 -d:))
tgtEnd=($(grep -nr 'map_service.destroy_node()' ${parent_path}/a4vai/a4vai/controller/controller.py | cut -f1 -d:))

sed "${tgtStart}:$(${tgtEnd} + 2)d" ${parent_path}/a4vai/a4vai/controller/controller.py