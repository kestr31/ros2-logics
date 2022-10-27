#!/bin/bash

parent_path=$( cd "$(dirname "${BASH_SOURCE[0]}")" ; pwd -P )

sed -i "s/map_queue = 'a4vai\/map_queue'\n//g" ${parent_path}/a4vai/setup.py 
sed -i "s/package_name, controller, path_following, path_planning, collision_avoidance, map_queue/package_name, controller, path_following, path_planning, collision_avoidance/g" ${parent_path}/a4vai/setup.py
sed -i "s/'map_queue = a4vai.map_queue.map_queue:main',\n//g" ${parent_path}/a4vai/setup.py

sed -i "s/from .map_service import MapService/# from .map_service import MapService/g" ${parent_path}/a4vai/a4vai/controller/controller.py
sed -i "s/from .map_service import MapService/# from .map_service import MapService/g" ${parent_path}/a4vai/a4vai/controller/controller.py

tgtStart=($(grep -nr 'if self.map_generation_flag is True :' ${parent_path}/a4vai/a4vai/controller/controller.py | cut -f1 -d:))
tgtEnd=($(grep -nr 'map_service.destroy_node()' ${parent_path}/a4vai/a4vai/controller/controller.py | cut -f1 -d:))

echo ${tgtStart}
echo $((${tgtEnd} + 2))

sed -i "${tgtStart},$((${tgtEnd} + 2))d" ${parent_path}/a4vai/a4vai/controller/controller.py