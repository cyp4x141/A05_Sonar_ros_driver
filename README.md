#A05 sonar driver

## A05 sonar
A05 is 1*4 sonar package which have 4 sonar sensors

## build
```
$ rosdep install -y --from-paths {{ WORKSPACE_ROOT }}/src --ignore-src --rosdistro {{ rosdistro }}
$ colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## run
```
$ sudo chmod 777 /dev/${sonar-serial-port}
$ source install/setup.bash
$ ros2 launch a05_driver a05-sonar.launch.xml  
```
`$ ros2 run tf2_ros static_transform_publisher "0" "0" "0" "0" "0" "0" "map" "sonar_01"` or use urdf to send TF  
