CHange microstrain_inertial_driver/microstrain_launcher.launch CODE

1. <arg name="node_instance" default="1" doc="Instance number to differentiate multiple nodes" />

2. <node name="$(arg node_name)_$(arg node_instance)" pkg="microstrain_inertial_driver" type="microstrain_inertial_driver_node" output="screen" ns="$(arg namespace)">
