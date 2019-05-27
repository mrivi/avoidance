#!/bin/bash
cat > launch/landing_site_detection_launch.launch <<- EOM
<launch>
    <arg name="ns" default="/"/>
    <arg name="fcu_url" default="udp://:14540@localhost:14557"/>
    <arg name="gcs_url" default="" />   <!-- GCS link is provided by SITL -->
    <arg name="tgt_system" default="1" />
    <arg name="tgt_component" default="1" />

    <!-- Launch MavROS -->
    <group ns="\$(arg ns)">
        <include file="\$(find mavros)/launch/node.launch">
            <arg name="pluginlists_yaml" value="\$(find mavros)/launch/px4_pluginlists.yaml" />
            <!-- Need to change the config file to get the tf topic and get local position in terms of local origin -->
            <arg name="config_yaml" value="\$(find local_planner)/resource/px4_config.yaml" />
            <arg name="fcu_url" value="\$(arg fcu_url)" />
            <arg name="gcs_url" value="\$(arg gcs_url)" />
            <arg name="tgt_system" value="\$(arg tgt_system)" />
            <arg name="tgt_component" value="\$(arg tgt_component)" />
        </include>
    </group>

    <!-- Launch cameras -->
EOM

# Fix the on/of script for realsense auto-exposure
cat > resource/realsense_params.sh <<- EOM
#!/bin/bash
# Disable and enable auto-exposure for all cameras as it does not work at startup
EOM

# Set the frame rate to 15 if it is undefined
if [ -z $DEPTH_CAMERA_FRAME_RATE ]; then
  DEPTH_CAMERA_FRAME_RATE=15
fi

# The CAMERA_CONFIGS string has semi-colon separated camera configurations
IFS=";"
for camera in $CAMERA_CONFIGS; do
	IFS="," #Inside each camera configuration, the parameters are comma-separated
	set $camera
	if [[ $# != 8 ]]; then
		echo "Invalid camera configuration $camera"
	else
		echo "Adding camera $1 with serial number $2"
		if [[ $camera_topics == "" ]]; then
			camera_topics="/$1/depth/points"
		else
			camera_topics="$camera_topics,/$1/depth/points"
		fi

    # Append to the launch file
    cat >> launch/landing_site_detection_launch.launch <<- EOM
			<node pkg="tf" type="static_transform_publisher" name="tf_$1"
			 args="$3 $4 $5 $6 $7 $8 fcu $1_link 10"/>
			<include file="\$(find local_planner)/launch/rs_depthcloud.launch">
				<arg name="namespace"             value="$1" />
				<arg name="tf_prefix"             value="$1" />
				<arg name="serial_no"             value="$2"/>
				<arg name="depth_fps"             value="$DEPTH_CAMERA_FRAME_RATE"/>
				<arg name="enable_pointcloud"     value="false"/>
				<arg name="enable_fisheye"        value="false"/>
			</include>
		EOM

    # Append to the realsense auto exposure toggling
    echo "rosrun dynamic_reconfigure dynparam set /$1/stereo_module enable_auto_exposure 0
rosrun dynamic_reconfigure dynparam set /$1/stereo_module enable_auto_exposure 1
rosrun dynamic_reconfigure dynparam set /$1/stereo_module visual_preset 4
" >> resource/realsense_params.sh

	fi
done

if [ ! -z $VEHICLE_CONFIG ]; then
cat >> launch/landing_site_detection_launch.launch <<- EOM
  <node name="dynparam" pkg="dynamic_reconfigure" type="dynparam" args="load local_planner_node \$(find local_planner)/cfg/$VEHICLE_CONFIG.yaml" />
EOM
echo "Adding vehicle paramters: $VEHICLE_CONFIG"
fi

cat >> launch/landing_site_detection_launch.launch <<- EOM
    <!-- Launch avoidance -->
    <arg name="pointcloud_topics" default="$camera_topics"/>

    <node name="landing_site_detection_node" pkg="landing_site_detection" type="landing_site_detection_node" output="screen" >
      <param name="pointcloud_topics" value="\$(arg pointcloud_topics)" />
    </node>

    <!-- switch off and on auto exposure of Realsense cameras, as it does not work on startup -->
    <node name="set_RS_param" pkg="landing_site_detection" type="realsense_params.sh" />

</launch>
EOM

# Set the frame rate in the JSON file as well
sed -i '/stream-fps/c\    \"stream-fps\": \"'$DEPTH_CAMERA_FRAME_RATE'\",' resource/stereo_calib.json