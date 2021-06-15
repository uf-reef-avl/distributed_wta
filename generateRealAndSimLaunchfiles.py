#!/usr/bin/env python

import sys, getopt
import random

def generate_real_turtlebot_launch_file(id,target_number, weapon_number):
    f = open("launch/real_agent_"+str(id)+".launch", "w")
    file_str = r"""<launch> 
    <arg name="turtlebot_name"  default="agent_"""+str(id)+r""""/>
    <arg name="node_start_delay" default="0.0" />

    <arg name="num_weapons" default="""+"\""+str(weapon_number)+"\""+r"""/>
    <arg name="num_targets" default="""+"\""+str(target_number)+"\""+r"""/>
    <arg name="run_gazebo" default="true"/>
    <arg name="model" default="burger"/>


  <node pkg="ros_vrpn_client" name="$(arg turtlebot_name)" type="ros_vrpn_client" args="_vrpn_server_ip:=192.168.1.104" required="true" output="screen"/>

  <group ns = "$(arg turtlebot_name)">
        <arg name="agent_name" default="$(arg turtlebot_name)"/>
	<include file = "$(find turtlebot_bringup)/launch/minimal.launch" output = "screen">
	</include>

        <node name="distributed_optimization" pkg="wta_distributed_optimization" type="ros_wrapper.py" output="screen"
              launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' ">
            <param name="num_weapons"     value="$(arg num_weapons)"/>
            <param name="num_targets"     value="$(arg num_targets)"/>
            <rosparam>
                is_simulated: False
            </rosparam>
            <remap from="goal_pose" to="setpoint"/>
        </node>

	<group if="$(arg run_gazebo)">
		    <arg name="x_pos" default="0.0"/>
		    <arg name="y_pos" default="0.0"/>
		    <arg name="z_pos" default="0.0"/>
		    <arg name="yaw"   default="0.0"/>
		    <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
		    <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher2" output="screen">
		        <param name="publish_frequency" type="double" value="50.0" />
		        <param name="tf_prefix" value="$(arg agent_name)" />
		    </node>

		    <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" output="log" args="-urdf -model $(arg agent_name) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -Y $(arg yaw) -param robot_description" />

	<arg name="param_file" default="$(find optitrack_to_gazebo)/params/basic_param.yaml" />
	<node pkg="optitrack_to_gazebo" type="optitrack_to_gazebo_node.py" name="optitrack_to_gazebo" >
	     <param name="rigidbody_name" value="$(arg agent_name)" type="str"/>
	    <remap from="pose_stamped" to="nwu/pose_stamped"/>
	    <rosparam file="$(arg param_file)" command="load"/>
	</node>
	</group>



	<rosparam command="load" file="$(find turtlebot_pid)/config/turtlebot_param_mocap.yaml"/>
	<node name="controller" pkg="turtlebot_pid" type="turtlebot_pid_node" args="/$(arg turtlebot_name)" output = "screen">
	<remap from="turtle_pose" to="nwu/pose"/>
	<remap from="turtle_goal" to="setpoint"/>
	<remap from="turtle_velocity_publisher" to="mobile_base/commands/velocity"/>
        <remap from="global_initialisation_finished" to="/global_initialisation_finished"/>
	</node>
    </group>
    </launch>"""


    f.write(file_str)
    f.close()

def generate_sim_turtlebot_launch_file(real_robots_number,target_number, weapon_number):
    f = open("launch/sim_agents.launch", "w")

    file_str = r"""<launch>

	 <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="paused" value="false"/>
    <arg name="use_sim_time" value="false"/>
    <arg name="gui" value="false"/>
    <arg name="recording" value="false"/>
    <arg name="debug" value="false"/>
  </include>
	<node name="rviz_node" pkg="rviz" type="rviz" args="-d $(find wta_distributed_optimization)/rviz/wta_demo.rviz"/>
 
<node name="listener" pkg="roscpp_tutorials" type="listener" launch-prefix="bash -c 'sleep 5.0; $0 $@' " />

    <arg name="model" default="burger"/>
    <arg name="run_gazebo" default="true"/>
    <arg name="node_start_delay" default="0.0" />
    <arg name="run_rviz" default="false"/>
    <arg name="record_bag" default="false"/>
    <arg name="bag_name" default="test"/>
    <arg name="num_weapons" default="""+"\""+str(weapon_number)+"\""+r"""/>
    <arg name="num_targets" default="""+"\""+str(target_number)+"\""+r"""/>
    <rosparam file="$(find wta_distributed_optimization)/params/target_position_attrition.yaml"/>
    """


    file_str += r"""
<node pkg="initialisation_synchronizer" type="initialisation_synchronizer_node.py" name="initialisation_synchronizer" >
	<rosparam>
        initialized_params_name_list: """
    initialisation_param_list = []
    for id in range(weapon_number):
        initialisation_param_list.append("/agent_"+str(id)+"/initialisation_finished")
    file_str += str(initialisation_param_list)
    file_str += r"""
        global_launch_param_name: "/global_initialisation_finished"
        user_initialisation_enable: false
        user_initialisation_name: "/user_launch"
    </rosparam>
</node>"""
    for id in range(real_robots_number, weapon_number):
        file_str += r"""<arg name="agent_"""+str(id)+r""""  default="agent_"""+str(id)+r""""/>
    <group ns="$(arg agent_"""+str(id)+r""")">
        <arg name="agent_name" default="$(arg agent_"""+str(id)+r""")"/>
        <node name="distributed_optimization" pkg="wta_distributed_optimization" type="ros_wrapper.py" output="screen"
              launch-prefix="bash -c 'sleep $(arg node_start_delay); $0 $@' ">
            <param name="num_weapons"     value="$(arg num_weapons)"/>
            <param name="num_targets"     value="$(arg num_targets)"/>
            <rosparam>
                is_simulated: True
            </rosparam>
            <remap from="goal_pose" to="setpoint"/>
        </node>

        <group if="$(arg run_gazebo)">
            <arg name="x_pos" default="""+"\""+str(-id)+"\""+r"""/>
            <arg name="y_pos" default="""+"\""+str(-4)+"\""+r"""/>
            <arg name="z_pos" default="0.0"/>
            <arg name="yaw"   default="0.0"/>
            <param name="robot_description" command="$(find xacro)/xacro --inorder $(find turtlebot3_description)/urdf/turtlebot3_$(arg model).urdf.xacro" />
            <node pkg="robot_state_publisher" type="robot_state_publisher" name="robot_state_publisher" output="screen">
                <param name="publish_frequency" type="double" value="50.0" />
                <param name="tf_prefix" value="$(arg agent_name)" />
            </node>

            <node name="spawn_urdf" pkg="gazebo_ros" type="spawn_model" output="log" args="-urdf -model $(arg agent_name) -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -Y $(arg yaw) -param robot_description" />

            <rosparam command="load" file="$(find turtlebot_pid)/config/turtlebot_param_sim.yaml"/>
            <node name="controller" pkg="turtlebot_pid" type="turtlebot_pid_node" args="/$(arg agent_name)" output = "screen">
                <remap from="turtle_pose" to="odom"/>
                <remap from="turtle_goal" to="setpoint"/>
                <remap from="turtle_velocity_publisher" to="cmd_vel"/>
                <remap from="global_initialisation_finished" to="/global_initialisation_finished"/>
            </node>
        </group>
    </group>
"""
    file_str+=r"""    
    <node if="$(arg run_rviz)" name="rviz_node" pkg="rviz" type="rviz" output="log" args="-d $(find wta_distributed_optimization)/rviz/wta_demo.rviz"/>
    <node if="$(arg record_bag)" name="record" pkg="rosbag" type="record" args="-O $(arg bag_name) dual_0 dual_1 dual_2 primal_0 primal_1 primal_2 agent_00/setpoint agent_01/setpoint agent_02/setpoint" output="screen"/>

<node pkg="wta_visualization" type="wta_visualization_node.py" name="wta_visualization" output="screen">
    <param name="number_of_target" value="4" type="int"/>
    <param name="number_of_weapon" value="5" type="int"/>
	<remap from="assignment_" to="assignment_"/>
</node>
    <node name="attrition" pkg="wta_distributed_optimization" type="attrition.py" output="screen"/>
</launch>"""
    f.write(file_str)
    f.close()

def generate_params_yaml_file(target_number, weapon_number):
    f = open("params/target_position.yaml", "w")
    file_str = r"""target_position: ["""
    for i in range(target_number):
        on_diag = random.uniform(0,1) * i + i
        on_diag = on_diag % 5
        on_diag = on_diag * random.choice([-1.,1.])
        file_str+=  "{position: ["+str(random.uniform(-4,4))+"," +str(random.uniform(-4,4))+"," +str(random.uniform(0,0.5))+"]},"
    file_str +=  r"""]
    
Pk: """
    pk = []
    for i in range(weapon_number):
        pk.append([])
        for j in range(target_number):
            pk[i].append(random.uniform(0,0.99999))
    file_str += str(pk)
    f.write(file_str)
    f.close()


if __name__ == "__main__":
    print( sys.argv[1:])
    try:
        opts, args = getopt.getopt(
            sys.argv[1:],
            "t:w:r:",
            ["target_number=", "weapons_number=", "real_robots="])
    except getopt.GetoptError as msg:
        print("error : %s" % msg)

    print(opts)
    print(args)
    target_number, weapon_number, real_robots= None, None, None
    for opt, arg in opts:
        print(opt, arg)
        if opt in ("-t", "--target_number"):
            target_number = int(arg)
        elif opt in ("-w", "--weapons_number"):
            weapon_number = int(arg)
        elif opt in ("-r", "--real_robots"):
            real_robots = int(arg)

    if target_number!= None and weapon_number!= None and real_robots!= None:
        generate_sim_turtlebot_launch_file(real_robots,target_number, weapon_number)
        #generate_params_yaml_file(target_number, weapon_number)
        for i in range(real_robots):
            generate_real_turtlebot_launch_file(i,target_number, weapon_number)


