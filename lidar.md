<!--add lidar-->
    <link name="hokuyo_link">
      <pose>0 0 0 0 0 0</pose>
      <collision name="collision">
        <pose>0 0 0.3 0 0 0</pose>
        <geometry>
          <box>
            <size>0.1 0.1 0.1</size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <pose>0 0 0.27 0 0 0</pose>
        <geometry>
          <mesh>
            <uri>model://hokuyo/meshes/hokuyo.dae</uri>
          </mesh>
        </geometry>
      </visual>
      <inertial>
        <mass>0.016</mass>
        <inertia>
            <ixx>0.0001</ixx>
            <ixy>0</ixy>
            <ixz>0</ixz>
            <iyy>0.0001</iyy>
            <iyz>0</iyz>
            <izz>0.0001</izz>
            <!-- low intertia necessary to avoid not disturb the drone -->
        </inertia>
      </inertial>

      <sensor type="ray" name="laser">
        <pose>0 0 0.3 0 0 1.57</pose>
        <visualize>true</visualize>
        <update_rate>10</update_rate>
        <ray>
          <scan>
            <horizontal>
              <samples>1024</samples>
              <resolution>1</resolution>
              <min_angle>-3.141593</min_angle>
              <max_angle>3.141593</max_angle>
            </horizontal>
          </scan>
          <range>
            <min>0.1</min>
            <max>30</max>
            <resolution>0.1</resolution>
          </range>
          <!-- <noise>
            <type>Gaussian</type>
            <mean>0.0</mean>
            <stddev>0.01</stddev>
          </noise> -->
        </ray>
        <plugin name="hokuyo_node" filename="libgazebo_ros_laser.so">
          <robotNamespace></robotNamespace>
          <topicName>/spur/laser/scan</topicName>
          <frameName>/hokuyo_sensor_link</frameName>
        </plugin>
      </sensor>
    </link>

    <joint name="hokuyo_joint" type="fixed">
      <pose>0 0 0 0 0 0</pose>
      <parent>iris::base_link</parent>
      <child>hokuyo_link</child>
    </joint>
