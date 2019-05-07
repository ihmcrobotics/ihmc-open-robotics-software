package us.ihmc.communication.util;

import java.util.Random;

public class NetworkPorts
{
   // Network processor <-> Controller communication
   public static final NetworkPorts CONTROLLER_PORT = new NetworkPorts(4895, "controller");

   // Network processor <-> UI Communication
   public static final NetworkPorts NETWORK_PROCESSOR_TO_UI_TCP_PORT = new NetworkPorts(4898, "network_processor_to_ui");
   public static final NetworkPorts NETWORK_PROCESSOR_TO_UI_TEST_TCP_PORT = new NetworkPorts(4899, "network_processor_to_ui");

   // Cloud dispatcher ports
   public static final NetworkPorts NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT = new NetworkPorts(5000, "network_processor_cloud_dispatcher_backend");
   public static final NetworkPorts NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT = new NetworkPorts(5005, "network_processor_cloud_dispatcher_backend_console");
   public static final NetworkPorts CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT = new NetworkPorts(5002, "controller_cloud_dispatcher_backend");
   public static final NetworkPorts CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT = new NetworkPorts(5007, "controller_cloud_dispatcher_backend_console");

   // Hands
   public static final NetworkPorts LEFT_HAND_PORT = new NetworkPorts(5003, "left_hand");
   public static final NetworkPorts RIGHT_HAND_PORT = new NetworkPorts(5004, "right_hand");

   // Network manager ports
   public static final NetworkPorts BEHAVIOUR_MODULE_PORT = new NetworkPorts(6001, "behaviour_module");
   public static final NetworkPorts BEHAVIOUR_COMMUNICATION_PORT = new NetworkPorts(6000, "behaviour_communication");

   public static final NetworkPorts UI_MODULE = new NetworkPorts(6002, "ui_module");
   public static final NetworkPorts SENSOR_MANAGER = new NetworkPorts(6003, "sensor_manager");
   public static final NetworkPorts LEFT_HAND_MANAGER_PORT = new NetworkPorts(6004, "left_hand_manager");
   public static final NetworkPorts RIGHT_HAND_MANAGER_PORT = new NetworkPorts(6005, "right_hand_manager");
   public static final NetworkPorts ROS_MODULE = new NetworkPorts(6006, "ros_module");
   public static final NetworkPorts ROS_API_COMMUNICATOR = new NetworkPorts(6007, "ros_api_communicator");
   public static final NetworkPorts MOCAP_MODULE = new NetworkPorts(6008, "mocap_module");
   public static final NetworkPorts MULTISENSE_MOCAP_MANUAL_CALIBRATION_TEST_MODULE = new NetworkPorts(6009, "multisense_mocap_manual_calibration_test_module");
   public static final NetworkPorts ROS_AUXILIARY_ROBOT_DATA_PUBLISHER = new NetworkPorts(6010, "ros_auxiliary_robot_data_publisher");
   public static final NetworkPorts ZERO_POSE_PRODUCER = new NetworkPorts(6011, "zero_pose_producer");
   public static final NetworkPorts DRILL_DETECTOR = new NetworkPorts(6012, "drill_detector");
   public static final NetworkPorts TEXT_TO_SPEECH = new NetworkPorts(6013, "text_to_speech");
   public static final NetworkPorts FACE_TRACKING = new NetworkPorts(6014, "face_tracking");
   public static final NetworkPorts AUDIO_MODULE_PORT = new NetworkPorts(6015, "audio_module");
   public static final NetworkPorts TOUCH_MODULE_PORT = new NetworkPorts(6016, "touch_module");
   public static final NetworkPorts KINEMATICS_TOOLBOX_MODULE_PORT = new NetworkPorts(6017, "kinematics_toolbox_module");
   public static final NetworkPorts COACTIVE_ELEMENTS_PORT = new NetworkPorts(6018, "coactive_elements");
   /** Port for the robot environment awareness module. Not yet available in the open source repo. */
   public static final NetworkPorts REA_MODULE_PORT = new NetworkPorts(6019, "rea_module");
   public static final NetworkPorts FOOTSTEP_PLANNING_TOOLBOX_MODULE_PORT = new NetworkPorts(6020, "footstep_planning_toolbox_module");
   public static final NetworkPorts HEIGHT_QUADTREE_TOOLBOX_MODULE_PORT = new NetworkPorts(6021, "height_quadtree_toolbox_module");
   public static final NetworkPorts REA_MODULE_UI_PORT = new NetworkPorts(6022, "rea_module_ui");
   public static final NetworkPorts LIDAR_SCAN_LOGGER_PORT = new NetworkPorts(6023, "lidar_scan_logger");
   public static final NetworkPorts VALVE_DETECTOR_SERVER_PORT = new NetworkPorts(6024, "valve_detector_server");
   public static final NetworkPorts VALVE_DETECTOR_FEEDBACK_PORT = new NetworkPorts(6025, "valve_detector_feedback");
   public static final NetworkPorts MOCAP_MODULE_VIZ = new NetworkPorts(6026, "mocap_module_viz");
   public static final NetworkPorts VISIBILITY_GRAPHS = new NetworkPorts(6027, "visibility_graphs");
   public static final NetworkPorts WHOLE_BODY_TRAJECTORY_TOOLBOX_MODULE_PORT = new NetworkPorts(6028, "whole_body_trajectory_toolbox_module");
   public static final NetworkPorts CONTROLLER_FILTER_MODULE_PORT = new NetworkPorts(6029, "controller_filter_module");
   public static final NetworkPorts JOYSTICK_BASED_CONTINUOUS_STEPPING = new NetworkPorts(6030, "joystick_based_stepping");

   // Mission control ports
   public static final NetworkPorts MISSION_CONTROL_SERVER_PORT = new NetworkPorts(2046, "mission_control_server");
   public static final NetworkPorts MISSION_CONTROL_CPU0_TEST_PORT = new NetworkPorts(2147, "mission_control_cpu0_test");
   public static final NetworkPorts MISSION_CONTROL_CPU1_TEST_PORT = new NetworkPorts(2148, "mission_control_cpu1_test");
   public static final NetworkPorts MISSION_CONTROL_CPU2_TEST_PORT = new NetworkPorts(2149, "mission_control_cpu2_test");

   // Teleop ports
   public static final NetworkPorts XBOX_CONTROLLER_TELEOP_PORT = new NetworkPorts(3001, "xbox_controller_teleop");

   // Dynamic ports for testing purposes
   public static NetworkPorts createMissionControlIntraprocessPort(NetworkPorts port)
   {
      return new NetworkPorts(port.getPort() + 100, port.getName() + "_intraprocess");
   }

   public static NetworkPorts createRandomTestPort(Random random)
   {
      int port = random.nextInt(65535 - 1025) + 1025;
      return new NetworkPorts(port, "random_" + port);
   }

   /**
    * The network port.
    */
   private final int port;

   /**
    * Unique name used for corresponding ROS topics.
    *
    * Must be lower_undercased and starting with a letter.
    */
   private final String name;

   private NetworkPorts(int port, String name)
   {
      this.port = port;
      this.name = name;
   }

   public int getPort()
   {
      return port;
   }

   public String getName()
   {
      return name;
   }

   @Override
   public String toString()
   {
      return Integer.toString(port);
   }
}
