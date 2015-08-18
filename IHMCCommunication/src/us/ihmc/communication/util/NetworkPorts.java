package us.ihmc.communication.util;

import java.util.Random;

public class NetworkPorts
{
   // Network processor <-> Controller communication
   public static final NetworkPorts CONTROLLER_PORT = new NetworkPorts(4895);

   // Network processor <-> UI Communication
   public static final NetworkPorts NETWORK_PROCESSOR_TO_UI_TCP_PORT = new NetworkPorts(4898);
   public static final NetworkPorts NETWORK_PROCESSOR_TO_UI_TEST_TCP_PORT = new NetworkPorts(4899);

   // Cloud dispatcher ports
   public static final NetworkPorts NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT = new NetworkPorts(5000);
   public static final NetworkPorts NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT = new NetworkPorts(5005);
   public static final NetworkPorts CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT = new NetworkPorts(5002);
   public static final NetworkPorts CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT = new NetworkPorts(5007);

   // Hands
   public static final NetworkPorts LEFT_HAND_PORT = new NetworkPorts(5003);
   public static final NetworkPorts RIGHT_HAND_PORT = new NetworkPorts(5004);

   // Network manager ports
   public static final NetworkPorts BEHAVIOUR_MODULE_PORT = new NetworkPorts(6001); 
   public static final NetworkPorts UI_MODULE = new NetworkPorts(6002);
   public static final NetworkPorts SENSOR_MANAGER = new NetworkPorts(6003);
   public static final NetworkPorts LEFT_HAND_MANAGER_PORT = new NetworkPorts(6004);
   public static final NetworkPorts RIGHT_HAND_MANAGER_PORT = new NetworkPorts(6005); 
   public static final NetworkPorts ROS_MODULE = new NetworkPorts(6006);
   public static final NetworkPorts GFE_COMMUNICATOR = new NetworkPorts(6007);
   public static final NetworkPorts MOCAP_MODULE = new NetworkPorts(6008);
   public static final NetworkPorts MULTISENSE_MOCAP_MANUAL_CALIBRATION_TEST_MODULE = new NetworkPorts(6009);
   public static final NetworkPorts ROS_AUXILIARY_ROBOT_DATA_PUBLISHER = new NetworkPorts(6010);
   public static final NetworkPorts ZERO_POSE_PRODUCER = new NetworkPorts(6011);
   public static final NetworkPorts DRILL_DETECTOR = new NetworkPorts(6012);

   // Mission control ports
   public static final NetworkPorts MISSION_CONTROL_SERVER_PORT = new NetworkPorts(2046);
   public static final NetworkPorts MISSION_CONTROL_CPU0_TEST_PORT = new NetworkPorts(2147);
   public static final NetworkPorts MISSION_CONTROL_CPU1_TEST_PORT = new NetworkPorts(2148);
   public static final NetworkPorts MISSION_CONTROL_CPU2_TEST_PORT = new NetworkPorts(2149);

   // Dynamic ports for testing purposes
   public static NetworkPorts createMissionControlIntraprocessPort(NetworkPorts port)
   {
      return new NetworkPorts(port.getPort() + 100);
   }

   public static NetworkPorts createRandomTestPort()
   {
      return new NetworkPorts(new Random().nextInt(65535 - 1025) + 1025);
   }
   
   private final int port;

   private NetworkPorts(int port)
   {
      this.port = port;
   }
   
   public int getPort()
   {
      return port;
   }

   @Override
   public String toString()
   {
      return Integer.toString(port);
   }
}
