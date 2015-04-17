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

   // Mission Control ports
   public static final NetworkPorts MISSION_CONTROL_SERVER_PORT = new NetworkPorts(7010);
   public static final NetworkPorts MISSION_CONTROL_INTRAPROCESS_PORT = new NetworkPorts(7030);
   public static final NetworkPorts MISSION_CONTROL_CPU0_TEST_PORT = new NetworkPorts(7040);
   public static final NetworkPorts MISSION_CONTROL_CPU1_TEST_PORT = new NetworkPorts(7041);
   public static final NetworkPorts MISSION_CONTROL_CPU2_TEST_PORT = new NetworkPorts(7042);
   public static final NetworkPorts MISSION_CONTROL_FIELD_SHAPER_TEST_PORT = new NetworkPorts(7043);
   public static final NetworkPorts MISSION_CONTROL_OCU_SHAPER_TEST_PORT = new NetworkPorts(7044);

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
   
   public static final boolean USE_BEHAVIORS_MODULE = false;
}
