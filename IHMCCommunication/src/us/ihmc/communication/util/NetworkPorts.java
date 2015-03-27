package us.ihmc.communication.util;

public class NetworkPorts
{
   public static final boolean USE_BEHAVIORS_MODULE = false;

   public static final boolean ENABLE_TESTBED_ALIGNMENT = false;
   
   public static final int NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT = 4895;
   public static final int NETWORK_PROCESSOR_TO_UI_TCP_PORT = 4897;
   public static final int NETWORK_PROCESSOR_TO_UI_RAW_PROTOCOL_TCP_PORT = 4898;
   public static final int NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT = 5000;
   public static final int CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT = 5002;
   public static final int DEFAULT_YOVARIABLE_SERVER_PORT = 5555;
   public static final int LEFT_HAND_PORT = 5003;
   public static final int RIGHT_HAND_PORT = 5004;
//   public static final int NETWORK_PROCESSOR_TO_BEHAVIOR_MODULE_TCP_PORT = 4900;
//   public static final int BEHAVIOR_MODULE_TO_CONTROLLER_TCP_PORT = 4901;
   
//   public static final int NETWORK_PROCESSOR_TCP_PORT = USE_BEHAVIORS_MODULE ? NETWORK_PROCESSOR_TO_BEHAVIOR_MODULE_TCP_PORT : NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT;
//   public static final int CONTROLLER_TCP_PORT = USE_BEHAVIORS_MODULE ? BEHAVIOR_MODULE_TO_CONTROLLER_TCP_PORT : NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT;
}
