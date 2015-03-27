package us.ihmc.communication.util;

public enum NetworkPorts
{
   // Network processor <-> Controller communication
   NETWORK_PROCESSOR_TO_CONTROLLER_TCP_PORT(4895),
   
   // Network processor <-> UI Communication
   NETWORK_PROCESSOR_TO_UI_TCP_PORT(4898),
   NETWORK_PROCESSOR_TO_UI_TEST_TCP_PORT(4899),
   
   // Cloud dispatcher ports
   NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_TCP_PORT(5000),
   NETWORK_PROCESSOR_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT(5005),
   CONTROLLER_CLOUD_DISPATCHER_BACKEND_TCP_PORT(5002),
   CONTROLLER_CLOUD_DISPATCHER_BACKEND_CONSOLE_TCP_PORT(5007),
   
   // Hands
   LEFT_HAND_PORT(5003),
   RIGHT_HAND_PORT(5004)
   ;
   
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
