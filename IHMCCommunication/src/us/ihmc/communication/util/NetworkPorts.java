package us.ihmc.communication.util;

public enum NetworkPorts
{
   // Network processor <-> Controller communication
   CONTROLLER_PORT(4895),
   
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
   RIGHT_HAND_PORT(5004),
   
   
   // Test ports
   TEST_PORT_A(21090),
   TEST_PORT_B(21091),
   TEST_PORT_C(21092),
   TEST_PORT_D(21093),
   TEST_PORT_E(21094),
   TEST_PORT_F(21095), 
   
   // Network manager ports
   BEHAVIOUR_MODULE_PORT(6001), 
   UI_MODULE(6002), 
   SENSOR_MANAGER(6003), 
   LEFT_HAND_MANAGER_PORT(6004),
   RIGHT_HAND_MANAGER_PORT(6005), 
   ROS_MODULE(6006), 
   GFE_COMMUNICATOR(6007),
   
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
