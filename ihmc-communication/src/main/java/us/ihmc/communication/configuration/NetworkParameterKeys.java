package us.ihmc.communication.configuration;

public enum NetworkParameterKeys
{
   robotController("Hostname/IP of the robot controller.", "127.0.0.1", true),
   logger("Hostname/IP of the logger or visualizer network.", "127.0.0.1", true),
   networkManager("Hostname/IP of the network manager, as seen from the communication shaper.", "127.0.0.1", true),
   
   rosURI("Fully qualified ROS master URI.", "http://127.0.0.1:11311", false),
   head("Hostname/IP of the head.", "", true),
   leftHand("Hostname/IP of the left hand.", "", true),
   rightHand("Hostname/IP of the right hand.", "", true),
   
   loggedCameras("Cameras to be recorded by the logger. Should be a comma seperated list", "", false), 
   RTPSDomainID("RTPS Domain ID used for rtps publishers and subscribers", "-1", false);
   
   
   private final String description;
   private final String defaultValue;
   private final boolean isIPAddress;

   private NetworkParameterKeys(String description, String defaultValue, boolean isIPAddress)
   {
      this.description = description; 
      this.defaultValue = defaultValue;
      this.isIPAddress = isIPAddress;
   }
   
   public String getDescription()
   {
      return description;
   }

   public boolean isIPAddress()
   {
      return isIPAddress;
   }

   public String getDefaultValue()
   {
      return defaultValue;
   }
}
