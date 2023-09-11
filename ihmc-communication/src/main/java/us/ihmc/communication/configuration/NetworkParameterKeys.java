package us.ihmc.communication.configuration;

public enum NetworkParameterKeys
{
   robotController("Hostname/IP of the robot controller.", "127.0.0.1", true),
   networkManager("Hostname/IP of the network manager, as seen from the communication shaper.", "127.0.0.1", true),

   rosURI("Fully qualified ROS master URI.", "http://127.0.0.1:11311", false),
   head("Hostname/IP of the head.", "", true),
   leftHand("Hostname/IP of the left hand.", "", true),
   rightHand("Hostname/IP of the right hand.", "", true),

   RTPSDomainID("RTPS Domain ID used for rtps publishers and subscribers", "-1", false),
   RTPSSubnet("RTPS subnet restriction, expected format: \"192.168.1.0/24\". "
              + "When provided RTPS only communicate through the indicated sub-network. "
              + "Might not work on Windows!", null, true);

   private final String description;
   private final String defaultValue;
   private final boolean isIPAddress;

   NetworkParameterKeys(String description, String defaultValue, boolean isIPAddress)
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
