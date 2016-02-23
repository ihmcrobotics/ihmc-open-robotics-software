package us.ihmc.communication.configuration;

public enum NetworkParameterKeys
{
   robotController(true, "Hostname/IP of the robot controller.", "127.0.0.1", true),
   logger(true, "Hostname/IP of the logger or visualizer network.", "127.0.0.1", true),
   networkManager(false, "Hostname/IP of the network manager, as seen from the communication shaper.", "127.0.0.1", true),
   
   rosURI(false, "Fully qualified ROS master URI.", "http://127.0.0.1:11311", true),
   head(false, "Hostname/IP of the head.", "", true),
   leftHand(false, "Hostname/IP of the left hand.", "", true),
   rightHand(false, "Hostname/IP of the right hand.", "", true),
   
   loggedCameras(false, "Cameras to be recorded by the logger. Should be a comma seperated list", "", false);
   
   
   private final boolean required;
   private final String description;
   private final String defaultValue;
   private final boolean isIPAddress;

   private NetworkParameterKeys(boolean required, String description, String defaultValue, boolean isIPAddress)
   {
      this.required = required;
      this.description = description; 
      this.defaultValue = defaultValue;
      this.isIPAddress = isIPAddress;
   }
   
   
   public String getDescription()
   {
      return description;
   }
   
   public boolean isRequired()
   {
      return required;
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
