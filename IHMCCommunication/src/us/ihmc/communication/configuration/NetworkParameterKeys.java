package us.ihmc.communication.configuration;

public enum NetworkParameterKeys
{
   robotController(true, "Hostname/IP of the robot controller.", "127.0.0.1"),
   logger(true, "Hostname/IP of the logger or visualizer network.", "127.0.0.1"),
   networkManager(false, "Hostname/IP of the network manager, as seen from the communication shaper.", "127.0.0.1"),
   
   rosURI(false, "Fully qualified ROS master URI.", "http://127.0.0.1:11311"),
   head(false, "Hostname/IP of the head.", ""),
   leftHand(false, "Hostname/IP of the left hand.", ""),
   rightHand(false, "Hostname/IP of the right hand.", ""), 
   
   loggedCameras(false, "Cameras to be recorded by the logger. Should be a comma seperated list", "");
   
   
   private final boolean required;
   private final String description;
   private final String defaultValue;
   
   private NetworkParameterKeys(boolean required, String description, String defaultValue)
   {
      this.required = required;
      this.description = description; 
      this.defaultValue = defaultValue;
   }
   
   
   public String getDescription()
   {
      return description;
   }
   
   public boolean isRequired()
   {
      return required;
   }
   
   public String getDefaultValue()
   {
      return defaultValue;
   }
}
