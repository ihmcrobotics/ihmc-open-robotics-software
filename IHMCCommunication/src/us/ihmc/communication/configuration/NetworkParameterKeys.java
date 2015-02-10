package us.ihmc.communication.configuration;

public enum NetworkParameterKeys
{
   robotController(true, "Hostname/IP of the robot controller."),
   logger(true, "Hostname/IP of the logger."),
   networkManager(true, "Hostname/IP of the network manager, as seen from the communication shaper."),
   networkManagerForUI(true, "Hostname/IP of the networkmanager, as seen from the UI. In normal operation, this is the same as networkmanager"),
   
   rosHost(false, "ROS hostname/IP."),
   head(false, "Hostname/IP of the head."),
   leftHand(false, "Hostname/IP of the left hand."),
   rightHand(false, "Hostname/IP of the right hand.");
   
   
   private final boolean required;
   private final String description;
   private NetworkParameterKeys(boolean required, String description)
   {
      this.required = required;
      this.description = description; 
   }
   
   
   public String getDescription()
   {
      return description;
   }
   
   public boolean isRequired()
   {
      return required;
   }
   
}
