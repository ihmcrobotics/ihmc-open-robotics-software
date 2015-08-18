package us.ihmc.communication.configuration;

public enum NetworkParameterKeys
{
   robotController(true, "Hostname/IP of the robot controller."),
   logger(true, "Hostname/IP of the logger or visualizer network."),
   networkManager(false, "Hostname/IP of the network manager, as seen from the communication shaper."),
   networkManagerForUI(false, "Hostname/IP of the networkmanager, as seen from the UI. In normal operation, this is the same as networkmanager"),
   
   rosURI(false, "Fully qualified ROS master URI."),
   head(false, "Hostname/IP of the head."),
   leftHand(false, "Hostname/IP of the left hand."),
   rightHand(false, "Hostname/IP of the right hand."),
   
   onboard1(false, "IP of first onboard computer"),
   onboard2(false, "IP of second onboard computer"),
   onboard3(false, "IP of third onboard computer"),
   
   packetShaperServer(false, "IP of the packet shaper Field server"),
   packetShaperClient(false, "IP of the packet shaper OCU client");
   
   
   
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
