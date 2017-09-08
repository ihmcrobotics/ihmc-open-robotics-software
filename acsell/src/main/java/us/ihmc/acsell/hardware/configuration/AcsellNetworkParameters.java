package us.ihmc.acsell.hardware.configuration;

public abstract class AcsellNetworkParameters
{
   
   public static final int VARIABLE_SERVER_PORT = 5555;   
   
   public static final int UDP_MULTICAST_STATE_PORT = 11303;
   public static final int UDP_MULTICAST_CONTROL_PORT = 11305;
   
   public static final int UDP_MULTICAST_STREAM_COMMAND_PORT = 11300;
   
   public static final int UDP_MULTICAST_PARAMETER_REQUEST_PORT = 11302;
   public static final int UDP_MULTICAST_PARAMETER_REPLY_PORT = 11306;
   
   public static final int UDP_MULTICAST_POWER_BUS_PORT = 11302;
   
   
   public static final String ACSELL_MULTICAST_GROUP = "224.0.0.123";

   public abstract String getMultiCastGroup();

   public abstract int getMulticastControlPort();

}
