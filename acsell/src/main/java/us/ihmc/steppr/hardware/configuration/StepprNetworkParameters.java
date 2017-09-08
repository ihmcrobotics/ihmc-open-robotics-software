package us.ihmc.steppr.hardware.configuration;

import us.ihmc.acsell.hardware.configuration.AcsellNetworkParameters;

public class StepprNetworkParameters extends AcsellNetworkParameters
{

   @Override
   public String getMultiCastGroup()
   {
      return ACSELL_MULTICAST_GROUP;
   }

   @Override
   public int getMulticastControlPort()
   {
      return UDP_MULTICAST_CONTROL_PORT;
   }
}
