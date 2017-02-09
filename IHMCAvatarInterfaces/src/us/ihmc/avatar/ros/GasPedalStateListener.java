package us.ihmc.avatar.ros;

import us.ihmc.avatar.ros.messages.Float64Message;

public interface GasPedalStateListener
{
   public void receivedGasPedalState(Float64Message state);
}
