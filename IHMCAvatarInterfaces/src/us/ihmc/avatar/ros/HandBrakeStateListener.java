package us.ihmc.avatar.ros;

import us.ihmc.avatar.ros.messages.Float64Message;

public interface HandBrakeStateListener
{
   public void receivedHandBrakeState(Float64Message state);
}
