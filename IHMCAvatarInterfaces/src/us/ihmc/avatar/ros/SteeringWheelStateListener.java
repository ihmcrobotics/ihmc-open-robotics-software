package us.ihmc.avatar.ros;

import us.ihmc.avatar.ros.messages.Float64Message;

public interface SteeringWheelStateListener
{
   public void receivedSteeringWheelState(Float64Message state);
}
