package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.ros.messages.Float64Message;

public interface SteeringWheelStateListener
{
   public void receivedSteeringWheelState(Float64Message state);
}
