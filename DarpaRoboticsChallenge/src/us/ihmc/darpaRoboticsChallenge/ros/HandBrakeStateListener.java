package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.ros.messages.Float64Message;

public interface HandBrakeStateListener
{
   public void receivedHandBrakeState(Float64Message state);
}
