package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.ros.messages.Float64Message;

public interface GasPedalStateListener
{
   public void receivedGasPedalState(Float64Message state);
}
