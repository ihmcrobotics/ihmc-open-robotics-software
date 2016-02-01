package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.ros.messages.Float64Message;

public interface BrakePedalStateListener
{
   public void receivedBrakePedalState(Float64Message state);
}
