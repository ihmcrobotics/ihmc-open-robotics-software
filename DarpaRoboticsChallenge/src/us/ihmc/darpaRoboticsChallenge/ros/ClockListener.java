package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.ros.messages.ClockMessage;

public interface ClockListener
{
   public void receivedClockMessage(ClockMessage message);
}
