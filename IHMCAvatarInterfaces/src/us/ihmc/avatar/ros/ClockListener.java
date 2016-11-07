package us.ihmc.avatar.ros;

import us.ihmc.avatar.ros.messages.ClockMessage;

public interface ClockListener
{
   public void receivedClockMessage(ClockMessage message);
}
