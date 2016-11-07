package us.ihmc.avatar.ros;

import us.ihmc.avatar.ros.messages.Float64Message;

public interface BrakePedalStateListener
{
   public void receivedBrakePedalState(Float64Message state);
}
