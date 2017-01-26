package us.ihmc.avatar.ros;

import us.ihmc.avatar.ros.messages.PoseMessage;

public interface VehiclePoseListener
{
   public void receivedVehiclePose(PoseMessage message);
}
