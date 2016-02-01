package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.darpaRoboticsChallenge.ros.messages.PoseMessage;

public interface VehiclePoseListener
{
   public void receivedVehiclePose(PoseMessage message);
}
