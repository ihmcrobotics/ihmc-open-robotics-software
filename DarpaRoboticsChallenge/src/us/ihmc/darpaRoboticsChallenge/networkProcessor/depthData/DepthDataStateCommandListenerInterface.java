package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;

public interface DepthDataStateCommandListenerInterface
{
   public void setLidarState(DepthDataStateCommand lidarStateCommand);

   public void clearLidar(DepthDataClearCommand object);

   public void setFilterParameters(DepthDataFilterParameters parameters);
}
