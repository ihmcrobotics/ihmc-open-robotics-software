package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataClearCommand;
import com.yobotics.simulationconstructionset.simulatedSensors.DepthDataStateCommand;

public interface DepthDataStateCommandListenerInterface
{
   public void setLidarState(DepthDataStateCommand lidarStateCommand);

   public void clearLidar(DepthDataClearCommand object);

   public void setFilterParameters(DepthDataFilterParameters parameters);
}
