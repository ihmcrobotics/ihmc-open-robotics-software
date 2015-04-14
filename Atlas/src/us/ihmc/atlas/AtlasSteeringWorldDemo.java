package us.ihmc.atlas;

import us.ihmc.darpaRoboticsChallenge.DRCSteeringWheelDemo;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;

import com.martiansoftware.jsap.JSAPException;

public class AtlasSteeringWorldDemo
{
   private static final double ROBOT_FLOATING_HEIGHT = 0.3;

   
   public static void main(final String[] args) throws JSAPException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
      boolean useHighResolutionContactPointGrid = true;
      robotModel.createHandContactPoints(useHighResolutionContactPointGrid);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.setUseUiModule(true);
      networkProcessorParameters.setUseBehaviorModule(true);
      networkProcessorParameters.setUsePerceptionModule(true);
      networkProcessorParameters.setUseSensorModule(true);
      networkProcessorParameters.setUseLocalControllerCommunicator(true);

      new DRCSteeringWheelDemo(robotModel, networkProcessorParameters, ROBOT_FLOATING_HEIGHT);
   }
}
