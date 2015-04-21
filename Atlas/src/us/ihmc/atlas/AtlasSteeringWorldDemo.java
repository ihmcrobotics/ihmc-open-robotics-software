package us.ihmc.atlas;

import us.ihmc.atlas.initialSetup.AtlasDrivingInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSteeringWheelDemo;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;

import com.martiansoftware.jsap.JSAPException;

public class AtlasSteeringWorldDemo
{
   
   public static void main(final String[] args) throws JSAPException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
      boolean useHighResolutionContactPointGrid = true;
      robotModel.createHandContactPoints(useHighResolutionContactPointGrid);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableUiModule(true);
      networkProcessorParameters.enableBehaviorModule(true);
      networkProcessorParameters.enablePerceptionModule(true);
      networkProcessorParameters.enableSensorModule(true);
      networkProcessorParameters.enableLocalControllerCommunicator(true);
      
      AtlasDrivingInitialSetup initialSetup = new AtlasDrivingInitialSetup();

      double percentOfSteeringWheelRadius = 1.0;
      new DRCSteeringWheelDemo(robotModel, networkProcessorParameters, initialSetup, percentOfSteeringWheelRadius);
   }
}
