package us.ihmc.atlas;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.atlas.initialSetup.AtlasInitialSetupFromFile;
import us.ihmc.darpaRoboticsChallenge.DRCSteeringWheelDemo;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;

public class AtlasSteeringWorldDemo
{
   
   public static void main(final String[] args) throws JSAPException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
      boolean useHighResolutionContactPointGrid = true;
      robotModel.createHandContactPoints(useHighResolutionContactPointGrid);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableUiModule(true);
      networkProcessorParameters.enableBehaviorModule(true);
      networkProcessorParameters.enablePerceptionModule(true);
      networkProcessorParameters.enableSensorModule(true);
      networkProcessorParameters.enableLocalControllerCommunicator(true);
      
      AtlasInitialSetupFromFile initialSetup = new AtlasInitialSetupFromFile("initialDrivingSetup");

      double percentOfSteeringWheelRadius = 1.0;
      new DRCSteeringWheelDemo(robotModel, networkProcessorParameters, initialSetup, percentOfSteeringWheelRadius);
   }
}
