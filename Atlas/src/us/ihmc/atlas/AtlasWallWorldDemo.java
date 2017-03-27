package us.ihmc.atlas;

import com.martiansoftware.jsap.JSAPException;

import us.ihmc.atlas.parameters.AtlasContactPointParameters;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.WallWorldEnvironment;

public class AtlasWallWorldDemo
{

   public static void main(final String[] args) throws JSAPException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_INVISIBLE_CONTACTABLE_PLANE_HANDS, DRCRobotModel.RobotTarget.SCS, false);
      CommonAvatarEnvironmentInterface environment = new WallWorldEnvironment(-10.0, 10.0);

      AtlasContactPointParameters contactPointParameters = robotModel.getContactPointParameters();
      contactPointParameters.createHandKnobContactPoints();

//      double stepHeight = 0.2;
//      CommonAvatarEnvironmentInterface environment = new BigStepUpWithHandPlatformEnvironment(stepHeight);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setRunMultiThreaded(true);

      boolean automaticallyStartSimulation = true;
      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.enableUiModule(true);
      networkProcessorParameters.enableBehaviorModule(true);
      networkProcessorParameters.enablePerceptionModule(true);
      networkProcessorParameters.enableSensorModule(true);
      networkProcessorParameters.enableLocalControllerCommunicator(true);
      simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);
   }
}
