package us.ihmc.atlas;

import javax.vecmath.Point3d;

import us.ihmc.darpaRoboticsChallenge.DRCSimulationStarter;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationTools;
import us.ihmc.darpaRoboticsChallenge.environment.CommonAvatarEnvironmentInterface;
import us.ihmc.darpaRoboticsChallenge.environment.DRCSteeringWheelEnvironment;
import us.ihmc.darpaRoboticsChallenge.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.humanoidOperatorInterface.driving.BasicDriverControllerHandler;

import com.martiansoftware.jsap.JSAPException;

public class AtlasSteeringWorldDemo
{
   public static void main(final String[] args) throws JSAPException
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, AtlasRobotModel.AtlasTarget.SIM, false);
      boolean useHighResolutionContactPointGrid = true;
      robotModel.createHandContactPoints(useHighResolutionContactPointGrid);
      
      Point3d steeringWheelCenterInChestFrame = BasicDriverControllerHandler.intersectionOfSteeringAxisWithGraspPlaneInPelvisZUpFrame;
      
      CommonAvatarEnvironmentInterface environment = new DRCSteeringWheelEnvironment(steeringWheelCenterInChestFrame.x, steeringWheelCenterInChestFrame.y, 0.85, 0.0, 57.0);
      
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, environment);
      simulationStarter.setRunMultiThreaded(true);

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();
      networkProcessorParameters.setUseUiModule(true);
      networkProcessorParameters.setUseBehaviorModule(true);
      networkProcessorParameters.setUsePerceptionModule(true);
      networkProcessorParameters.setUseSensorModule(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      networkProcessorParameters.setUseLocalControllerCommunicator(true);
//      simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);

      DRCSimulationTools.startSimulationWithGraphicSelector(simulationStarter);
      
   }
}
