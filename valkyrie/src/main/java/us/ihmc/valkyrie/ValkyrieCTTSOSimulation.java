package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.robotEnvironmentAwareness.tools.ConstantPlanarRegionsPublisher;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;

public class ValkyrieCTTSOSimulation
{
   private static final String IHMC_JERSEY_BARRIERS_NARROW  = "20190220_172417_Jersey_Barriers_IHMC_55cm";
   private static final String IHMC_JERSEY_BARRIERS_WIDE    = "20190220_172417_Jersey_Barriers_IHMC_65cm";
   private static final String JSC_JERSEY_BARRIERS_NARROW   = "20190220_172417_Jersey_Barriers_JSC_60cm";
   private static final String JSC_JERSEY_BARRIERS_WIDE     = "20190220_172417_Jersey_Barriers_JSC_78cm";
   private static final String JSC_CINDERS                  = "20190220_172417_EOD_Cinders";
   private static final String JSC_JERSEY_KNEE_17           = "20190402_114002_Jersey_17_KneeCollision";
   private static final String JSC_JERSEY_KNEE_18           = "20190402_113344_Jersey_18_KneeCollision";

   private static final String DATA_SET_TO_USE = JSC_JERSEY_KNEE_17;

   public static void main(String[] args)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(DATA_SET_TO_USE);

      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, false);
      PlanarRegionsListDefinedEnvironment simEnvironment = new PlanarRegionsListDefinedEnvironment(dataSet.getPlanarRegionsList(), 0.02, true);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, simEnvironment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setStartingLocation(() -> new OffsetAndYawRobotInitialSetup(dataSet.getPlannerInput().getStartPosition(), dataSet.getPlannerInput().getStartYaw()));

      DRCNetworkModuleParameters networkProcessorParameters = new DRCNetworkModuleParameters();

      // talk to controller and footstep planner
      networkProcessorParameters.enableControllerCommunicator(true);
      networkProcessorParameters.enableFootstepPlanningToolbox(true);
      networkProcessorParameters.enableWalkingPreviewToolbox(true);
      networkProcessorParameters.enableBipedalSupportPlanarRegionPublisher(true);

      networkProcessorParameters.enablePerceptionModule(true);

      // disable everything else
      networkProcessorParameters.enableUiModule(false);
      networkProcessorParameters.enableBehaviorModule(false);
      networkProcessorParameters.enableBehaviorVisualizer(false);
      networkProcessorParameters.enableSensorModule(true);
      networkProcessorParameters.enableZeroPoseRobotConfigurationPublisherModule(false);
      networkProcessorParameters.setEnableJoystickBasedStepping(false);
      networkProcessorParameters.enableRosModule(false);
      networkProcessorParameters.enableLocalControllerCommunicator(false);
      networkProcessorParameters.enableKinematicsToolbox(false);
      networkProcessorParameters.enableWholeBodyTrajectoryToolbox(false);
      networkProcessorParameters.enableKinematicsPlanningToolbox(false);
      networkProcessorParameters.enableRobotEnvironmentAwerenessModule(false);
      networkProcessorParameters.enableMocapModule(false);
      networkProcessorParameters.enableAutoREAStateUpdater(true);

      // start sim
      simulationStarter.startSimulation(networkProcessorParameters, false);

      // spoof and publish planar regions
      ConstantPlanarRegionsPublisher constantPlanarRegionsPublisher = new ConstantPlanarRegionsPublisher(dataSet.getPlanarRegionsList());
      constantPlanarRegionsPublisher.start(2000);
   }
}
