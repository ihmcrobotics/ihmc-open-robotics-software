package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotEnvironmentAwareness.tools.ConstantPlanarRegionsPublisher;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;

public class ValkyrieCTTSOSimulation
{
   private static final DataSetName DATA_SET_TO_USE = DataSetName._20190220_172417_EOD_Cinders;

   public static void main(String[] args)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(DATA_SET_TO_USE);

      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      PlanarRegionsListDefinedEnvironment simEnvironment = new PlanarRegionsListDefinedEnvironment(dataSet.getPlanarRegionsList(), 0.02, true);

      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, simEnvironment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setStartingLocation(() -> new OffsetAndYawRobotInitialSetup(dataSet.getPlannerInput().getStartPosition(),
                                                                                    dataSet.getPlannerInput().getStartYaw()));

      HumanoidNetworkProcessorParameters networkProcessorParameters = new HumanoidNetworkProcessorParameters();

      // talk to controller and footstep planner
      networkProcessorParameters.setUseFootstepPlanningToolboxModule(true);
      networkProcessorParameters.setUseWalkingPreviewModule(true);
      networkProcessorParameters.setUseBipedalSupportPlanarRegionPublisherModule(true);

      // disable everything else
      networkProcessorParameters.setUseSensorModule(true);
      networkProcessorParameters.setUseHumanoidAvatarREAStateUpdater(true);

      // start sim
      simulationStarter.startSimulation(networkProcessorParameters, false);

      // spoof and publish planar regions
      ConstantPlanarRegionsPublisher constantPlanarRegionsPublisher = new ConstantPlanarRegionsPublisher(dataSet.getPlanarRegionsList());
      constantPlanarRegionsPublisher.start(2000);
   }
}
