package us.ihmc.avatar;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.HumanoidNetworkProcessorParameters;
import us.ihmc.avatar.simulationStarter.DRCSimulationStarter;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.pathPlanning.DataSet;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotEnvironmentAwareness.tools.ConstantPlanarRegionsPublisher;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.simulationConstructionSetTools.util.environments.PlanarRegionsListDefinedEnvironment;

public class AvatarPlanarRegionsSimulation
{
   public AvatarPlanarRegionsSimulation(DRCRobotModel robotModel, DataSetName dataSetName)
   {
      DataSet dataSet = DataSetIOTools.loadDataSet(dataSetName);
      startSimulation(robotModel, dataSet.getPlanarRegionsList(), dataSet.getPlannerInput().getStartPosition(), dataSet.getPlannerInput().getStartYaw());
   }

   public static void startSimulation(DRCRobotModel robotModel, PlanarRegionsList planarRegionsList, Tuple3DReadOnly startPosition, double startOrientation)
   {
      PlanarRegionsListDefinedEnvironment simEnvironment = new PlanarRegionsListDefinedEnvironment(planarRegionsList, 0.02, true);
      DRCSimulationStarter simulationStarter = new DRCSimulationStarter(robotModel, simEnvironment);
      simulationStarter.setRunMultiThreaded(true);
      simulationStarter.setInitializeEstimatorToActual(true);
      simulationStarter.setStartingLocation(() -> new OffsetAndYawRobotInitialSetup(startPosition, startOrientation));
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
      ConstantPlanarRegionsPublisher constantPlanarRegionsPublisher = new ConstantPlanarRegionsPublisher(planarRegionsList);
      constantPlanarRegionsPublisher.start(2000);
   }
}
