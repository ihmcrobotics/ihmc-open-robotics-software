package us.ihmc.valkyrie;

import us.ihmc.avatar.AvatarPlanarRegionsSimulation;
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

public class ValkyriePlanarRegionsSimulation
{
   private static final DataSetName DATA_SET_TO_USE = DataSetName._20200226_120200_FlatGround_StartMidRegion;

   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      new AvatarPlanarRegionsSimulation(robotModel, DATA_SET_TO_USE);
   }
}
