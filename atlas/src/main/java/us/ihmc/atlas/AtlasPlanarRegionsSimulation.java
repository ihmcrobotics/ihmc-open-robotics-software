package us.ihmc.atlas;

import us.ihmc.avatar.AvatarPlanarRegionsSimulation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.BlockEnvironment;

public class AtlasPlanarRegionsSimulation
{
   private static final DataSetName DATA_SET_TO_USE = DataSetName._20190219_182005_SteppingStones;

   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS);
      double height = 0.3;
      BlockEnvironment blockEnvironment = new BlockEnvironment(1.0, 1.0, height);
      AvatarPlanarRegionsSimulation.startSimulation(robotModel, blockEnvironment.getPlanarRegionsList(), new Vector3D(0.0, 0.0, height), 0.0);
   }
}
