package us.ihmc.atlas;

import us.ihmc.avatar.AvatarPlanarRegionsSimulation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.BlockEnvironment;

public class AtlasPlanarRegionsSimulation
{
   private static final DataSetName DATA_SET_TO_USE = DataSetName._20200226_120200_FlatGround_StartMidRegion;

   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS);
      new AvatarPlanarRegionsSimulation(robotModel, DATA_SET_TO_USE);
   }
}
