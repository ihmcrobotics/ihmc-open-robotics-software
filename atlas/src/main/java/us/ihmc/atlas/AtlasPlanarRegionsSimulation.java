package us.ihmc.atlas;

import us.ihmc.avatar.AvatarPlanarRegionsSimulation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.pathPlanning.DataSetName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasPlanarRegionsSimulation
{
   private static final DataSetName DATA_SET_TO_USE = DataSetName._20200513_151318_StairsIHMC_Bottom;
   private static final boolean GENERATE_GROUND_PLANE = false;
   private static final boolean ADD_EXTRA_CONTACT_POINTS = true;

   public static void main(String[] args)
   {
      DRCRobotModel robotModel;
      if (ADD_EXTRA_CONTACT_POINTS)
      {
         FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, false);
         robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, true, simulationContactPoints);
      }
      else
      {
         robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS);
      }

      new AvatarPlanarRegionsSimulation(robotModel, DATA_SET_TO_USE, GENERATE_GROUND_PLANE);
   }
}
