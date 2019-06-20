package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;

public class ValkyrieStandaloneFootstepPlanningToolbox
{
   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, true);
      new MultiStageFootstepPlanningModule(robotModel, null, false);
   }
}
