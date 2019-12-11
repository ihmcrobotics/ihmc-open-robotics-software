package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class ValkyrieMultistageFootstepPlanningModule
{
   public ValkyrieMultistageFootstepPlanningModule()
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.REAL_ROBOT);
      new MultiStageFootstepPlanningModule(robotModel, null, true, PubSubImplementation.FAST_RTPS);
   }

   public static void main(String[] args)
   {
      new ValkyrieMultistageFootstepPlanningModule();
   }
}
