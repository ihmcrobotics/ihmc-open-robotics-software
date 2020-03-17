package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.valkyrie.configuration.ValkyrieRobotVersion;

public class ValkyrieStandaloneFootstepPlanningModule
{
   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS, ValkyrieRobotVersion.fromEnvironment());
      FootstepPlanningModuleLauncher.createModule(robotModel, DomainFactory.PubSubImplementation.FAST_RTPS);
   }
}
