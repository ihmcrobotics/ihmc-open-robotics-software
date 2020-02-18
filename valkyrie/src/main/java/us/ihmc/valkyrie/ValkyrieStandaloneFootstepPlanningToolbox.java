package us.ihmc.valkyrie;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.pubsub.DomainFactory;

public class ValkyrieStandaloneFootstepPlanningToolbox
{
   public static void main(String[] args)
   {
      ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      new FootstepPlanningToolboxModule(robotModel).setupWithRos(DomainFactory.PubSubImplementation.FAST_RTPS);
   }
}
