package us.ihmc.atlas;

import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.pushRecoveryToolboxModule.PushRecoveryToolboxModule;
import us.ihmc.pubsub.DomainFactory;

public class AtlasPushRecoveryToolboxModule
{
   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
      PushRecoveryToolboxModule module = new PushRecoveryToolboxModule(robotModel, true, DomainFactory.PubSubImplementation.FAST_RTPS, 9.81);
      module.wakeUp();
   }
}
