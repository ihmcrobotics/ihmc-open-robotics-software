package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.RemoteBehaviorInterface;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.messager.Messager;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasBehaviorUIAndModule
{
   public AtlasBehaviorUIAndModule()
   {
      ThreadTools.startAThread(() -> new AtlasBehaviorModule(), AtlasBehaviorUIAndModule.class.getSimpleName());

      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);

      Messager behaviorMessager = RemoteBehaviorInterface.createForUI("localhost");

      new BehaviorUI(behaviorMessager, drcRobotModel, PubSubImplementation.FAST_RTPS);
   }

   public static void main(String[] args)
   {
      new AtlasBehaviorUIAndModule();
   }
}
