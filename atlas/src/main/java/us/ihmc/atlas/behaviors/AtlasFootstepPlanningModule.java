package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;

public class AtlasFootstepPlanningModule
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.REAL_ROBOT;

   public AtlasFootstepPlanningModule()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);
      FootstepPlanningModule module = FootstepPlanningModuleLauncher.createModule(robotModel, DomainFactory.PubSubImplementation.FAST_RTPS);
      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         LogTools.info("Shutting down network processor modules.");
         module.closeAndDispose();
         ThreadTools.sleep(10);
      }, getClass().getSimpleName() + "Shutdown"));
      ThreadTools.sleepForever();
   }

   public static void main(String[] args)
   {
      new AtlasFootstepPlanningModule();
   }
}
