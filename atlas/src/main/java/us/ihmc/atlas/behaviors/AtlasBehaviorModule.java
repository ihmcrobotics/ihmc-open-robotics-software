package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;

public class AtlasBehaviorModule
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean START_FOOTSTEP_PLANNING_TOOLBOX = false;

   private MultiStageFootstepPlanningModule multiStageFootstepPlanningModule;

   public AtlasBehaviorModule()
   {
      if (START_FOOTSTEP_PLANNING_TOOLBOX)
      {
         ThreadTools.startAsDaemon(() ->
         {
            LogTools.info("Creating footstep toolbox");
            multiStageFootstepPlanningModule = new MultiStageFootstepPlanningModule(createRobotModel(),
                                                                                    null,
                                                                                    false,
                                                                                    DomainFactory.PubSubImplementation.FAST_RTPS);
         }, "MultiStageFootstepPlanningModule");
      }

      LogTools.info("Creating behavior module");
      BehaviorModule.createForBackpack(createRobotModel());

      Runtime.getRuntime().addShutdownHook(ThreadTools.startAThread(() ->
      { // add cleanup actions here
         multiStageFootstepPlanningModule.destroy();
      }, "Cleanup"));
   }

   private AtlasRobotModel createRobotModel()
   {
      return new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);
   }

   /** To run remotely */
   public static void main(String[] args)
   {
      new AtlasBehaviorModule();
   }
}
