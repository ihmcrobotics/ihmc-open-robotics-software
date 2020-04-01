package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;

public class AtlasBehaviorModule
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean START_FOOTSTEP_PLANNING_TOOLBOX = false;

   private FootstepPlanningModule footstepPlanningModule;

   public AtlasBehaviorModule()
   {
      if (START_FOOTSTEP_PLANNING_TOOLBOX)
      {
         ThreadTools.startAsDaemon(() ->
         {
            LogTools.info("Creating footstep toolbox");
            footstepPlanningModule = FootstepPlanningModuleLauncher.createModule(createRobotModel(), DomainFactory.PubSubImplementation.FAST_RTPS);
         }, "MultiStageFootstepPlanningModule");
      }

      LogTools.info("Creating behavior module");
      BehaviorModule.createForBackpack(createRobotModel());

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      { // add cleanup actions here
         if (START_FOOTSTEP_PLANNING_TOOLBOX)
         {
            footstepPlanningModule.closeAndDispose();
         }
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
