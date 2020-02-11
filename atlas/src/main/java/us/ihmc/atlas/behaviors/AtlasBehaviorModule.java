package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanningToolboxModule.FootstepPlanningToolboxModule;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.BehaviorRegistry;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;

public class AtlasBehaviorModule
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;
   private static final boolean START_FOOTSTEP_PLANNING_TOOLBOX = false;

   private FootstepPlanningToolboxModule footstepPlanningModule;

   public AtlasBehaviorModule()
   {
      if (START_FOOTSTEP_PLANNING_TOOLBOX) ThreadTools.startAsDaemon(this::footstepPlanningToolbox, "FootstepPlanningToolbox");

      LogTools.info("Creating behavior module");
      BehaviorModule.createInterprocess(BehaviorRegistry.DEFAULT_BEHAVIORS, createRobotModel());

      Runtime.getRuntime().addShutdownHook(ThreadTools.startAThread(this::shutdown, "Cleanup"));
   }

   private void footstepPlanningToolbox()
   {
      LogTools.info("Creating footstep toolbox");
      footstepPlanningModule = new FootstepPlanningToolboxModule(createRobotModel(), null, false, DomainFactory.PubSubImplementation.FAST_RTPS);
   }

   private void shutdown() // add cleanup actions here
   {
      if (START_FOOTSTEP_PLANNING_TOOLBOX) footstepPlanningModule.destroy();
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
