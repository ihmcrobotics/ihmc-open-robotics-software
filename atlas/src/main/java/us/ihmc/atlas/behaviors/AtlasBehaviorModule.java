package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningModule;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.tools.FlatGroundPlanarRegionPublisher;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasBehaviorModule
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.SCS;

   public AtlasBehaviorModule()
   {
      new Thread(() -> {
         LogTools.info("Creating footstep toolbox");
         new MultiStageFootstepPlanningModule(createRobotModel(), null, false, DomainFactory.PubSubImplementation.FAST_RTPS);
      }).start();

      new Thread(() -> {
         LogTools.info("Creating flat ground region publisher");
         new FlatGroundPlanarRegionPublisher();
      }).start();

      LogTools.info("Creating behavior module");
      BehaviorModule.createForBackpack(createRobotModel());
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
