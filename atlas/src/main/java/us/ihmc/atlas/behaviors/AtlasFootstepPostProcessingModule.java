package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingModule;
import us.ihmc.avatar.networkProcessor.footstepPlanPostProcessingModule.FootstepPlanPostProcessingModuleLauncher;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class AtlasFootstepPostProcessingModule
{
   public static final AtlasRobotVersion ATLAS_VERSION = AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS;
   private static final RobotTarget ATLAS_TARGET = RobotTarget.REAL_ROBOT;

   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(ATLAS_VERSION, ATLAS_TARGET, false);
      FootstepPlanPostProcessingModuleLauncher.createModule(robotModel, PubSubImplementation.FAST_RTPS);
   }
}
