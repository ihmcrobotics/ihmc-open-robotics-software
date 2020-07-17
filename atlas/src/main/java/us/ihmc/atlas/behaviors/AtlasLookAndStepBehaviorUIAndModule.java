package us.ihmc.atlas.behaviors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.humanoidBehaviors.BehaviorModule;
import us.ihmc.humanoidBehaviors.ui.BehaviorUI;
import us.ihmc.humanoidBehaviors.ui.BehaviorUIRegistry;
import us.ihmc.humanoidBehaviors.ui.behaviors.LookAndStepBehaviorUI;

public class AtlasLookAndStepBehaviorUIAndModule
{
   public AtlasLookAndStepBehaviorUIAndModule()
   {
      DRCRobotModel drcRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT, false);
      BehaviorUIRegistry behaviorRegistry = BehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION);
      BehaviorModule.createInterprocess(behaviorRegistry, drcRobotModel);
      BehaviorUI.createInterprocess(behaviorRegistry, drcRobotModel, "localhost");
   }

   public static void main(String[] args)
   {
      new AtlasLookAndStepBehaviorUIAndModule();
   }
}
