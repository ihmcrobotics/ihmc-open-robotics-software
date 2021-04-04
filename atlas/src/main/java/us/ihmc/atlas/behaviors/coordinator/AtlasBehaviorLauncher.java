package us.ihmc.atlas.behaviors.coordinator;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.BehaviorRegistry;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.javafx.JavaFXBehaviorUIRegistry;
import us.ihmc.behaviors.javafx.behaviors.LookAndStepBehaviorUI;

public class AtlasBehaviorLauncher
{
   public static void main(String[] args)
   {
      DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT, false);
      BehaviorRegistry behaviorRegistry = JavaFXBehaviorUIRegistry.of(LookAndStepBehaviorUI.DEFINITION, TraverseStairsBehavior.DEFINITION);
      BehaviorModule.createInterprocess(behaviorRegistry, robotModel);
   }
}
