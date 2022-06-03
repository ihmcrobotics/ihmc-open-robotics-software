package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.HumanoidHandDesiredConfigurationBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasHandDesiredConfigurationBehaviorTest extends HumanoidHandDesiredConfigurationBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHandDesiredConfigurationBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testCloseHand()
   {
      super.testCloseHand();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testPauseAndResumeCloseHand()
   {
      super.testPauseAndResumeCloseHand();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testStopCloseHand()
   {
      super.testStopCloseHand();
   }
}
