package us.ihmc.valkyrie.behaviorTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.behaviorTests.HumanoidHandDesiredConfigurationBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieHandDesiredConfigurationBehaviorTest extends HumanoidHandDesiredConfigurationBehaviorTest
{
   private final ValkyrieRobotModel robotModel = new ValkyrieRobotModel(RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.VALKYRIE);
   }

   @Tag("fast")
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

   @Override
   protected double getFingerClosedJointAngleSign(RobotSide robotSide)
   {
      return robotSide.negateIfLeftSide(1.0);
   }
}