package us.ihmc.valkyrie.behaviorTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.behaviorTests.KinematicsPlanningBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

public class ValkyrieKinematicsPlanningBehaviorTest extends KinematicsPlanningBehaviorTest
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

   @Tag("humanoid-toolbox")
   @Override
   @Test
   public void testReachToDoorKnob()
   {
      super.testReachToDoorKnob();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSingleKeyFrameInput()
   {
      super.testSingleKeyFrameInput();
   }
}
