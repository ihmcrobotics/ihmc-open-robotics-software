package us.ihmc.valkyrie.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.avatar.behaviorTests.DRCWalkToLocationBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;

@Tag("humanoid-behaviors-slow")
public class ValkyrieWalkToLocationBehaviorTest extends DRCWalkToLocationBehaviorTest
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

   @Override
   @Test
   public void testTurn361DegreesInPlace()
   {
      super.testTurn361DegreesInPlace();
   }

   @Override
   @Test
   public void testWalkAndStopBehavior()
   {
      super.testWalkAndStopBehavior();
   }

   @Override
   @Test
   public void testWalkAtAngleAndFinishAlignedWithInitialOrientation()
   {
      super.testWalkAtAngleAndFinishAlignedWithInitialOrientation();
   }

   @Override
   @Test
   public void testWalkAtAngleAndFinishAlignedWithWalkingPath()
   {
      super.testWalkAtAngleAndFinishAlignedWithWalkingPath();
   }

   @Override
   @Test
   public void testWalkAtAngleUsingStartOrientation()
   {
      super.testWalkAtAngleUsingStartOrientation();
   }

   @Override
   @Test
   public void testWalkAtAngleUsingStartTargetMeanOrientation()
   {
      super.testWalkAtAngleUsingStartTargetMeanOrientation();
   }

   @Override
   @Test
   public void testWalkAtAngleUsingTargetOrientation()
   {
      super.testWalkAtAngleUsingTargetOrientation();
   }

   @Override
   @Test
   public void testWalkBackwardsASmallAmountWithoutTurningInPlace()
   {
      super.testWalkBackwardsASmallAmountWithoutTurningInPlace();
   }

   @Override
   @Test
   public void testWalkForwardsX()
   {
      super.testWalkForwardsX();
   }

   @Override
   @Disabled
   @Test
   public void testWalkPauseAndResumeBehavior()
   {
      super.testWalkPauseAndResumeBehavior();
   }

   @Override
   @Disabled
   @Test
   public void testWalkPauseAndResumeOnLastStepBehavior()
   {
      super.testWalkPauseAndResumeOnLastStepBehavior();
   }

   @Override
   @Disabled
   @Test
   public void testWalkStopAndWalkToDifferentLocation()
   {
      super.testWalkStopAndWalkToDifferentLocation();
   }
}
