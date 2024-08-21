package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCWalkToLocationBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AtlasWalkToLocationBehaviorTest extends DRCWalkToLocationBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasWalkToLocationBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testTurn361DegreesInPlace()
   {
      super.testTurn361DegreesInPlace();
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testWalkAndStopBehavior()
   {
      super.testWalkAndStopBehavior();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleAndFinishAlignedWithInitialOrientation()
   {
      super.testWalkAtAngleAndFinishAlignedWithInitialOrientation();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleAndFinishAlignedWithWalkingPath()
   {
      super.testWalkAtAngleAndFinishAlignedWithWalkingPath();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleUsingStartOrientation()
   {
      super.testWalkAtAngleUsingStartOrientation();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleUsingStartTargetMeanOrientation()
   {
      super.testWalkAtAngleUsingStartTargetMeanOrientation();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkAtAngleUsingTargetOrientation()
   {
      super.testWalkAtAngleUsingTargetOrientation();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkBackwardsASmallAmountWithoutTurningInPlace()
   {
      super.testWalkBackwardsASmallAmountWithoutTurningInPlace();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testWalkForwardsX()
   {
      super.testWalkForwardsX();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Disabled
   @Test
   public void testWalkPauseAndResumeBehavior()
   {
      super.testWalkPauseAndResumeBehavior();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Disabled
   @Test
   public void testWalkPauseAndResumeOnLastStepBehavior()
   {
      super.testWalkPauseAndResumeOnLastStepBehavior();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Disabled
   @Test
   public void testWalkStopAndWalkToDifferentLocation()
   {
      super.testWalkStopAndWalkToDifferentLocation();
   }
}
