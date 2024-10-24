package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.WholeBodyInverseKinematicsBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;

@Disabled
@Deprecated
public class AtlasWholeBodyInverseKinematicsBehaviorTest extends WholeBodyInverseKinematicsBehaviorTest
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return robotModel.getSimpleRobotName();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForAHandPose()
   {
      super.testSolvingForAHandPose();
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testSolvingForBothHandPoses()
   {
      super.testSolvingForBothHandPoses();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForChestAngularControl()
   {
      super.testSolvingForChestAngularControl();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForHandAngularLinearControl()
   {
      super.testSolvingForHandAngularLinearControl();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForHandRollConstraint()
   {
      super.testSolvingForHandRollConstraint();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForHandSelectionMatrix()
   {
      super.testSolvingForHandSelectionMatrix();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testSolvingForPelvisAngularControl()
   {
      super.testSolvingForPelvisAngularControl();
   }
}
