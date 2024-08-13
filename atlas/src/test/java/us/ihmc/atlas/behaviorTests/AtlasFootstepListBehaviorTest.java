package us.ihmc.atlas.behaviorTests;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCFootstepListBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class AtlasFootstepListBehaviorTest extends DRCFootstepListBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasFootstepListBehaviorTest()
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
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testSideStepping()
   {
      super.testSideStepping();
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testStepLongerThanMaxStepLength()
   {
      super.testStepLongerThanMaxStepLength();
   }

   @Tag("humanoid-behaviors")
   @Override
   @Test
   public void testStop()
   {
      super.testStop();
   }

   @Tag("humanoid-behaviors-slow")
   @Override
   @Test
   public void testTwoStepsForwards()
   {
      super.testTwoStepsForwards();
   }
}
