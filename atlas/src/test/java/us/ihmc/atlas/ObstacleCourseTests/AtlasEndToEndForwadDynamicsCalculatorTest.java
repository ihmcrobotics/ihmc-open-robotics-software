package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.AvatarEndToEndForwadDynamicsCalculatorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class AtlasEndToEndForwadDynamicsCalculatorTest extends AvatarEndToEndForwadDynamicsCalculatorTest
{
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 300000)
   public void testStanding() throws Exception
   {
      super.testStanding();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 35.0)
   @Test(timeout = 300000)
   public void testFloating() throws Exception
   {
      super.testFloating();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 300000)
   public void testWalking() throws Exception
   {
      super.testWalking();
   }

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
}
