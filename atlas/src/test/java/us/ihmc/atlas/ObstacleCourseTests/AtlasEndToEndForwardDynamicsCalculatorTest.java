package us.ihmc.atlas.ObstacleCourseTests;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.AvatarEndToEndForwardDynamicsCalculatorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
public class AtlasEndToEndForwardDynamicsCalculatorTest extends AvatarEndToEndForwardDynamicsCalculatorTest
{
   private final AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

   @Override
   @Test
   public void testStanding() throws Exception
   {
      super.testStanding();
   }

   @Override
   @Test
   public void testFloating() throws Exception
   {
      super.testFloating();
   }

   @Override
   @Test
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
