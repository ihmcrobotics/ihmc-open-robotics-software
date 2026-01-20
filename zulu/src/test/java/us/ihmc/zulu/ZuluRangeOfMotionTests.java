package us.ihmc.zulu;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.AvatarRangeOfMotionTests;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.tools.CITools;

public class ZuluRangeOfMotionTests extends AvatarRangeOfMotionTests
{
   private final DRCRobotModel robotModel = new ZuluRobotModel(ZuluVersion.V1_FULL_ROBOT, RobotTarget.SCS);

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public double getDesiredPelvisHeightAboveFoot()
   {
      return 0.4;
   }

   @Disabled
   @Override
   public void testWalkingOffOfLargePlatform()
   {
      super.testWalkingOffOfLargePlatform();
   }

   @Override
   public String getSimpleRobotName()
   {
      return CITools.getSimpleRobotNameFor(CITools.SimpleRobotNameKeys.ZULU);
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSquattingDown() throws Exception
   {
      super.testSquattingDown();
   }

}
