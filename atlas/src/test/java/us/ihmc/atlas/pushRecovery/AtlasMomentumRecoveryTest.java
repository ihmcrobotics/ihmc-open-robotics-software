package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.HumanoidMomentumRecoveryTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;

public class AtlasMomentumRecoveryTest extends HumanoidMomentumRecoveryTest
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
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Test
   @Override
   public void testPushDuringDoubleSupport()
   {
      super.testPushDuringDoubleSupport();
   }

   @Test
   @Override
   public void testPushDuringDoubleSupportExpectFall()
   {
      super.testPushDuringDoubleSupportExpectFall();
   }

   @Test
   @Override
   public void testPushDuringSwingExpectFall()
   {
      super.testPushDuringSwingExpectFall();
   }

   @Test
   @Override
   public void testRegularWalk()
   {
      super.testRegularWalk();
   }

   @Test
   @Override
   public void testPushDuringSwing()
   {
      super.testPushDuringSwing();
   }
}
