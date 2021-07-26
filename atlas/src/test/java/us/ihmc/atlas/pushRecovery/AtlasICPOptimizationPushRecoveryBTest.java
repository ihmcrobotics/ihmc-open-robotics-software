package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarICPOptimizationPushRecoveryBTest;

public class AtlasICPOptimizationPushRecoveryBTest extends AvatarICPOptimizationPushRecoveryBTest
{
   @Override
   protected DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   protected double getSizeScale()
   {
      return 1.0;
   }

   @Override
   public double getNominalHeight()
   {
      return 0.9;
   }

   @Override
   public double getSlowSwingDuration()
   {
      return 1.2;
   }

   @Override
   public double getSlowTransferDuration()
   {
      return 0.8;
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationDiagonalOutwardPushInSwing() throws Exception
   {
      percentWeight = 0.2;
      super.testPushICPOptimizationDiagonalOutwardPushInSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationDiagonalYawingOutwardPushInSwing() throws Exception
   {
      percentWeight = 0.2; // Used to be 0.125 before PR #1326
      super.testPushICPOptimizationDiagonalYawingOutwardPushInSwing();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationLongBackwardPushInSwing() throws Exception
   {
      percentWeight = 0.10;
      super.testPushICPOptimizationLongBackwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushICPOptimizationLongForwardPushInSwing() throws Exception
   {
      percentWeight = 0.15;
      super.testPushICPOptimizationLongForwardPushInSwing();
   }

   @Disabled // This test is covered by flat ground track and so many other tests that it is not really useful anymore.
   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushICPOptimizationNoPush() throws Exception
   {
      super.testPushICPOptimizationNoPush();
   }

   @Tag("humanoid-push-recovery-slow")
   @Override
   @Test
   public void testPushICPOptimizationOutwardPushInSlowSwing() throws Exception
   {
      percentWeight = 0.4;
      super.testPushICPOptimizationOutwardPushInSlowSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushICPOptimizationOutwardPushInSwing() throws Exception
   {
      percentWeight = 0.4;
      super.testPushICPOptimizationOutwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testPushICPOptimizationRandomPushInSwing() throws Exception
   {
      percentWeight = 0.18;
      super.testPushICPOptimizationRandomPushInSwing();
   }

   public static void main(String[] args)
   {
      AtlasICPOptimizationPushRecoveryBTest test = new AtlasICPOptimizationPushRecoveryBTest();
      try
      {
         test.testPushICPOptimizationNoPush();
      }
      catch (Exception e)
      {

      }
   }
}
