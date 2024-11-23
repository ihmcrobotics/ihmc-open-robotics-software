package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarPushRecoveryWalkingTrackTest;

public class AtlasPushRecoveryWalkingTrackTest extends AvatarPushRecoveryWalkingTrackTest
{
   private DRCRobotModel robotModel;

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public double getForwardPushDeltaV()
   {
      return 0.5;
   }

   @Override
   public double getOutwardPushDeltaV()
   {
      return 0.45;
   }

   @Override
   public double getBackwardPushDeltaV()
   {
      return 0.3;
   }

   @Override
   public double getInwardPushDeltaV()
   {
      return 0.2;
   }

   @Override
   public double getForwardPushInTransferDeltaV()
   {
      return 0.25;
   }

   @Override
   public double getOutwardPushInTransferDeltaV()
   {
      return 0.25;
   }

   @Override
   public double getBackwardPushInTransferDeltaV()
   {
      return 0.15;
   }

   @Override
   public double getInwardPushInTransferDeltaV()
   {
      return 0.08;
   }

   @Override
   @Test
   public void testFlatGroundWalking()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      super.testFlatGroundWalking();
   }
}
