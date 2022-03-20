package us.ihmc.atlas.pushRecovery;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.pushRecovery.AvatarLongPushRecoveryWalkingTest;
import us.ihmc.avatar.pushRecovery.AvatarQuickPushRecoveryWalkingTest;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

public class AtlasLongPushRecoveryWalkingTest extends AvatarLongPushRecoveryWalkingTest
{
   @Override
   public DRCRobotModel getRobotModel()
   {
      return new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   public double getForwardPushDelta()
   {
      return 0.6;
   }

   @Override
   public double getOutwardPushDelta()
   {
      return 0.5;
   }

   @Override
   public double getBackwardPushDelta()
   {
      return 0.6;
   }

   @Override
   public double getInwardPushDelta()
   {
      return 0.3;
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testInwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testInwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testOutwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testOutwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testForwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testForwardPushInSwing();
   }

   @Tag("humanoid-push-recovery")
   @Override
   @Test
   public void testBackwardPushInSwing() throws SimulationExceededMaximumTimeException
   {
      super.testBackwardPushInSwing();
   }
}
