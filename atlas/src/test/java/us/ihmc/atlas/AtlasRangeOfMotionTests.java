package us.ihmc.atlas;

import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.AvatarRangeOfMotionTests;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

public class AtlasRangeOfMotionTests extends AvatarRangeOfMotionTests
{
   private final DRCRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

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

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testSquattingDown() throws Exception
   {
      super.testSquattingDown();
   }

   @Tag("controller-api-2")
   @Override
   @Test
   public void testWalkingOffOfLargePlatform() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(false);
      super.testWalkingOffOfLargePlatform();
   }

   @Tag("controller-api-2")
   @Test
   public void testWalkingOffOfLargePlatformWithExperimentalPhysicsEngine() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      setUseExperimentalPhysicsEngine(true);
      super.testWalkingOffOfLargePlatform();
   }

}
