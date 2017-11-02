package us.ihmc.atlas.behaviorTests;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.behaviorTests.DRCHeadTrajectoryBehaviorTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.FAST})
public class AtlasHeadTrajectoryBehaviorTest extends DRCHeadTrajectoryBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHeadTrajectoryBehaviorTest()
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
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 37.6)
   @Test(timeout = 60000)
   public void testHeadPitch() throws SimulationExceededMaximumTimeException
   {
      super.testHeadPitch();
   }

   @Override
   //@ContinuousIntegrationTest(estimatedDuration = 37.6)
   //@Test(timeout = 950000)
   public void testHeadRoll() throws SimulationExceededMaximumTimeException
   {
      super.testHeadRoll();
   }

   @Override
   //@ContinuousIntegrationTest(estimatedDuration = 37.6)
   //@Test(timeout = 950000)
   public void testHeadYaw() throws SimulationExceededMaximumTimeException
   {
      super.testHeadYaw();
   }

   @Override
   //@ContinuousIntegrationTest(estimatedDuration = 37.6)
   //@Test(timeout = 950000)
   public void testRandomOrientation() throws SimulationExceededMaximumTimeException
   {
      super.testRandomOrientation();
   }

}
