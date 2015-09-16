package us.ihmc.atlas.behaviorTests;

import org.junit.Test;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCHandPoseListBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanTarget;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;

@DeployableTestClass(targets = {TestPlanTarget.Slow, TestPlanTarget.Flaky})
public class AtlasHandPoseListBehaviorTest extends DRCHandPoseListBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHandPoseListBehaviorTest()
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, DRCRobotModel.RobotTarget.SCS, false);
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
   @DeployableTestMethod(estimatedDuration = 41.1)
   @Test(timeout = 210000)
   public void testMoveOneRandomJoint20Deg() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testMoveOneRandomJoint20Deg();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 63.1)
   @Test(timeout = 320000)
   public void testMultipleArmPosesOnBothArms() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testMultipleArmPosesOnBothArms();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 75.3)
   @Test(timeout = 380000)
   public void testRandomJointSpaceMoveAndTaskSpaceMoveBackToHome() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Flaky);
      super.testRandomJointSpaceMoveAndTaskSpaceMoveBackToHome();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 41.1)
   @Test(timeout = 210000)
   public void testWristRoll() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testWristRoll();
   }
}
