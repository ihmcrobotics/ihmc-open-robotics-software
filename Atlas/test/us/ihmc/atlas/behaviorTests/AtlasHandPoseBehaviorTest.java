package us.ihmc.atlas.behaviorTests;

import java.io.FileNotFoundException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCHandPoseBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;
import us.ihmc.tools.testing.TestPlanTarget;

@DeployableTestClass(targets = {TestPlanTarget.InDevelopment, TestPlanTarget.Slow})
public class AtlasHandPoseBehaviorTest extends DRCHandPoseBehaviorTest
{
   private final AtlasRobotModel robotModel;

   public AtlasHandPoseBehaviorTest()
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
   @DeployableTestMethod(estimatedDuration = 31.1)
   @Test(timeout = 160000)
   public void testAimPalmNormalXaxis() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testAimPalmNormalXaxis();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 28.1)
   @Test(timeout = 140000)
   public void testAimPalmNormalXaxisZupGrasp() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testAimPalmNormalXaxisZupGrasp();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 26.1)
   @Test(timeout = 130000)
   public void testAimPalmNormalYaxis() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testAimPalmNormalYaxis();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 26.6)
   @Test(timeout = 130000)
   public void testAimPalmNormalYaxisZupGrasp() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testAimPalmNormalYaxisZupGrasp();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 40.7)
   @Test(timeout = 200000)
   public void testJointSpaceHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testJointSpaceHandPoseMove();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 42.2)
   @Test(timeout = 210000)
   public void testMoveHandToHome() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testMoveHandToHome();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 56.8)
   @Test(timeout = 280000)
   public void testPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testPauseAndResume();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 53.6)
   @Test(timeout = 270000)
   public void testStop() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testStop();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 57.8)
   @Test(timeout = 290000)
   public void testTaskSpaceMoveToPoseAchievedInJointSpace() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.InDevelopment);
      super.testTaskSpaceMoveToPoseAchievedInJointSpace();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 51.2)
   @Test(timeout = 260000)
   public void testTwoSequentialHandPoses() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testTwoSequentialHandPoses();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 39.1)
   @Test(timeout = 200000)
   public void testTwoSimultaneousHandPoseBehaviors() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testTwoSimultaneousHandPoseBehaviors();
   }

   @Override
   @DeployableTestMethod(estimatedDuration = 30.0)
   @Test(timeout = 150000)
   public void testUnreachableHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      TestPlanTarget.assumeRunningOnPlanIfRunningOnBamboo(TestPlanTarget.Slow);
      super.testUnreachableHandPoseMove();
   }
}
