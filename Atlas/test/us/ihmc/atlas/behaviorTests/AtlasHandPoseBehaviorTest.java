package us.ihmc.atlas.behaviorTests;

import java.io.FileNotFoundException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCHandPoseBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;


@DeployableTestClass(planType = {BambooPlanType.InDevelopment, BambooPlanType.Slow})
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
	@DeployableTestMethod(duration = 26.2)
   @Test(timeout = 130000)
   public void testAimPalmNormalXaxis() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testAimPalmNormalXaxis();
   }
   
   @Override
	@DeployableTestMethod(duration = 28.1)
   @Test(timeout = 140000)
   public void testAimPalmNormalXaxisZupGrasp() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testAimPalmNormalXaxisZupGrasp();
   }
   
   @Override
	@DeployableTestMethod(duration = 26.1)
   @Test(timeout = 130000)
   public void testAimPalmNormalYaxis() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testAimPalmNormalYaxis();
   }
   
   @Override
	@DeployableTestMethod(duration = 26.6)
   @Test(timeout = 130000)
   public void testAimPalmNormalYaxisZupGrasp() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testAimPalmNormalYaxisZupGrasp();
   }
   
   @Override
	@DeployableTestMethod(duration = 33.2)
   @Test(timeout = 170000)
   public void testJointSpaceHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testJointSpaceHandPoseMove();
   }
   
   @Override
	@DeployableTestMethod(duration = 42.2)
   @Test(timeout = 210000)
   public void testMoveHandToHome() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testMoveHandToHome();
   }
   
   @Override
	@DeployableTestMethod(duration = 43.4)
   @Test(timeout = 220000)
   public void testPauseAndResume() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testPauseAndResume();
   }
   
   @Override
	@DeployableTestMethod(duration = 34.1)
   @Test(timeout = 170000)
   public void testStop() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testStop();
   }
   
   @Override
	@DeployableTestMethod(duration = 57.8)
   @Test(timeout = 290000)
   public void testTaskSpaceMoveToPoseAchievedInJointSpace() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.InDevelopment);
      super.testTaskSpaceMoveToPoseAchievedInJointSpace();
   }
   
   @Override
	@DeployableTestMethod(duration = 35.2)
   @Test(timeout = 180000)
   public void testTwoSequentialHandPoses() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testTwoSequentialHandPoses();
   }
   
   @Override
	@DeployableTestMethod(duration = 35.9)
   @Test(timeout = 180000)
   public void testTwoSimultaneousHandPoseBehaviors() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testTwoSimultaneousHandPoseBehaviors();
   }
   
   @Override
	@DeployableTestMethod(duration = 27.6)
   @Test(timeout = 140000)
   public void testUnreachableHandPoseMove() throws SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testUnreachableHandPoseMove();
   }
}
