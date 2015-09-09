package us.ihmc.atlas.behaviorTests;

import java.io.FileNotFoundException;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.darpaRoboticsChallenge.behaviorTests.DRCScriptBehaviorTest;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.testing.BambooPlanType;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestClass;
import us.ihmc.tools.testing.TestPlanAnnotations.DeployableTestMethod;


@DeployableTestClass(planType = {BambooPlanType.Flaky, BambooPlanType.Slow})
public class AtlasScriptBehaviorTest extends DRCScriptBehaviorTest
{
   private final AtlasRobotModel robotModel;
   
   public AtlasScriptBehaviorTest()
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
   @DeployableTestMethod(duration = 50.0)
   @Test(timeout = 300000)
   public void testPauseAndResumeScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Flaky);
      super.testPauseAndResumeScript();
   }
   
   @Override
   @DeployableTestMethod(duration = 50.0)
   @Test(timeout = 300000)
   public void testScriptWithOneHandPosePacket() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Flaky);
      super.testScriptWithOneHandPosePacket();
   }
   
   @Override
	@DeployableTestMethod(duration = 46.5)
   @Test(timeout = 230000)
   public void testScriptWithTwoComHeightScriptPackets() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testScriptWithTwoComHeightScriptPackets();
   }
   
   @Override
   @DeployableTestMethod(duration = 50.0)
   @Test(timeout = 300000)
   public void testScriptWithTwoHandPosePackets() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Flaky);
      super.testScriptWithTwoHandPosePackets();
   }
   
   @Override
	@DeployableTestMethod(duration = 26.8)
   @Test(timeout = 130000)
   public void testSimpleScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testSimpleScript();
   }
   
   @Override
	@DeployableTestMethod(duration = 41.1)
   @Test(timeout = 210000)
   public void testStopScript() throws FileNotFoundException, SimulationExceededMaximumTimeException
   {
      BambooPlanType.assumeRunningOnPlanIfRunningOnBamboo(BambooPlanType.Slow);
      super.testStopScript();
   }
}
