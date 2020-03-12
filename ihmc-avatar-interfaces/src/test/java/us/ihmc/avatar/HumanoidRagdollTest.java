package us.ihmc.avatar;

import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class HumanoidRagdollTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   public void testZeroTorque(TestInfo testInfo) throws Exception
   {
      DRCRobotModel robotModel = getRobotModel();
      DRCSimulationTestHelper drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel);

      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + "." + testInfo.getTestMethod().get().getName() + "()");
      // Switch to zero-torque controller.
      drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getRequestedControlStateEnum().set(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      drcSimulationTestHelper.simulateAndBlock(1.0);
   }
}
