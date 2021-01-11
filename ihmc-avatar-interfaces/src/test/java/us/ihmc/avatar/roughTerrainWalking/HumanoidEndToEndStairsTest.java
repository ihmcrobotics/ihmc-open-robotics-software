package us.ihmc.avatar.roughTerrainWalking;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.planarRegionEnvironments.StaircaseEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class HumanoidEndToEndStairsTest implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters.setKeepSCSUp(true);
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testUpStairsSlow(TestInfo testInfo) throws Exception
   {
      int numberOfSteps = 15;
      double stepHeight = 9.25 * 0.0254;
      double stepLength = 0.32;
      DRCRobotModel robotModel = getRobotModel();
      double actualFootLength = robotModel.getWalkingControllerParameters().getSteppingParameters().getActualFootLength();

      StaircaseEnvironment environment = new StaircaseEnvironment(numberOfSteps, stepHeight, stepLength, false);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + " " + testInfo.getTestMethod().get().getName());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      drcSimulationTestHelper.publishToController(translate(createUpStairsFootsteps(true, stepHeight, stepLength, 0.30, numberOfSteps),
                                                            new Vector3D(0.6 - 0.045 - actualFootLength / 2.0, 0.0, 0.0)));
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(30.0);
   }

   private static FootstepDataListMessage translate(FootstepDataListMessage message, Tuple3DReadOnly translation)
   {
      for (int i = 0; i < message.getFootstepDataList().size(); i++)
      {
         message.getFootstepDataList().get(i).getLocation().add(translation);
      }

      return message;
   }

   private static FootstepDataListMessage createUpStairsFootsteps(boolean slow, double stepHeight, double stepLength, double stanceWidth, int numberOfSteps)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      double x = 0.0;
      double z = 0.0;

      if (slow)
      {
         for (int i = 0; i < numberOfSteps + 1; i++)
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
               footstep.setRobotSide(robotSide.toByte());
               footstep.getLocation().set(x, 0.5 * robotSide.negateIfRightSide(stanceWidth), z);
            }

            x += stepLength;
            z += stepHeight;
         }
      }
      else
      {
         throw new UnsupportedOperationException();
      }

      return footsteps;
   }
}
