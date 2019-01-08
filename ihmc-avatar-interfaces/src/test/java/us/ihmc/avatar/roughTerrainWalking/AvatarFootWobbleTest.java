package us.ihmc.avatar.roughTerrainWalking;

import java.util.Random;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class AvatarFootWobbleTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper testHelper;

   private static final double walkDistance = 1.0;
   private static final double stepLenght = 0.1;
   private static final double bumpWidth = 0.01;
   private static final double bumpHeight = 0.01;

   private static final Random random = new Random(20L);

   public void testWobble() throws SimulationExceededMaximumTimeException
   {
      TestEnvironment environment = new TestEnvironment();
      testHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      testHelper.createSimulation(getSimpleRobotName() + "FootWobbleTest");
      testHelper.setupCameraForUnitTest(new Point3D(walkDistance / 2.0, 0.0, 0.2), new Point3D(walkDistance / 2.0, -walkDistance * 3.0, 0.2));
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(0.25));

      FootstepDataListMessage steps = new FootstepDataListMessage();
      double duration = environment.packSteps(steps, getRobotModel().getWalkingControllerParameters());

      testHelper.publishToController(steps);
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(duration + 0.25));
   }

   private class TestEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
      private static final double padding = 0.5;

      public TestEnvironment()
      {
         // Ground plane that allows the robot to walk forward in x direction
         terrain.addBox(-padding, -padding, walkDistance + padding, padding, -0.05, 0.0);

         // Add little bumps on the ground that the robot steps on.
         double x = stepLenght;
         while (x < walkDistance - stepLenght)
         {
            double location = x + 2.0 * (random.nextDouble() - 0.5) * 2.0 * bumpWidth; // adjust location by up to twice the nominal width
            double width = bumpWidth + 2.0 * random.nextDouble() * bumpWidth; // adjust width by growing it randomly up to three times the nominal value
            terrain.addBox(location - width / 2.0, -padding, location + width / 2.0, padding, 0.0, bumpHeight);
            x += stepLenght;
         }
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrain;
      }

      public double packSteps(FootstepDataListMessage steps, WalkingControllerParameters walkingControllerParameters)
      {
         double duration = walkingControllerParameters.getDefaultInitialTransferTime() - walkingControllerParameters.getDefaultTransferTime();
         double x = stepLenght;
         double y = walkingControllerParameters.getSteppingParameters().getInPlaceWidth() / 2.0;
         RobotSide side = RobotSide.LEFT;
         while (x < walkDistance - stepLenght)
         {
            FootstepDataMessage step = steps.getFootstepDataList().add();
            step.setRobotSide(side.toByte());
            step.getLocation().set(x, side.negateIfRightSide(y), 0.0);
            side = side.getOppositeSide();
            x += stepLenght;
            duration += walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
         }
         for (int i = 0; i < 2; i++)
         {
            FootstepDataMessage step = steps.getFootstepDataList().add();
            step.setRobotSide(side.toByte());
            step.getLocation().set(walkDistance, side.negateIfRightSide(y), 0.0);
            side = side.getOppositeSide();
            duration += walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();
         }
         duration += walkingControllerParameters.getDefaultFinalTransferTime();
         return duration;
      }
   }

   @Before
   public void showMemoryUsageBeforeTest()
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (testHelper != null)
      {
         testHelper.destroySimulation();
         testHelper = null;
      }

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
      simulationTestingParameters = null;
   }
}
