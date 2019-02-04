package us.ihmc.avatar.roughTerrainWalking;

import org.junit.After;
import org.junit.Assert;
import org.junit.Before;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class AvatarFootWobbleTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters;
   private DRCSimulationTestHelper testHelper;

   public void testICPReplanning() throws SimulationExceededMaximumTimeException
   {
      testHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), new FlatGroundEnvironment());
      testHelper.createSimulation(getSimpleRobotName() + "FootWobbleTest");
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(0.25));

      // Create step in place:
      RobotSide stepSide = RobotSide.LEFT;
      FramePose3D footPose = new FramePose3D(testHelper.getReferenceFrames().getSoleFrame(stepSide));
      footPose.changeFrame(ReferenceFrame.getWorldFrame());
      FootstepDataListMessage steps = new FootstepDataListMessage();
      FootstepDataMessage step = steps.getFootstepDataList().add();
      step.getLocation().set(footPose.getPosition());
      step.getOrientation().set(footPose.getOrientation());
      step.setRobotSide(stepSide.toByte());
      testHelper.publishToController(steps);

      // Simulate until the swing is half over:
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultInitialTransferTime()));
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getWalkingControllerParameters().getDefaultSwingTime() / 2.0));

      // Trigger a fake rotation detection - the rotation detector is set up so these variables can be changed like this:
      String sidePrefix = stepSide.getOppositeSide().getLowerCaseName();
      testHelper.getYoVariable(sidePrefix + "IsRotating").setValueFromDouble(1.0);
      testHelper.getYoVariable(sidePrefix + "LineOfRotationPointX").setValueFromDouble(0.0);
      testHelper.getYoVariable(sidePrefix + "LineOfRotationPointY").setValueFromDouble(0.0);
      testHelper.getYoVariable(sidePrefix + "LineOfRotationDirectionX").setValueFromDouble(0.0);
      testHelper.getYoVariable(sidePrefix + "LineOfRotationDirectionY").setValueFromDouble(1.0);

      // Simulate some more and then observe the CoP waypoints in the ICP planner. They should have shifted away from the line of rotation.
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(getRobotModel().getControllerDT() * 5.0));
      // TODO: add checks.

      // Assert that the line of rotation has not changed. This would happen if the foot was actually rotating or the rotation detector changes such
      // that the above method for changing the line of rotation does not persist anymore.
      String message = "Line of rotation has changed from what it was set to.";
      Assert.assertEquals(message, 1.0, testHelper.getYoVariable(sidePrefix + "IsRotating").getValueAsDouble(), Double.MIN_VALUE);
      Assert.assertEquals(message, 0.0, testHelper.getYoVariable(sidePrefix + "LineOfRotationPointX").getValueAsDouble(), Double.MIN_VALUE);
      Assert.assertEquals(message, 0.0, testHelper.getYoVariable(sidePrefix + "LineOfRotationPointY").getValueAsDouble(), Double.MIN_VALUE);
      Assert.assertEquals(message, 0.0, testHelper.getYoVariable(sidePrefix + "LineOfRotationDirectionX").getValueAsDouble(), Double.MIN_VALUE);
      Assert.assertEquals(message, 1.0, testHelper.getYoVariable(sidePrefix + "LineOfRotationDirectionY").getValueAsDouble(), Double.MIN_VALUE);
   }

   private static final double stepLenght = 0.15;
   private static final int stepPairs = 1;
   private static final double walkDistance = stepLenght * (1 + stepPairs * 2);
   private static final boolean allowStepAdjustment = false;

   public void testWobble() throws SimulationExceededMaximumTimeException
   {
      TestEnvironment environment = new TestEnvironment();
      testHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel(), environment);
      testHelper.setStartingLocation(environment.getStartingLocation());
      testHelper.createSimulation(getSimpleRobotName() + "FootWobbleTest");
      testHelper.setupCameraForUnitTest(new Point3D(walkDistance / 2.0, 0.0, 0.2), new Point3D(walkDistance / 2.0, -walkDistance * 3.0, 0.2));
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(0.25));

      FootstepDataListMessage steps = new FootstepDataListMessage();
      steps.setAreFootstepsAdjustable(allowStepAdjustment);

      double duration = environment.packSteps(steps, getRobotModel().getWalkingControllerParameters());

      testHelper.publishToController(steps);
      Assert.assertTrue(testHelper.simulateAndBlockAndCatchExceptions(duration + 0.25));
   }

   private class TestEnvironment implements CommonAvatarEnvironmentInterface
   {
      private final CombinedTerrainObject3D terrain = new CombinedTerrainObject3D(getClass().getSimpleName());
      private static final double padding = 0.5;
      private static final double rampHeight = 0.02;

      public TestEnvironment()
      {
         // Start and end platform
         terrain.addBox(-padding, -padding, stepLenght, padding, 0.0, 2.0 * rampHeight);
         terrain.addBox(walkDistance - stepLenght, -padding, walkDistance + padding, padding, 0.0, 2.0 * rampHeight);

         // Add little ramps on the ground that the robot steps on.
         double x = stepLenght;
         while (x < walkDistance - stepLenght)
         {
            terrain.addRamp(x + stepLenght, -padding, x, padding, 2.0 * rampHeight, YoAppearance.LightGray());
            terrain.addRamp(x, -padding, x + stepLenght, padding, 2.0 * rampHeight, YoAppearance.DarkGray());
            terrain.addBox(x + stepLenght, -padding, x + 2.0 * stepLenght, padding, 2.0 * rampHeight);
            x += 2.0 * stepLenght;
         }
      }

      @Override
      public TerrainObject3D getTerrainObject3D()
      {
         return terrain;
      }

      public OffsetAndYawRobotInitialSetup getStartingLocation()
      {
         return new OffsetAndYawRobotInitialSetup(2.0 * rampHeight, new Vector3D(), 0.0);
      }

      public double packSteps(FootstepDataListMessage steps, WalkingControllerParameters walkingControllerParameters)
      {
         double duration = walkingControllerParameters.getDefaultInitialTransferTime() - walkingControllerParameters.getDefaultTransferTime();
         double x = stepLenght;
         double y = walkingControllerParameters.getSteppingParameters().getInPlaceWidth() / 2.0;
         RobotSide side = RobotSide.RIGHT;
         while (x < walkDistance - stepLenght)
         {
            FootstepDataMessage step1 = steps.getFootstepDataList().add();
            step1.setRobotSide(side.toByte());
            step1.getLocation().set(x, side.negateIfRightSide(y), 2.0 * rampHeight);
            side = side.getOppositeSide();
            duration += walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();

            FootstepDataMessage step2 = steps.getFootstepDataList().add();
            step2.setRobotSide(side.toByte());
            step2.getLocation().set(x + stepLenght, side.negateIfRightSide(y), 2.0 * rampHeight);
            side = side.getOppositeSide();
            duration += walkingControllerParameters.getDefaultSwingTime() + walkingControllerParameters.getDefaultTransferTime();

            x += 2.0 * stepLenght;
         }
         for (int i = 0; i < 2; i++)
         {
            FootstepDataMessage step = steps.getFootstepDataList().add();
            step.setRobotSide(side.toByte());
            step.getLocation().set(walkDistance, side.negateIfRightSide(y), 2.0 * rampHeight);
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
