package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.FootMotionOnTouchdownSlipper;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import java.util.ArrayList;

import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarFeetErrorTranslationTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;


   protected int getNumberOfSteps()
   {
      return 10;
   }

   protected double getStepLength()
   {
      return 0.25;
   }

   protected double getStepWidth()
   {
      return 0.08;
   }



   protected FootstepDataListMessage getFootstepDataListMessage()
   {
      return new FootstepDataListMessage();
   }

   @Test
   public void testForwardWalk() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      PrintTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(className);
      robotModel = getRobotModel();

      createFootSlipper(drcSimulationTestHelper.getSimulationConstructionSet(), drcSimulationTestHelper.getRobot());

      int numberOfSteps = getNumberOfSteps();
      double stepLength = getStepLength();
      double stepWidth = getStepWidth();

      ThreadTools.sleep(1000);

      setupCameraSideView();

      RobotSide side = RobotSide.LEFT;

      FootstepDataListMessage footMessage = getFootstepDataListMessage();
      ArrayList<Point3D> rootLocations = new ArrayList<>();

      PelvisCheckpointChecker controllerSpy = new PelvisCheckpointChecker(drcSimulationTestHelper);

      for (int currentStep = 0; currentStep < numberOfSteps; currentStep++)
      {
         if (drcSimulationTestHelper.getQueuedControllerCommands().isEmpty())
         {
            Point3D footLocation = new Point3D(stepLength * currentStep, side.negateIfRightSide(stepWidth / 2), 0.0);
            rootLocations.add(new Point3D(stepLength * currentStep, 0.0, 0.0));
            Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
            addFootstep(footLocation, footOrientation, side, footMessage);
            side = side.getOppositeSide();
         }
      }
      Point3D footLocation = new Point3D(stepLength * (numberOfSteps - 1), side.negateIfRightSide(stepWidth / 2), 0.0);
      rootLocations.add(new Point3D(stepLength * (numberOfSteps - 1), 0.0, 0.0));
      Quaternion footOrientation = new Quaternion(0.0, 0.0, 0.0, 1.0);
      addFootstep(footLocation, footOrientation, side, footMessage);

      double intitialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      int steps = footMessage.getFootstepDataList().size();

      controllerSpy.setFootStepCheckPoints(rootLocations, getStepLength(), getStepWidth());
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
      drcSimulationTestHelper.publishToController(footMessage);
      double simulationTime = intitialTransfer + (transfer + swing) * steps + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));
      controllerSpy.assertCheckpointsReached();
   }

   private static void createFootSlipper(SimulationConstructionSet simulationConstructionSet, HumanoidFloatingRootJointRobot robot)
   {
      int ticksPerPerturbation = 10;
      FootMotionOnTouchdownSlipper oscillateFeetPerturber = new FootMotionOnTouchdownSlipper(robot, simulationConstructionSet.getDT());
      oscillateFeetPerturber.setSlipDuration(0.1);
      oscillateFeetPerturber.setTranslationMagnitudes(new double[] { 0.0, 0.0, -0.05});
      oscillateFeetPerturber.setRotationMagnitudesYawPitchRoll(new double[] { 0.0, 0.0, 0.0 });


      robot.setController(oscillateFeetPerturber, ticksPerPerturbation);
   }

   private void addFootstep(Point3D stepLocation, Quaternion orient, RobotSide robotSide, FootstepDataListMessage message)
   {
      FootstepDataMessage footstepData = new FootstepDataMessage();
      footstepData.getLocation().set(stepLocation);
      footstepData.getOrientation().set(orient);
      footstepData.setRobotSide(robotSide.toByte());
      message.getFootstepDataList().add().set(footstepData);
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters.setKeepSCSUp(true);
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      if (simulationTestingParameters.getKeepSCSUp())
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }
}
