package us.ihmc.avatar.angularMomentumTest;

import static us.ihmc.robotics.Assert.assertTrue;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.Random;

import org.apache.commons.io.IOUtils;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.EuclideanTrajectoryMessage;
import controller_msgs.msg.dds.EuclideanTrajectoryPointMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.MomentumTrajectoryMessage;
import gnu.trove.list.array.TDoubleArrayList;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public abstract class AvatarAngularExcursionTest implements MultiRobotTestInterface
{
   private static final boolean keepSCSUp = false;

   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private DRCSimulationTestHelper drcSimulationTestHelper;
   private DRCRobotModel robotModel;
   private final Random random = new Random(1738);

   // default step parameters for flat ground walking

   protected abstract double getStepLength();

   protected abstract double getStepWidth();

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters.setKeepSCSUp(keepSCSUp && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
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

   private void setupTest()
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      PrintTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(className);
      robotModel = getRobotModel();

      ThreadTools.sleep(1000);
   }

   @Test
   public void testWalkInASquare() throws SimulationExceededMaximumTimeException
   {
      // only set to true when saving new angular momentum data. output file usually needs to be manually moved to resources folder
      boolean exportAchievedAngularMomentum = false;

      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      FootstepDataListMessage footsteps = createSquareFootstepMessage(numberOfSteps);
      drcSimulationTestHelper.publishToController(footsteps);

      double simulationTime = initialTransfer + (transfer + swing) * (footsteps.getFootstepDataList().size() + 1) + 1.0;

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

   }


   private FootstepDataListMessage createSquareFootstepMessage(int stepsOnASide)
   {
      double stepLength = getStepLength();
      double stepWidth = getStepWidth();
      FootstepDataListMessage message = new FootstepDataListMessage();

      PoseReferenceFrame planningFrame = new PoseReferenceFrame("planningFrame", ReferenceFrame.getWorldFrame());
      PoseReferenceFrame turningFrame = new PoseReferenceFrame("turningFrame", planningFrame);

      for (int i = 0; i < 4; i++)
      {
         RobotSide side = RobotSide.LEFT;

         double stepX = 0.0;
         for (int currentStep = 0; currentStep < stepsOnASide; currentStep++)
         {
            stepX += stepLength;
            FramePose3D stepPose = new FramePose3D(planningFrame);
            stepPose.getPosition().set(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
            stepPose.changeFrame(ReferenceFrame.getWorldFrame());
            message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, stepPose.getPosition(), stepPose.getOrientation()));

            side = side.getOppositeSide();
         }

         FramePose3D closingStepPose = new FramePose3D(planningFrame);
         closingStepPose.getPosition().set(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
         closingStepPose.changeFrame(ReferenceFrame.getWorldFrame());
         message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, closingStepPose.getPosition(), closingStepPose.getOrientation()));

         side = side.getOppositeSide();

         FramePose3D newPose = new FramePose3D(planningFrame);
         newPose.setX(stepLength * stepsOnASide);
         newPose.changeFrame(ReferenceFrame.getWorldFrame());
         planningFrame.setPoseAndUpdate(newPose);

         side = RobotSide.LEFT;

         for (int turningStep = 0; turningStep < 3; turningStep++)
         {
            FramePose3D stepPose = new FramePose3D(planningFrame);
            stepPose.getOrientation().appendYawRotation((turningStep + 1) * Math.toRadians(30));

            turningFrame.setPoseAndUpdate(stepPose);
            stepPose.setToZero(turningFrame);
            stepPose.setY(side.negateIfRightSide(stepWidth / 2));
            stepPose.changeFrame(ReferenceFrame.getWorldFrame());

            message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, stepPose.getPosition(), stepPose.getOrientation()));

            side = side.getOppositeSide();
         }

         FramePose3D turnedPose = new FramePose3D(planningFrame);
         turnedPose.getOrientation().appendYawRotation(Math.toRadians(90));

         turningFrame.setPoseAndUpdate(turnedPose);

         FramePose3D stepPose = new FramePose3D(turningFrame);
         stepPose.setToZero(turningFrame);
         stepPose.setY(side.negateIfRightSide(stepWidth / 2));
         stepPose.changeFrame(ReferenceFrame.getWorldFrame());

         message.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, stepPose.getPosition(), stepPose.getOrientation()));

         turnedPose.changeFrame(ReferenceFrame.getWorldFrame());
         planningFrame.setPoseAndUpdate(turnedPose);

      }

      return message;
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

}
