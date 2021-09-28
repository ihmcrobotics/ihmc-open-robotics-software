package us.ihmc.avatar.angularMomentumTest;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.ArrayList;
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
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.FootstepDataListCommand;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.lists.PairList;
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

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      PrintTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setTestEnvironment(flatGround);
      drcSimulationTestHelper.createSimulation(className);
      robotModel = getRobotModel();

      ThreadTools.sleep(1000);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0);
   }

   @Test
   public void testMoveInPlace() throws SimulationExceededMaximumTimeException
   {
      // only set to true when saving new angular momentum data. output file usually needs to be manually moved to resources folder
      setupTest();
      setupCameraSideView();

      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1.5, 0.9));

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      ((YoBoolean) drcSimulationTestHelper.getYoVariable("zeroAngularExcursionFlag")).set(true);

      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1.5, 0.7));
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      drcSimulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1.5, 0.9));
      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);


      YoDouble angularExcursionYaw = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionYaw");
      YoDouble angularExcursionPitch = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionPitch");
      YoDouble angularExcursionRoll = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionRoll");

      assertEquals(0.0, angularExcursionYaw.getDoubleValue(), 5e-3);
      assertEquals(0.0, angularExcursionPitch.getDoubleValue(), 5e-3);
      assertEquals(0.0, angularExcursionRoll.getDoubleValue(), 5e-3);
   }

   @Test
   public void testForwardWalk() throws SimulationExceededMaximumTimeException
   {
      // only set to true when saving new angular momentum data. output file usually needs to be manually moved to resources folder
      setupTest();
      setupCameraSideView();

      ((YoBoolean) drcSimulationTestHelper.getYoVariable("zeroAngularExcursionFlag")).set(true);

      YoDouble angularExcursionYaw = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionYaw");
      YoDouble angularExcursionPitch = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionPitch");
      YoDouble angularExcursionRoll = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionRoll");

      RobotSide side = RobotSide.LEFT;

      double stepLength = getStepLength();
      double stepWidth = getStepWidth();

      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      double stepX = 0.0;
      for (int currentStep = 0; currentStep < 4; currentStep++)
      {
         stepX += stepLength;
         FramePose3D stepPose = new FramePose3D(ReferenceFrame.getWorldFrame());
         stepPose.getPosition().set(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
         stepPose.changeFrame(ReferenceFrame.getWorldFrame());
         footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, stepPose.getPosition(), stepPose.getOrientation()));

         side = side.getOppositeSide();
      }

      FramePose3D closingStepPose = new FramePose3D(ReferenceFrame.getWorldFrame());
      closingStepPose.getPosition().set(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
      closingStepPose.changeFrame(ReferenceFrame.getWorldFrame());
      footsteps.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, closingStepPose.getPosition(), closingStepPose.getOrientation()));

      drcSimulationTestHelper.publishToController(footsteps);


      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      double simulationTime = initialTransfer + (transfer + swing) * (footsteps.getFootstepDataList().size() + 1) + 1.0;
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

      assertEquals(0.0, angularExcursionYaw.getDoubleValue(), 1e-2);
      assertEquals(0.0, angularExcursionRoll.getDoubleValue(), 1e-2);
      assertEquals(0.0, angularExcursionPitch.getDoubleValue(), 1e-1);
   }

   @Test
   public void testWalkInASquare() throws SimulationExceededMaximumTimeException
   {
      // only set to true when saving new angular momentum data. output file usually needs to be manually moved to resources folder
      setupTest();
      setupCameraSideView();

      ((YoBoolean) drcSimulationTestHelper.getYoVariable("zeroAngularExcursionFlag")).set(true);

      drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.1);

      YoDouble angularExcursionYaw = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionYaw");
      YoDouble angularExcursionPitch = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionPitch");
      YoDouble angularExcursionRoll = (YoDouble) drcSimulationTestHelper.getYoVariable("angularExcursionRoll");

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      PairList<FootstepDataListMessage, FootstepDataListMessage> footstepsSides = createSquareFootstepMessage(numberOfSteps);
      for (int i = 0; i < footstepsSides.size(); i++)
      {
         FootstepDataListMessage sideFootsteps = footstepsSides.get(i).getLeft();
         drcSimulationTestHelper.publishToController(sideFootsteps);

         double simulationTime = initialTransfer + (transfer + swing) * (sideFootsteps.getFootstepDataList().size() + 1) + 1.0;
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

//         assertEquals("Roll excursion is too great.", 0.0, angularExcursionRoll.getDoubleValue(), 1e-2);
//         assertEquals("Pitch excursion is too great. ", 0.0, angularExcursionPitch.getDoubleValue(), 1e-3);

         FootstepDataListMessage turnFootsteps = footstepsSides.get(i).getRight();
         drcSimulationTestHelper.publishToController(turnFootsteps);

         simulationTime = initialTransfer + (transfer + swing) * (turnFootsteps.getFootstepDataList().size() + 1) + 1.0;
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(simulationTime));

//         assertEquals(0.0, angularExcursionPitch.getDoubleValue(), 1e-3);
//         assertEquals(0.0, angularExcursionRoll.getDoubleValue(), 1e-3);
      }

      assertEquals(0.0, angularExcursionYaw.getDoubleValue(), 1e-3);
   }


   private PairList<FootstepDataListMessage, FootstepDataListMessage> createSquareFootstepMessage(int stepsOnASide)
   {
      double stepLength = getStepLength();
      double stepWidth = getStepWidth();
      PairList<FootstepDataListMessage, FootstepDataListMessage> stepSides = new PairList<>();

      PoseReferenceFrame planningFrame = new PoseReferenceFrame("planningFrame", ReferenceFrame.getWorldFrame());
      PoseReferenceFrame turningFrame = new PoseReferenceFrame("turningFrame", planningFrame);

      for (int i = 0; i < 4; i++)
      {
         RobotSide side = RobotSide.LEFT;

         FootstepDataListMessage sideMessage = new FootstepDataListMessage();
         FootstepDataListMessage turnMessage = new FootstepDataListMessage();

         double stepX = 0.0;
         for (int currentStep = 0; currentStep < stepsOnASide; currentStep++)
         {
            stepX += stepLength;
            FramePose3D stepPose = new FramePose3D(planningFrame);
            stepPose.getPosition().set(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
            stepPose.changeFrame(ReferenceFrame.getWorldFrame());
            sideMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, stepPose.getPosition(), stepPose.getOrientation()));

            side = side.getOppositeSide();
         }

         FramePose3D closingStepPose = new FramePose3D(planningFrame);
         closingStepPose.getPosition().set(stepX, side.negateIfRightSide(stepWidth / 2), 0.0);
         closingStepPose.changeFrame(ReferenceFrame.getWorldFrame());
         sideMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, closingStepPose.getPosition(), closingStepPose.getOrientation()));

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

            turnMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, stepPose.getPosition(), stepPose.getOrientation()));

            side = side.getOppositeSide();
         }

         FramePose3D turnedPose = new FramePose3D(planningFrame);
         turnedPose.getOrientation().appendYawRotation(Math.toRadians(90));

         turningFrame.setPoseAndUpdate(turnedPose);

         FramePose3D stepPose = new FramePose3D(turningFrame);
         stepPose.setToZero(turningFrame);
         stepPose.setY(side.negateIfRightSide(stepWidth / 2));
         stepPose.changeFrame(ReferenceFrame.getWorldFrame());

         turnMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(side, stepPose.getPosition(), stepPose.getOrientation()));

         turnedPose.changeFrame(ReferenceFrame.getWorldFrame());
         planningFrame.setPoseAndUpdate(turnedPose);

         stepSides.add(sideMessage, turnMessage);
      }

      return stepSides;
   }

   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

}
