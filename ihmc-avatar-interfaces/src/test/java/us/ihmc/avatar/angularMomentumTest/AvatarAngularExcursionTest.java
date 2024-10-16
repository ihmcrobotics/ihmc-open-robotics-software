package us.ihmc.avatar.angularMomentumTest;

import controller_msgs.msg.dds.FootstepDataListMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.commons.referenceFrames.PoseReferenceFrame;
import us.ihmc.commons.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.commons.lists.PairList;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.Random;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarAngularExcursionTest implements MultiRobotTestInterface
{
   private static final boolean keepSCSUp = false;

   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;
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
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   private void setupTest() throws SimulationExceededMaximumTimeException
   {
      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();
      String className = getClass().getSimpleName();

      PrintTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      robotModel = getRobotModel();

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                             flatGround,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);

      simulationTestHelper.simulateNow(1.0);
   }

   @Test
   public void testMoveInPlace() throws SimulationExceededMaximumTimeException
   {
      // only set to true when saving new angular momentum data. output file usually needs to be manually moved to resources folder
      setupTest();
      setupCameraSideView();

      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1.5, 0.9));

      simulationTestHelper.simulateNow(2.0);

      ((YoBoolean) simulationTestHelper.findVariable("zeroAngularExcursionFlag")).set(true);

      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1.5, 0.7));
      simulationTestHelper.simulateNow(3.0);

      simulationTestHelper.publishToController(HumanoidMessageTools.createChestTrajectoryMessage(1.0, new Quaternion(Math.toRadians(20.0), Math.toRadians(10.0), Math.toRadians(15.0)), ReferenceFrame.getWorldFrame()));
      simulationTestHelper.simulateNow(2.5);
      simulationTestHelper.publishToController(HumanoidMessageTools.createChestTrajectoryMessage(1.0, new Quaternion(Math.toRadians(-20.0), Math.toRadians(10.0), Math.toRadians(15.0)), ReferenceFrame.getWorldFrame()));
      simulationTestHelper.simulateNow(2.5);
      simulationTestHelper.publishToController(HumanoidMessageTools.createChestTrajectoryMessage(1.0, new Quaternion(Math.toRadians(-20.0), Math.toRadians(-10.0), Math.toRadians(15.0)), ReferenceFrame.getWorldFrame()));
      simulationTestHelper.simulateNow(2.5);
      simulationTestHelper.publishToController(HumanoidMessageTools.createChestTrajectoryMessage(1.0, new Quaternion(), ReferenceFrame.getWorldFrame()));
      simulationTestHelper.publishToController(HumanoidMessageTools.createPelvisHeightTrajectoryMessage(1.5, 0.9));
      simulationTestHelper.simulateNow(4.0);


      YoDouble angularExcursionYaw = (YoDouble) simulationTestHelper.findVariable("angularExcursionYaw");
      YoDouble angularExcursionPitch = (YoDouble) simulationTestHelper.findVariable("angularExcursionPitch");
      YoDouble angularExcursionRoll = (YoDouble) simulationTestHelper.findVariable("angularExcursionRoll");

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

      ((YoBoolean) simulationTestHelper.findVariable("zeroAngularExcursionFlag")).set(true);

      YoDouble angularExcursionYaw = (YoDouble) simulationTestHelper.findVariable("angularExcursionYaw");
      YoDouble angularExcursionPitch = (YoDouble) simulationTestHelper.findVariable("angularExcursionPitch");
      YoDouble angularExcursionRoll = (YoDouble) simulationTestHelper.findVariable("angularExcursionRoll");

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

      simulationTestHelper.publishToController(footsteps);


      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      double simulationTime = initialTransfer + (transfer + swing) * (footsteps.getFootstepDataList().size() + 1) + 1.0;
      assertTrue(simulationTestHelper.simulateNow(simulationTime));

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

      ((YoBoolean) simulationTestHelper.findVariable("zeroAngularExcursionFlag")).set(true);

      simulationTestHelper.simulateNow(0.1);

      YoDouble angularExcursionYaw = (YoDouble) simulationTestHelper.findVariable("angularExcursionYaw");
      YoDouble angularExcursionPitch = (YoDouble) simulationTestHelper.findVariable("angularExcursionPitch");
      YoDouble angularExcursionRoll = (YoDouble) simulationTestHelper.findVariable("angularExcursionRoll");

      int numberOfSteps = 4;

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();

      PairList<FootstepDataListMessage, FootstepDataListMessage> footstepsSides = createSquareFootstepMessage(numberOfSteps);
      for (int i = 0; i < footstepsSides.size(); i++)
      {
         FootstepDataListMessage sideFootsteps = footstepsSides.get(i).getLeft();
         simulationTestHelper.publishToController(sideFootsteps);

         double simulationTime = initialTransfer + (transfer + swing) * (sideFootsteps.getFootstepDataList().size() + 1) + 1.0;
         assertTrue(simulationTestHelper.simulateNow(simulationTime));

//         assertEquals("Roll excursion is too great.", 0.0, angularExcursionRoll.getDoubleValue(), 1e-2);
//         assertEquals("Pitch excursion is too great. ", 0.0, angularExcursionPitch.getDoubleValue(), 1e-3);

         FootstepDataListMessage turnFootsteps = footstepsSides.get(i).getRight();
         simulationTestHelper.publishToController(turnFootsteps);

         simulationTime = initialTransfer + (transfer + swing) * (turnFootsteps.getFootstepDataList().size() + 1) + 1.0;
         assertTrue(simulationTestHelper.simulateNow(simulationTime));

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
      simulationTestHelper.setCameraFocusPosition(cameraFix);
      simulationTestHelper.setCameraPosition(cameraPosition);
   }

}
