package us.ihmc.avatar;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapData;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.StateTransitionCondition;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationToolkit.controllers.PushRobotControllerSCS2;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoEnum;

import static us.ihmc.robotics.Assert.assertFalse;
import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarTurnInPlaceTest implements MultiRobotTestInterface
{
   private SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private DRCRobotModel robotModel;


   private double getTurnAngle()
   {
      return Math.toRadians(30);
   }

   protected OffsetAndYawRobotInitialSetup getStartingLocation()
   {
      return new OffsetAndYawRobotInitialSetup();
   }

   protected double getStepWidth()
   {
      return 0.25;
   }

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");

      FlatGroundEnvironment flatGround = new FlatGroundEnvironment();

      robotModel = getRobotModel();

      LogTools.debug("simulationTestingParameters.getKeepSCSUp " + simulationTestingParameters.getKeepSCSUp());
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             flatGround,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(getStartingLocation());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();
   }

   @AfterEach
   public void tearDown()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      simulationTestingParameters = null;
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   @Test
   public void testTurn720InPlace()
   {
      setupCameraSideView();

      simulationTestHelper.simulateNow(0.5);

      FootstepDataListMessage footstepDataListMessage = generateFootsteps(simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame());

      double initialTransfer = robotModel.getWalkingControllerParameters().getDefaultInitialTransferTime();
      double transfer = robotModel.getWalkingControllerParameters().getDefaultTransferTime();
      double swing = robotModel.getWalkingControllerParameters().getDefaultSwingTime();
      int steps = footstepDataListMessage.getFootstepDataList().size();

      simulationTestHelper.simulateNow(1.0);
      simulationTestHelper.publishToController(footstepDataListMessage);
      double simulationTime = initialTransfer + (transfer + swing) * steps + 1.0;

      assertTrue(simulationTestHelper.simulateNow(simulationTime));
   }

   private FootstepDataListMessage generateFootsteps(ReferenceFrame midFootFrame)
   {
      FootstepDataListMessage footsteps = new FootstepDataListMessage();

      PoseReferenceFrame turnedFrame = new PoseReferenceFrame("turnedFrame", midFootFrame);
      FrameQuaternion rotation = new FrameQuaternion(midFootFrame);

      int stepsToGenerate = (int) (4 * Math.PI / getTurnAngle());
      RobotSide stepSide = RobotSide.LEFT;
      for (int i = 0; i < stepsToGenerate; i++)
      {
         rotation.appendYawRotation(getTurnAngle());
         turnedFrame.setOrientationAndUpdate(rotation);

         FramePose3D footPose = new FramePose3D(turnedFrame);
         footPose.setY(stepSide.negateIfRightSide(getStepWidth() / 2.0));

         footPose.changeFrame(ReferenceFrame.getWorldFrame());

         FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
         footstep.getLocation().set(footPose.getPosition());
         footstep.getOrientation().set(footPose.getOrientation());
         footstep.setRobotSide(stepSide.toByte());

         stepSide = stepSide.getOppositeSide();
      }

      FramePose3D footPose = new FramePose3D(turnedFrame);
      footPose.setY(stepSide.negateIfRightSide(getStepWidth() / 2.0));

      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepDataMessage footstep = footsteps.getFootstepDataList().add();
      footstep.getLocation().set(footPose.getPosition());
      footstep.getOrientation().set(footPose.getOrientation());
      footstep.setRobotSide(stepSide.toByte());

      return footsteps;
   }


   private void setupCameraSideView()
   {
      Point3D cameraFix = new Point3D(0.0, 0.0, 1.0);
      Point3D cameraPosition = new Point3D(0.0, 10.0, 1.0);
      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }
}
