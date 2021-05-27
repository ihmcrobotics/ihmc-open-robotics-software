package us.ihmc.avatar;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;
import static us.ihmc.robotics.Assert.assertTrue;

public abstract class AvatarRangeOfMotionTests implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private DRCSimulationTestHelper drcSimulationTestHelper;
   private boolean useExperimentalPhysicsEngine = true;

   public void setUseExperimentalPhysicsEngine(boolean useExperimentalPhysicsEngine)
   {
      this.useExperimentalPhysicsEngine = useExperimentalPhysicsEngine;
   }

   public abstract double getDesiredPelvisHeightAboveFoot();

   @BeforeEach
   public void setup()
   {
      simulationTestingParameters.setKeepSCSUp(simulationTestingParameters.getKeepSCSUp() && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      drcSimulationTestHelper = null;
   }

   @Test
   public void testSquattingDown() throws Exception
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.DEFAULT;

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(useExperimentalPhysicsEngine);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;

      FramePoint3D footPosition = new FramePoint3D(fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame());
      footPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double desiredPelvisHeight = footPosition.getZ() + getDesiredPelvisHeightAboveFoot();
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime,
                                                                                                                             desiredPelvisHeight);
      pelvisHeightTrajectoryMessage.setSequenceId(random.nextLong());

      drcSimulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(5.0 + trajectoryTime);
      assertTrue(success);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }


   @Test
   public void testWalkingOffOfLargePlatform() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_LARGE_PLATFORM;

      System.out.println(selectedLocation.getStartingLocationOffset().getAdditionalOffset());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(useExperimentalPhysicsEngine);
      drcSimulationTestHelper.createSimulation("DRCWalkingOffOfLargePlatformTest");

      setupCameraForWalkingOffOfLargePlatform();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOffOfLargePlatform();
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-5.8, -7.5, 0.87);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.2);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testPitchingChestSuperFar() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(DRCObstacleCourseStartingLocation.DEFAULT);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;

      FrameQuaternion chestOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      chestOrientation.appendPitchRotation(Math.toRadians(60));
      chestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        chestOrientation,
                                                                                                        ReferenceFrame.getWorldFrame());

      drcSimulationTestHelper.publishToController(chestTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0 + trajectoryTime);
      assertTrue(success);

      FramePoint3D footPosition = new FramePoint3D(fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame());
      footPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double desiredPelvisHeight = footPosition.getZ() + getDesiredPelvisHeightAboveFoot();
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime,
                                                                                                                             desiredPelvisHeight);
      drcSimulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0 + trajectoryTime);
      assertTrue(success);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testHighFoot() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(DRCObstacleCourseStartingLocation.DEFAULT);
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      drcSimulationTestHelper.createSimulation(getClass().getSimpleName());

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;

      FramePose3D footPose = new FramePose3D(fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame());
      footPose.getPosition().addZ(0.5);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(RobotSide.LEFT, trajectoryTime, footPose);

      drcSimulationTestHelper.publishToController(footTrajectoryMessage);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0 + trajectoryTime);
      assertTrue(success);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);
   }

   private void setupCameraForWalkingOffOfLargePlatform()
   {
      Point3D cameraFix = new Point3D(-4.68, -7.8, 0.55);
      Point3D cameraPosition = new Point3D(-8.6, -4.47, 0.58);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }


   private FootstepDataListMessage createFootstepsForSteppingOffOfLargePlatform()
   {
      double width = 0.3;
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                                                             new Point3D(-5.65, -7.471 - width / 2.0, 0.0),
                                                                                                             new Quaternion(-0.0042976203878775715, -0.010722204803598987, 0.9248070170408506, -0.38026115501738456)));
      footstepDataListMessage.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                                             new Point3D(-5.95, -7.471 + width / 2.0, 0.0),
                                                                                                             new Quaternion(-8.975861226689934E-4, 0.002016837110644428, 0.9248918980282926, -0.380223754740342)));
      return footstepDataListMessage;
   }
}
