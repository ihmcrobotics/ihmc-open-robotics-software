package us.ihmc.avatar;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
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
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class AvatarRangeOfMotionTests implements MultiRobotTestInterface
{
   private static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();

   private SCS2AvatarTestingSimulation simulationTestHelper;
   private boolean useExperimentalPhysicsEngine = true;

   public void setUseExperimentalPhysicsEngine(boolean useExperimentalPhysicsEngine)
   {
      this.useExperimentalPhysicsEngine = useExperimentalPhysicsEngine;
   }

   public abstract double getDesiredPelvisHeightAboveFoot();

   @BeforeEach
   public void setup()
   {
      simulationTestingParameters.setKeepSCSUp(simulationTestingParameters.getKeepSCSUp()
            && !ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
   }

   @AfterEach
   public void tearDown()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
      }

      simulationTestHelper = null;
   }

   @Test
   public void testSquattingDown() throws Exception
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      Random random = new Random(564574L);

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(useExperimentalPhysicsEngine);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;

      FramePoint3D footPosition = new FramePoint3D(fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame());
      footPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double desiredPelvisHeight = footPosition.getZ() + getDesiredPelvisHeightAboveFoot();
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime,
                                                                                                                             desiredPelvisHeight);
      pelvisHeightTrajectoryMessage.setSequenceId(random.nextLong());

      simulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);

      success = simulationTestHelper.simulateNow(5.0 + trajectoryTime);
      assertTrue(success);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testWalkingOffOfLargePlatform()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_LARGE_PLATFORM;

      System.out.println(selectedLocation.getStartingLocationOffset().getAdditionalOffset());

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(useExperimentalPhysicsEngine);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      setupCameraForWalkingOffOfLargePlatform();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(2.0);

      FootstepDataListMessage footstepDataList = createFootstepsForSteppingOffOfLargePlatform();
      simulationTestHelper.publishToController(footstepDataList);

      success = success && simulationTestHelper.simulateNow(4.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 1);

      assertTrue(success);

      Point3D center = new Point3D(-5.8, -7.5, 0.87);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.2);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   @Test
   public void testPitchingChestSuperFar()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(true);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;

      FrameQuaternion chestOrientation = new FrameQuaternion(fullRobotModel.getChest().getBodyFixedFrame());
      chestOrientation.appendPitchRotation(Math.toRadians(60));
      chestOrientation.changeFrame(ReferenceFrame.getWorldFrame());

      ChestTrajectoryMessage chestTrajectoryMessage = HumanoidMessageTools.createChestTrajectoryMessage(trajectoryTime,
                                                                                                        chestOrientation,
                                                                                                        ReferenceFrame.getWorldFrame());

      simulationTestHelper.publishToController(chestTrajectoryMessage);

      success = simulationTestHelper.simulateNow(1.0 + trajectoryTime);
      assertTrue(success);

      FramePoint3D footPosition = new FramePoint3D(fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame());
      footPosition.changeFrame(ReferenceFrame.getWorldFrame());

      double desiredPelvisHeight = footPosition.getZ() + getDesiredPelvisHeightAboveFoot();
      PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(trajectoryTime,
                                                                                                                             desiredPelvisHeight);
      simulationTestHelper.publishToController(pelvisHeightTrajectoryMessage);

      success = simulationTestHelper.simulateNow(4.0 + trajectoryTime);
      assertTrue(success);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   @Test
   public void testHighFoot()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setUseImpulseBasedPhysicsEngine(true);
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      ThreadTools.sleep(1000);
      boolean success = simulationTestHelper.simulateNow(0.5);
      assertTrue(success);

      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      double trajectoryTime = 1.0;

      FramePose3D footPose = new FramePose3D(fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame());
      footPose.getPosition().addZ(0.5);
      footPose.changeFrame(ReferenceFrame.getWorldFrame());

      FootTrajectoryMessage footTrajectoryMessage = HumanoidMessageTools.createFootTrajectoryMessage(RobotSide.LEFT, trajectoryTime, footPose);

      simulationTestHelper.publishToController(footTrajectoryMessage);

      success = simulationTestHelper.simulateNow(4.0 + trajectoryTime);
      assertTrue(success);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);
   }

   private void setupCameraForWalkingOffOfLargePlatform()
   {
      Point3D cameraFix = new Point3D(-4.68, -7.8, 0.55);
      Point3D cameraPosition = new Point3D(-8.6, -4.47, 0.58);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFootstepsForSteppingOffOfLargePlatform()
   {
      double width = 0.3;
      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.getFootstepDataList().add()
                             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT,
                                                                                 new Point3D(-5.65, -7.471 - width / 2.0, 0.0),
                                                                                 new Quaternion(-0.0042976203878775715,
                                                                                                -0.010722204803598987,
                                                                                                0.9248070170408506,
                                                                                                -0.38026115501738456)));
      footstepDataListMessage.getFootstepDataList().add()
                             .set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT,
                                                                                 new Point3D(-5.90, -7.471 + width / 2.0, 0.0),
                                                                                 new Quaternion(-8.975861226689934E-4,
                                                                                                0.002016837110644428,
                                                                                                0.9248918980282926,
                                                                                                -0.380223754740342)));
      return footstepDataListMessage;
   }
}
