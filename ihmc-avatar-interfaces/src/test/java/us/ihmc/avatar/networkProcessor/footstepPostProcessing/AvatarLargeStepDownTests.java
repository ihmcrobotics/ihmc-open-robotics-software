package us.ihmc.avatar.networkProcessor.footstepPostProcessing;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningRequestPacket;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.initialSetup.OffsetAndYawRobotInitialSetup;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.commons.ContinuousIntegrationTools;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.communication.FootstepPlannerCommunicationProperties;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.robotics.Assert.assertTrue;
import static us.ihmc.robotics.Assert.fail;

public abstract class AvatarLargeStepDownTests implements MultiRobotTestInterface
{
   protected SimulationTestingParameters simulationTestingParameters;
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
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


   @Test
   public void testWalkingOffOfMediumPlatform() throws SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(!ContinuousIntegrationTools.isRunningOnContinuousIntegrationServer());
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCNetworkModuleParameters networkModuleParameters = new DRCNetworkModuleParameters();
      networkModuleParameters.enableFootstepPlanningToolbox(true);

      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.ON_MEDIUM_PLATFORM;
      drcSimulationTestHelper = new DRCSimulationTestHelper( simulationTestingParameters, getRobotModel());
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.setNetworkProcessorParameters(networkModuleParameters);
      drcSimulationTestHelper.createSimulation("DRCWalkingOntoMediumPlatformToesTouchingTest");

      setupCameraForWalkingOffOfMediumPlatform();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);

      IHMCROS2Publisher<FootstepPlanningRequestPacket> planningRequestPublisher = drcSimulationTestHelper.createPublisher(FootstepPlanningRequestPacket.class, FootstepPlannerCommunicationProperties.subscriberTopicNameGenerator(getSimpleRobotName()));

      AtomicReference<FootstepPlanningToolboxOutputStatus> outputStatus = new AtomicReference<>();
      drcSimulationTestHelper.createSubscriber(FootstepPlanningToolboxOutputStatus.class,
                                               FootstepPlannerCommunicationProperties.publisherTopicNameGenerator(getSimpleRobotName()), outputStatus::set);


      OffsetAndYawRobotInitialSetup startingLocation = selectedLocation.getStartingLocationOffset();

      FramePose3D leftFoot = new FramePose3D(drcSimulationTestHelper.getControllerFullRobotModel().getSoleFrame(RobotSide.LEFT));
      leftFoot.changeFrame(ReferenceFrame.getWorldFrame());

      FootstepPlanningRequestPacket request = new FootstepPlanningRequestPacket();
      request.setInitialStanceRobotSide(FootstepPlanningRequestPacket.ROBOT_SIDE_LEFT);
      request.getStanceFootPositionInWorld().set(leftFoot.getPosition());
      request.getStanceFootOrientationInWorld().set(leftFoot.getOrientation());

      PoseReferenceFrame startingFrame = new PoseReferenceFrame("startingFrame", ReferenceFrame.getWorldFrame());
      startingFrame.setPositionAndUpdate(new FramePoint3D(ReferenceFrame.getWorldFrame(), startingLocation.getAdditionalOffset()));
      startingFrame.setOrientationAndUpdate(new Quaternion(startingLocation.getYaw(), 0.0, 0.0));

      FramePose3D goalpose = new FramePose3D(startingFrame);
      goalpose.setPosition(1.0, 0.0, 0.0);
      goalpose.changeFrame(ReferenceFrame.getWorldFrame());

      request.getGoalPositionInWorld().set(goalpose.getPosition());
      request.getGoalOrientationInWorld().set(goalpose.getOrientation());

      planningRequestPublisher.publish(request);


      double maxTimeToWait = 20.0;
      long startTime = System.nanoTime();
      while (outputStatus.get() == null && Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) < maxTimeToWait)
      {
         ThreadTools.sleep(100);
      }

      if (outputStatus.get() == null)
      {
         fail("Never received an output, even after 20 seconds.");
      }

      drcSimulationTestHelper.publishToController(outputStatus.get().getFootstepDataList());

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(4.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(-4.4003012528878935, -6.046150532235836, 0.7887649325247877);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }



   private void setupCameraForWalkingOffOfMediumPlatform()
   {
      Point3D cameraFix = new Point3D(-3.9, -5.6, 0.55);
      Point3D cameraPosition = new Point3D(-7.6, -2.4, 0.58);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
