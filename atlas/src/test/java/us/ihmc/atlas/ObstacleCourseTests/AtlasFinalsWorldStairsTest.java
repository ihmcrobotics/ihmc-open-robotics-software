package us.ihmc.atlas.ObstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.environments.DRCFinalsEnvironment;
import us.ihmc.avatar.simulationStarter.DRCSCStartingLocations;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

public class AtlasFinalsWorldStairsTest
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
   public void testWalkingUpStairs() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCStartingLocation selectedLocation = DRCSCStartingLocations.STAIRS_START;

      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 10, 2, true, true);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
      DRCFinalsEnvironment environment = new DRCFinalsEnvironment(false, false, false, false, true);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingUpStairsTest");

      setupCameraForWalkingUpStairs();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      ReferenceFrame rootFrame = drcSimulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      FramePoint3D pelvisPosition = new FramePoint3D(rootFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + 0.045);
      drcSimulationTestHelper.publishToController(message);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      FootstepDataListMessage footstepDataList = createFootstepsWithHighSwing(robotModel.getWalkingControllerParameters());
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(14.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.0, -15.7, 1.8);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@Disabled
   @Test
   public void testFastWalkingUpStairs() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCStartingLocation selectedLocation = DRCSCStartingLocations.STAIRS_START;

      FootContactPoints<RobotSide> simulationContactPoints = new AdditionalSimulationContactPoints<>(RobotSide.values, 8, 3, true, true);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
      DRCFinalsEnvironment environment = new DRCFinalsEnvironment(false, false, false, false, true);
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      drcSimulationTestHelper.setStartingLocation(selectedLocation);
      drcSimulationTestHelper.createSimulation("DRCWalkingUpStairsTest");

      setupCameraForWalkingUpStairs();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);
      ReferenceFrame rootFrame = drcSimulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      FramePoint3D pelvisPosition = new FramePoint3D(rootFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = HumanoidMessageTools.createPelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + 0.07);
      drcSimulationTestHelper.publishToController(message);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);

      FootstepDataListMessage footstepDataList = createFastFootstepsForStairs(robotModel.getWalkingControllerParameters());
      drcSimulationTestHelper.publishToController(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(12.0);


      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.0, -15.7, 1.8);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private ArrayList<Point2D> createPartialSupportPolygonForFoot(WalkingControllerParameters walkingControllerParameters)
   {
      ArrayList<Point2D> footSupportPolygon = new ArrayList<>();
      double rearOfFoot = -walkingControllerParameters.getSteppingParameters().getFootLength() / 2.0;
      double frontOfFoot = walkingControllerParameters.getSteppingParameters().getFootLength() / 2.0;
      double cropPercentage = 0.5;
      double adjustedRearOfFoot = rearOfFoot + cropPercentage * (frontOfFoot - rearOfFoot);
      double frontWidth = walkingControllerParameters.getSteppingParameters().getToeWidth() / 2.0;
      double rearWidth = (1 - cropPercentage) * walkingControllerParameters.getSteppingParameters().getFootWidth() / 2.0 + cropPercentage * frontWidth;

      footSupportPolygon.add(new Point2D(frontOfFoot, frontWidth));
      footSupportPolygon.add(new Point2D(frontOfFoot, -frontWidth));
      footSupportPolygon.add(new Point2D(adjustedRearOfFoot, rearWidth));
      footSupportPolygon.add(new Point2D(adjustedRearOfFoot, -rearWidth));

      return footSupportPolygon;
   }

   private FootstepDataListMessage createFootstepsWithHighSwing(WalkingControllerParameters walkingControllerParameters)
   {
      Quaternion orientation = new Quaternion();
      Vector3D verticalVector = new Vector3D(0.0, 0.0, 1.0);
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage();

      Point3D startingLocation = new Point3D(1.0, -13.53, 0.0);
      double directionYaw = -90.0;
      RotationTools.computeQuaternionFromYawAndZNormal(directionYaw / 180.0 * Math.PI, verticalVector, orientation);

      ArrayList<Point2D> leftFootPoint2ds = createPartialSupportPolygonForFoot(walkingControllerParameters);
      ArrayList<Point2D> rightFootPoint2ds = null;

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 0.25, startingLocation.getZ() + 0.0), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 0.55, startingLocation.getZ() + 0.0), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 0.83, startingLocation.getZ() + 0.0), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 0.83, startingLocation.getZ() + 0.0), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.10, startingLocation.getZ() + 0.25), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 1.18, startingLocation.getZ() + 0.25), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.40, startingLocation.getZ() + 0.45), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 1.45, startingLocation.getZ() + 0.45), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.68, startingLocation.getZ() + 0.68), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 1.74, startingLocation.getZ() + 0.68), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 2.0, startingLocation.getZ() + 0.91), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 2.05, startingLocation.getZ() + 0.91), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 2.2, startingLocation.getZ() + 0.91), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 2.2, startingLocation.getZ() + 0.91), new Quaternion(orientation), rightFootPoint2ds));

      return footstepDataList;
   }

   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   private void setupCameraForWalkingUpStairs()
   {
      Point3D cameraFix = new Point3D(1.8375, -15, 0.89);
      Point3D cameraPosition = new Point3D(8.5, -15, 2.0);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private FootstepDataListMessage createFastFootstepsForStairs(WalkingControllerParameters walkingControllerParameters)
   {
      Quaternion orientation = new Quaternion();
      Vector3D verticalVector = new Vector3D(0.0, 0.0, 1.0);
      double swingTime = 0.8; //(0.6 is default sim time)
      double transferTime = 0.25; //(0.25 is default sim time)

//      swingTime = 0.6; //default sim time
//      transferTime = 0.25; //default sim time
      FootstepDataListMessage footstepDataList = HumanoidMessageTools.createFootstepDataListMessage(swingTime, transferTime);

      Point3D startingLocation = new Point3D(1.0, -13.5, 0);
      double directionYaw = -90.0;
      RotationTools.computeQuaternionFromYawAndZNormal(directionYaw / 180.0 * Math.PI, verticalVector, orientation);

      ArrayList<Point2D> leftFootPoint2ds = createPartialSupportPolygonForFoot(walkingControllerParameters);
      ArrayList<Point2D> rightFootPoint2ds = createPartialSupportPolygonForFoot(walkingControllerParameters);

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 0.27, startingLocation.getZ() + 0.0), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 0.57, startingLocation.getZ() + 0.0), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 0.92, startingLocation.getZ() + 0.0), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 0.92, startingLocation.getZ() + 0.0), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.1, startingLocation.getZ() + 0.25), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 1.4, startingLocation.getZ() + 0.45), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.69, startingLocation.getZ() + 0.68), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 2.02, startingLocation.getZ() + 0.91), new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 2.22, startingLocation.getZ() + 0.91), new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.getFootstepDataList().add().set(HumanoidMessageTools.createFootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 2.22, startingLocation.getZ() + 0.91), new Quaternion(orientation), rightFootPoint2ds));


      return footstepDataList;
   }
}
