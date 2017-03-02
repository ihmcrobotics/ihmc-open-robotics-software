package us.ihmc.atlas.ObstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.avatar.simulationStarter.DRCSCStartingLocations;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationPlan;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJointReferenceFrame;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.environments.DarpaRoboticsChallengeFinalsEnvironment;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.wholeBodyController.AdditionalSimulationContactPoints;
import us.ihmc.wholeBodyController.FootContactPoints;

@ContinuousIntegrationPlan(categories = {IntegrationCategory.IN_DEVELOPMENT, IntegrationCategory.VIDEO})
public class AtlasFinalsWorldStairsTest
{
   protected SimulationTestingParameters simulationTestingParameters;
   protected DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
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

	@ContinuousIntegrationTest(estimatedDuration = 70.3, categoriesOverride = {IntegrationCategory.SLOW, IntegrationCategory.VIDEO})
   @Test(timeout = 350000)
   public void testWalkingUpStaris() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCStartingLocation selectedLocation = DRCSCStartingLocations.STAIRS_START;

      FootContactPoints simulationContactPoints = new AdditionalSimulationContactPoints(10, 2, true, false);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
      DarpaRoboticsChallengeFinalsEnvironment environment = new DarpaRoboticsChallengeFinalsEnvironment(false, false, false, false, true);
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCWalkingUpStairsTest", selectedLocation, simulationTestingParameters,
              robotModel);

      setupCameraForWalkingUpStairs();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);
      FloatingInverseDynamicsJointReferenceFrame rootFrame = drcSimulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      FramePoint pelvisPosition = new FramePoint(rootFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + 0.125);
      drcSimulationTestHelper.send(message);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);

      FootstepDataListMessage footstepDataList = createFootstepsWithHighSwing(robotModel.getWalkingControllerParameters());
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(24.0);


      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.0, -15.7, 1.8);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

	@Ignore("Improve the feet stability.")
   @ContinuousIntegrationTest(estimatedDuration = 120.0)
   @Test(timeout = 151825)
   public void testFastWalkingUpStaris() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());

      DRCStartingLocation selectedLocation = DRCSCStartingLocations.STAIRS_START;

      FootContactPoints simulationContactPoints = new AdditionalSimulationContactPoints(8, 3, false, false);
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false, simulationContactPoints);
      DarpaRoboticsChallengeFinalsEnvironment environment = new DarpaRoboticsChallengeFinalsEnvironment(false, false, false, false, true);
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCWalkingUpStairsTest", selectedLocation, simulationTestingParameters,
            robotModel);

      setupCameraForWalkingUpStairs();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);    // 2.0);
      FloatingInverseDynamicsJointReferenceFrame rootFrame = drcSimulationTestHelper.getControllerFullRobotModel().getRootJoint().getFrameAfterJoint();
      FramePoint pelvisPosition = new FramePoint(rootFrame);
      pelvisPosition.changeFrame(ReferenceFrame.getWorldFrame());
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage(0.5, pelvisPosition.getZ() + 0.15);
      drcSimulationTestHelper.send(message);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(0.5);    // 2.0);

      FootstepDataListMessage footstepDataList = createFastFootstepsForStairs(robotModel.getWalkingControllerParameters());
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(22.0);


      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3D center = new Point3D(1.0, -15.7, 1.8);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private ArrayList<Point2D> createPartialSupportPolygonForFoot(WalkingControllerParameters walkingControllerParameters)
   {
      ArrayList<Point2D> footSupportPolygon = new ArrayList<>();
      double rearOfFoot = -walkingControllerParameters.getFootLength() / 2.0;
      double frontOfFoot = walkingControllerParameters.getFootLength() / 2.0;
      double cropPercentage = 0.5;
      double adjustedRearOfFoot = rearOfFoot + cropPercentage * (frontOfFoot - rearOfFoot);
      double frontWidth = walkingControllerParameters.getToeWidth() / 2.0;
      double rearWidth = (1 - cropPercentage) * walkingControllerParameters.getFootWidth() / 2.0 + cropPercentage * frontWidth;

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

      Point3D startingLocation = new Point3D(1.0, -13.5, 0.02);
      double directionYaw = -90.0;
      RotationTools.computeQuaternionFromYawAndZNormal(directionYaw / 180.0 * Math.PI, verticalVector, orientation);

      ArrayList<Point2D> leftFootPoint2ds = createPartialSupportPolygonForFoot(walkingControllerParameters);
      ArrayList<Point2D> rightFootPoint2ds = null;
      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() + 0.0, startingLocation.getZ() + 0.0),
              new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() + 0.0, startingLocation.getZ() + 0.0),
              new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 0.25, startingLocation.getZ() + 0.0),
              new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 0.55, startingLocation.getZ() + 0.0),
              new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 0.83, startingLocation.getZ() + 0.0),
              new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 0.83, startingLocation.getZ() + 0.0),
              new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.10, startingLocation.getZ() + 0.30),
              new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 1.18, startingLocation.getZ() + 0.30),
              new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.40, startingLocation.getZ() + 0.55),
              new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 1.45, startingLocation.getZ() + 0.55),
              new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.68, startingLocation.getZ() + 0.75),
              new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 1.74, startingLocation.getZ() + 0.75),
              new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 2.0, startingLocation.getZ() + 1.0),
              new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 2.05, startingLocation.getZ() + 1.0),
              new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 2.2, startingLocation.getZ() + 1.0),
              new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 2.2, startingLocation.getZ() + 1.0),
              new Quaternion(orientation), rightFootPoint2ds));

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
      FootstepDataListMessage footstepDataList = new FootstepDataListMessage(swingTime, transferTime);

      Point3D startingLocation = new Point3D(1.0, -13.5, 0);
      double directionYaw = -90.0;
      RotationTools.computeQuaternionFromYawAndZNormal(directionYaw / 180.0 * Math.PI, verticalVector, orientation);

      ArrayList<Point2D> leftFootPoint2ds = createPartialSupportPolygonForFoot(walkingControllerParameters);
      ArrayList<Point2D> rightFootPoint2ds = createPartialSupportPolygonForFoot(walkingControllerParameters);
      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() + 0.0, startingLocation.getZ() + 0.0),
            new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() + 0.0, startingLocation.getZ() + 0.0),
            new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 0.25, startingLocation.getZ() + 0.0),
            new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 0.55, startingLocation.getZ() + 0.0),
            new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 0.90, startingLocation.getZ() + 0.0),
            new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 0.90, startingLocation.getZ() + 0.0),
            new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.08, startingLocation.getZ() + 0.32),
            new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 1.38, startingLocation.getZ() + 0.55),
            new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 1.67, startingLocation.getZ() + 0.78),
            new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 2.0, startingLocation.getZ() + 1.01),
            new Quaternion(orientation), rightFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.LEFT, new Point3D(startingLocation.getX() + 0.15, startingLocation.getY() - 2.2, startingLocation.getZ() + 1.01),
            new Quaternion(orientation), leftFootPoint2ds));

      footstepDataList.add(new FootstepDataMessage(RobotSide.RIGHT, new Point3D(startingLocation.getX() - 0.15, startingLocation.getY() - 2.2, startingLocation.getZ() + 1.01),
            new Quaternion(orientation), rightFootPoint2ds));


      return footstepDataList;
   }
}
