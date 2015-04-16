package us.ihmc.atlas.ObstacleCourseTests;

import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.atlas.parameters.AtlasWalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.communication.packets.walking.ComHeightPacket;
import us.ihmc.communication.packets.walking.FootstepData;
import us.ihmc.communication.packets.walking.FootstepDataList;
import us.ihmc.darpaRoboticsChallenge.*;
import us.ihmc.darpaRoboticsChallenge.environment.DRCFinalsEnvironment;
import us.ihmc.darpaRoboticsChallenge.testTools.DRCSimulationTestHelper;
import us.ihmc.darpaRoboticsChallenge.testTools.ScriptedFootstepGenerator;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.utilities.MemoryTools;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.code.agileTesting.BambooAnnotations;
import us.ihmc.utilities.math.geometry.*;
import us.ihmc.utilities.math.trajectories.TrajectoryType;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.robotSide.SideDependentList;

/**
 * Created by agrabertilton on 4/15/15.
 */
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


   @BambooAnnotations.EstimatedDuration(duration = 30.4)
   @Test(timeout = 151825)
   public void testWalkingUpStaris() throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage();

      DRCStartingLocation selectedLocation = DRCSCStartingLocations.STAIRS_START;
      simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();

      AtlasRobotModel robotModel = AtlasRobotModelFactory.createDefaultRobotModel();
      robotModel.addMoreFootContactPointsSimOnly();
      DRCFinalsEnvironment environment = new DRCFinalsEnvironment(false, false, false, false, true);
      drcSimulationTestHelper = new DRCSimulationTestHelper(environment, "DRCWalkingUpStairsTest", "", selectedLocation, simulationTestingParameters,
              robotModel);

      SimulationConstructionSet simulationConstructionSet = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

      setupCameraForWalkingUpStairs();

      ThreadTools.sleep(1000);
      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);
      ComHeightPacket heightPacket = new ComHeightPacket(0.1);
      drcSimulationTestHelper.send(heightPacket);

      success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);    // 2.0);

      FootstepDataList footstepDataList = createFootstepsWithHighSwing(robotModel.getWalkingControllerParameters());
      drcSimulationTestHelper.send(footstepDataList);

      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(24.0);


      drcSimulationTestHelper.createMovie(getSimpleRobotName(), 1);
      drcSimulationTestHelper.checkNothingChanged();

      assertTrue(success);

      Point3d center = new Point3d(1.0,-15.7, 1.8);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);


      BambooTools.reportTestFinishedMessage();
   }

   private ArrayList<Point2d> createSupportPolygonForFoot(RobotSide side, WalkingControllerParameters walkingControllerParameters){
      ArrayList<Point2d> footSupportPolygon = new ArrayList<>();
      double rearOfFoot = -walkingControllerParameters.getFootLength()/2.0;
      double frontOfFoot = walkingControllerParameters.getFootLength()/2.0;
      if (side == RobotSide.LEFT){
         double cropPercentage = 0.5;
         double adjustedRearOfFoot = rearOfFoot + cropPercentage * (frontOfFoot - rearOfFoot);
         double frontWidth = walkingControllerParameters.getToeWidth()/2.0;
         double rearWidth = (1- cropPercentage) * walkingControllerParameters.getFootWidth()/2.0 + cropPercentage *frontWidth;

         footSupportPolygon.add(new Point2d(frontOfFoot, frontWidth));
         footSupportPolygon.add(new Point2d(frontOfFoot, -frontWidth));
         footSupportPolygon.add(new Point2d(adjustedRearOfFoot, rearWidth));
         footSupportPolygon.add(new Point2d(adjustedRearOfFoot, -rearWidth));
         return footSupportPolygon;
      }
      if (side == RobotSide.RIGHT){
         return null;
      }
      return null;
   }

   private FootstepDataList createFootstepsWithHighSwing(WalkingControllerParameters walkingControllerParameters)
   {
      double swingHeight = 0.10;
      Quat4d orientation = new Quat4d();
      Vector3d verticalVector = new Vector3d(0.0, 0.0, 1.0);
      FootstepDataList footstepDataList = new FootstepDataList();

      Point3d startingLocation = new Point3d(1.0,-13.5, 0);
      double directionYaw = -90.0;
      RotationFunctions.getQuaternionFromYawAndZNormal(directionYaw/180.0 * Math.PI, verticalVector, orientation);

      ArrayList<Point2d> footPoint2ds;
      footPoint2ds = createSupportPolygonForFoot(RobotSide.LEFT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(startingLocation.x + 0.15, startingLocation.y + 0.0, startingLocation.z + 0.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.RIGHT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(startingLocation.x - 0.15, startingLocation.y + 0.0, startingLocation.z + 0.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.LEFT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(startingLocation.x + 0.15, startingLocation.y - 0.25, startingLocation.z + 0.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.RIGHT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(startingLocation.x - 0.15, startingLocation.y - 0.55, startingLocation.z + 0.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.LEFT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(startingLocation.x + 0.15, startingLocation.y - 0.83, startingLocation.z + 0.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.RIGHT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(startingLocation.x - 0.15, startingLocation.y - 0.83, startingLocation.z + 0.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.LEFT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(startingLocation.x + 0.15, startingLocation.y - 1.10, startingLocation.z + 0.30), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.RIGHT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(startingLocation.x - 0.15, startingLocation.y - 1.18, startingLocation.z + 0.30), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.LEFT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(startingLocation.x + 0.15, startingLocation.y - 1.35, startingLocation.z + 0.55), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.RIGHT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(startingLocation.x - 0.15, startingLocation.y - 1.45, startingLocation.z + 0.55), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.LEFT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(startingLocation.x + 0.15, startingLocation.y - 1.66, startingLocation.z + 0.75), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.RIGHT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(startingLocation.x - 0.15, startingLocation.y - 1.74, startingLocation.z + 0.75), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.LEFT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(startingLocation.x + 0.15, startingLocation.y - 2.0, startingLocation.z + 1.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.RIGHT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(startingLocation.x - 0.15, startingLocation.y - 2.05, startingLocation.z + 1.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.LEFT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.LEFT, new Point3d(startingLocation.x + 0.15, startingLocation.y - 2.2, startingLocation.z + 1.0), new Quat4d(orientation), footPoint2ds));

      footPoint2ds = createSupportPolygonForFoot(RobotSide.RIGHT, walkingControllerParameters);
      footstepDataList.add(new FootstepData(RobotSide.RIGHT, new Point3d(startingLocation.x - 0.15, startingLocation.y - 2.2, startingLocation.z + 1.0), new Quat4d(orientation), footPoint2ds));

      return footstepDataList;
   }

   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }

   private void setupCameraForWalkingUpStairs()
   {
      Point3d cameraFix = new Point3d(1.8375, -15, 0.89);
      Point3d cameraPosition = new Point3d(8.5, -15, 2.0);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }
}
