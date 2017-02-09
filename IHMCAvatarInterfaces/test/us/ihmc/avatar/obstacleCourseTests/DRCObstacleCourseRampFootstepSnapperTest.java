package us.ihmc.avatar.obstacleCourseTests;

import static org.junit.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Random;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.After;
import org.junit.Before;
import org.junit.Test;

import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.avatar.testTools.ScriptedFootstepGenerator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.BasicFootstepMask;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.GenericFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.SimpleFootstepSnapper;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.BoundingBox3d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.BambooTools;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.tools.MemoryTools;
import us.ihmc.tools.thread.ThreadTools;

public abstract class DRCObstacleCourseRampFootstepSnapperTest implements MultiRobotTestInterface
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromEnvironmentVariables();
   private final boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();

   private DRCSimulationTestHelper drcSimulationTestHelper;

   @Before
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @After
   public void destroySimulationAndRecycleMemory()
   {
      if (VISUALIZE)
      {
         ThreadTools.sleepForever();
      }

      // Do this here in case a test fails. That way the memory will be recycled.
      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   // The default height seems to be a bit too low for the ramp
//   private final ComHeightPacket comHeightPacket = new ComHeightPacket(0.05, 1.0);
   private final Random random = new Random(165163L);

	@ContinuousIntegrationTest(estimatedDuration = 50.0)
   @Test(timeout = 250000)
   public void testWalkingUpRampUsingSnapFootsteps() throws SimulationExceededMaximumTimeException
   {
      BambooTools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      doUpRampTest();

      Point3d center = new Point3d(7.579638943201888, 0.020725665285290903, 1.46537366331119);
      Vector3d plusMinusVector = new Vector3d(0.2, 0.2, 0.5);
      BoundingBox3d boundingBox = BoundingBox3d.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      drcSimulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      BambooTools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void doUpRampTest() throws SimulationExceededMaximumTimeException
   {
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_BOTTOM;

      drcSimulationTestHelper = new DRCSimulationTestHelper("DRCRampSnapFootstepsTest", selectedLocation, simulationTestingParameters, getRobotModel());

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      ScriptedFootstepGenerator scriptedFootstepGenerator = drcSimulationTestHelper.createScriptedFootstepGenerator();

//      drcSimulationTestHelper.send(comHeightPacket);

      setupCameraForWalkingOverRamp(scs);
      ThreadTools.sleep(1000);

      boolean success = drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(2.0);
      FullHumanoidRobotModel fullRobotModel = drcSimulationTestHelper.getAvatarSimulation().getControllerFullRobotModel();

      FootstepDataListMessage corruptedFootstepDataList = createFootstepsForWalkingUpRamp(scriptedFootstepGenerator);
      
      // Corrupt the footsteps by adding a big z offset and coorupting the pitch and roll
      FrameOrientation tempFrameOrientation = new FrameOrientation();
      for (int i = 0; i < corruptedFootstepDataList.getDataList().size(); i++)
      {
         FootstepDataMessage footstepData = corruptedFootstepDataList.getDataList().get(i);
         footstepData.location.setZ(footstepData.location.getZ() + 1.0);
         tempFrameOrientation.set(footstepData.getOrientation());
         double[] yawPitchRoll = tempFrameOrientation.getYawPitchRoll();
         yawPitchRoll[1] = RandomTools.generateRandomDouble(random, Math.PI / 4.0);
         yawPitchRoll[2] = RandomTools.generateRandomDouble(random, Math.PI / 4.0);
         tempFrameOrientation.setYawPitchRoll(yawPitchRoll);
         tempFrameOrientation.getQuaternion(footstepData.orientation);
      }
      
      vidualizeCorruptedFootsteps(corruptedFootstepDataList, scs);
      
      ArrayList<Footstep> corruptedFootstepList = new ArrayList<>();
      for (int i = 0; i < corruptedFootstepDataList.getDataList().size(); i++)
      {
         FootstepDataMessage footstepData = corruptedFootstepDataList.getDataList().get(i);
         RobotSide robotSide = footstepData.getRobotSide();
         RigidBody endEffector = fullRobotModel.getFoot(robotSide);
         ReferenceFrame soleFrame = fullRobotModel.getSoleFrame(robotSide);
         FramePose pose = new FramePose(ReferenceFrame.getWorldFrame());
         pose.setPose(footstepData.getLocation(), footstepData.getOrientation());
         PoseReferenceFrame poseReferenceFrame = new PoseReferenceFrame("footstep" + i, pose);
         corruptedFootstepList.add(new Footstep(endEffector, robotSide, soleFrame, poseReferenceFrame));
      }

      // Build the BoundingBox2d containing all the footsteps
      Point2d boundingBoxMin = new Point2d(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point2d boundingBoxMax = new Point2d(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      for (FootstepDataMessage footstepData : corruptedFootstepDataList.getDataList())
      {
         double footstepX = footstepData.getLocation().getX();
         double footstepY = footstepData.getLocation().getY();

         boundingBoxMin.setX(Math.min(boundingBoxMin.getX(), footstepX));
         boundingBoxMin.setY(Math.min(boundingBoxMin.getY(), footstepY));
         boundingBoxMax.setX(Math.max(boundingBoxMax.getX(), footstepX));
         boundingBoxMax.setY(Math.max(boundingBoxMax.getY(), footstepY));
      }

      double enlarge = 0.2;
      boundingBoxMin.sub(new Point2d(enlarge, enlarge));
      boundingBoxMax.add(new Point2d(enlarge, enlarge));

      BoundingBox2d footstepContainer = new BoundingBox2d(boundingBoxMin, boundingBoxMax);

      us.ihmc.graphicsDescription.HeightMap inputHeightMap = drcSimulationTestHelper.getTestEnviroment().getTerrainObject3D().getHeightMapIfAvailable();
      double resolution = 0.02;
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;
      HeightMapWithPoints heightMap = createHeightMap(inputHeightMap, footstepContainer, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
            maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes, scs);

      SimpleFootstepSnapper footstepSnapper = createSimpleFootstepSnapper();
      double maskSafetyBuffer = 0.01;
      double boundingBoxDimension = 0.3;
      footstepSnapper.setUseMask(true, maskSafetyBuffer, boundingBoxDimension);
 
      FootstepDataListMessage snappedFootstepDataList = new FootstepDataListMessage();
      for (int i = 0; i < corruptedFootstepList.size(); i++)
      {
         Footstep footstep = corruptedFootstepList.get(i);
         footstepSnapper.snapFootstep(footstep, heightMap);
         
         RobotSide robotSide = footstep.getRobotSide();
         Point3d location = new Point3d();
         Quat4d orientation = new Quat4d();
         footstep.getPosition(location);
         footstep.getOrientation(orientation);
         FootstepDataMessage footstepData = new FootstepDataMessage(robotSide, location, orientation);
         snappedFootstepDataList.add(footstepData);
      }

      // Send footsteps to controller
      drcSimulationTestHelper.send(snappedFootstepDataList);

      // Check for success
      success = success && drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(16.0);

      drcSimulationTestHelper.createVideo(getSimpleRobotName(), 2);

      drcSimulationTestHelper.checkNothingChanged();
      assertTrue(success);
   }

   private void vidualizeCorruptedFootsteps(FootstepDataListMessage corruptedFootstepDataList, SimulationConstructionSet scs)
   {
      if (!VISUALIZE)
         return;

      for (FootstepDataMessage footstepData : corruptedFootstepDataList.getDataList())
      {
            Graphics3DObject staticLinkGraphics = new Graphics3DObject();
            staticLinkGraphics.translate(new Vector3d(footstepData.location));
            Matrix3d rotationMatrix = new Matrix3d();
            rotationMatrix.set(footstepData.getOrientation());
            staticLinkGraphics.rotate(rotationMatrix);
            staticLinkGraphics.addCoordinateSystem(0.15, YoAppearance.Red());
            scs.addStaticLinkGraphics(staticLinkGraphics);
      }
   }

   private void setupCameraForWalkingOverRamp(SimulationConstructionSet scs)
   {
      Point3d cameraFix = new Point3d(5.0, -0.2, 0.89);
      Point3d cameraPosition = new Point3d(5.0, 7.8, 1.6);

      drcSimulationTestHelper.setupCameraForUnitTest(cameraFix, cameraPosition);
   }

   private SimpleFootstepSnapper createSimpleFootstepSnapper()
   {
      FootstepSnappingParameters snappingParameters = new GenericFootstepSnappingParameters();
      BasicFootstepMask footstepMask = new BasicFootstepMask(snappingParameters.getCollisionPolygon(), 0.0);

      SimpleFootstepSnapper footstepSnapper = new SimpleFootstepSnapper();
      boolean useMask = true;
      double kernelMaskSafetyBuffer = 0.15;
      double boundingBoxDimension = 0.15;
      footstepSnapper.setUseMask(useMask, kernelMaskSafetyBuffer, boundingBoxDimension);
      footstepSnapper.setMask(footstepMask);

      return footstepSnapper;
   }

   public HeightMapWithPoints createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap, BoundingBox2d testingRange, double resolution, double heightThreshold,
         double quadTreeMaxMultiLevelZChangeToFilterNoise, int maxSameHeightPointsPerNode, double maxAllowableXYDistanceForAPointToBeConsideredClose,
         int maxNodes, SimulationConstructionSet scs)
   {
      double minX = testingRange.getMinPoint().getX();
      double maxX = testingRange.getMaxPoint().getX();
      double minY = testingRange.getMinPoint().getY();
      double maxY = testingRange.getMaxPoint().getY();

      ArrayList<Point3d> listOfPoints = new ArrayList<Point3d>();

      for (double x = minX; x < maxX; x = x + resolution)
      {
         for (double y = minY; y < maxY; y = y + resolution)
         {
            double z = inputHeightMap.heightAt(x, y, 0.0);
            listOfPoints.add(new Point3d(x, y, z));

            if (VISUALIZE)
            {
               Graphics3DObject staticLinkGraphics = new Graphics3DObject();
               staticLinkGraphics.translate(new Vector3d(x, y, z + 0.001));
               staticLinkGraphics.addCube(0.002, 0.002, 0.002, YoAppearance.Blue());
               scs.addStaticLinkGraphics(staticLinkGraphics);
            }
         }
      }

      return createHeightMap(listOfPoints, testingRange, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
            maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);
   }

   public HeightMapWithPoints createHeightMap(ArrayList<Point3d> listOfPoints, BoundingBox2d testingRange, double resolution, double heightThreshold,
         double quadTreeMaxMultiLevelZChangeToFilterNoise, int maxSameHeightPointsPerNode, double maxAllowableXYDistanceForAPointToBeConsideredClose,
         int maxNodes)
   {
      double minX = testingRange.getMinPoint().getX();
      double maxX = testingRange.getMaxPoint().getX();
      double minY = testingRange.getMinPoint().getY();
      double maxY = testingRange.getMaxPoint().getY();

      Box bounds = new Box(minX, minY, maxX, maxY);
      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise,
            maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, -1);
      QuadTreeForGroundHeightMap heightMap = new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);

      for (Point3d point : listOfPoints)
      {
         heightMap.addPoint(point.getX(), point.getY(), point.getZ());
      }

      return heightMap;
   }

   private FootstepDataListMessage createFootstepsForWalkingUpRamp(ScriptedFootstepGenerator scriptedFootstepGenerator)
   {
      double[][][] footstepLocationsAndOrientations = new double[][][] {
            { { 3.00, -0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 3.35,  0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 3.73, -0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 4.10,  0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 4.48, -0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 4.86,  0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 5.25, -0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 5.63,  0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 6.01, -0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 6.40,  0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 6.79, -0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 7.17,  0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 7.56, -0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } },
            { { 7.56,  0.1, 0.0}, { 0.0, 0.0, 0.0, 1 } } };

      RobotSide[] robotSides = drcSimulationTestHelper.createRobotSidesStartingFrom(RobotSide.RIGHT, footstepLocationsAndOrientations.length);
      return scriptedFootstepGenerator.generateFootstepsFromLocationsAndOrientations(robotSides, footstepLocationsAndOrientations);
   }
}
