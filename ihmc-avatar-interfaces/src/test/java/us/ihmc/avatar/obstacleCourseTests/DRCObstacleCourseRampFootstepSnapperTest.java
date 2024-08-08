package us.ihmc.avatar.obstacleCourseTests;

import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import us.ihmc.avatar.DRCObstacleCourseStartingLocation;
import us.ihmc.avatar.MultiRobotTestInterface;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.BasicFootstepMask;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.GenericFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.QuadTreeFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.SimpleFootstepSnapper;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.simulationConstructionSetTools.tools.CITools;
import us.ihmc.simulationConstructionSetTools.util.environments.DefaultCommonAvatarEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;

public abstract class DRCObstacleCourseRampFootstepSnapperTest implements MultiRobotTestInterface
{
   private final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private final boolean VISUALIZE = simulationTestingParameters.getKeepSCSUp();

   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void showMemoryUsageBeforeTest()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
   }

   @AfterEach
   public void destroySimulationAndRecycleMemory()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }

      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " after test.");
   }

   // The default height seems to be a bit too low for the ramp
   //   private final ComHeightPacket comHeightPacket = new ComHeightPacket(0.05, 1.0);
   private final Random random = new Random(165163L);

   @Test
   public void testWalkingUpRampUsingSnapFootsteps()
   {
      CITools.reportTestStartedMessage(simulationTestingParameters.getShowWindows());
      doUpRampTest();

      Point3D center = new Point3D(7.579638943201888, 0.020725665285290903, 1.46537366331119);
      Vector3D plusMinusVector = new Vector3D(0.2, 0.2, 0.5);
      BoundingBox3D boundingBox = BoundingBox3D.createUsingCenterAndPlusMinusVector(center, plusMinusVector);
      simulationTestHelper.assertRobotsRootJointIsInBoundingBox(boundingBox);

      CITools.reportTestFinishedMessage(simulationTestingParameters.getShowWindows());
   }

   private void doUpRampTest()
   {
      DRCObstacleCourseStartingLocation selectedLocation = DRCObstacleCourseStartingLocation.RAMP_BOTTOM;

      DefaultCommonAvatarEnvironment environment = new DefaultCommonAvatarEnvironment();
      SCS2AvatarTestingSimulationFactory simulationTestHelperFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(getRobotModel(),
                                                                                                                                             environment,
                                                                                                                                             simulationTestingParameters);
      simulationTestHelperFactory.setStartingLocationOffset(selectedLocation.getStartingLocationOffset());
      simulationTestHelper = simulationTestHelperFactory.createAvatarTestingSimulation();
      simulationTestHelper.start();

      //      drcSimulationTestHelper.send(comHeightPacket);

      setupCameraForWalkingOverRamp();
      ThreadTools.sleep(1000);

      boolean success = simulationTestHelper.simulateNow(2.0);
      //      FullHumanoidRobotModel fullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      FootstepDataListMessage corruptedFootstepDataList = createFootstepsForWalkingUpRamp();

      //      ReferenceFrameHashCodeResolver resolver = new ReferenceFrameHashCodeResolver(fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));

      // Corrupt the footsteps by adding a big z offset and coorupting the pitch and roll
      FrameQuaternion tempFrameOrientation = new FrameQuaternion();
      for (int i = 0; i < corruptedFootstepDataList.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepData = corruptedFootstepDataList.getFootstepDataList().get(i);
         footstepData.getLocation().setZ(footstepData.getLocation().getZ() + 1.0);
         tempFrameOrientation.set(footstepData.getOrientation());
         YawPitchRoll yawPitchRoll = new YawPitchRoll();
         yawPitchRoll.set(tempFrameOrientation);
         yawPitchRoll.setPitch(RandomNumbers.nextDouble(random, Math.PI / 4.0));
         yawPitchRoll.setRoll(RandomNumbers.nextDouble(random, Math.PI / 4.0));
         tempFrameOrientation.set(yawPitchRoll);
         footstepData.getOrientation().set(tempFrameOrientation);
      }

      vidualizeCorruptedFootsteps(corruptedFootstepDataList);

      ArrayList<Footstep> corruptedFootstepList = new ArrayList<>();
      for (int i = 0; i < corruptedFootstepDataList.getFootstepDataList().size(); i++)
      {
         FootstepDataMessage footstepData = corruptedFootstepDataList.getFootstepDataList().get(i);
         RobotSide robotSide = RobotSide.fromByte(footstepData.getRobotSide());
         FramePose3D pose = new FramePose3D(ReferenceFrame.getWorldFrame());
         pose.set(footstepData.getLocation(), footstepData.getOrientation());
         corruptedFootstepList.add(new Footstep(robotSide, pose));
      }

      // Build the BoundingBox2D containing all the footsteps
      Point2D boundingBoxMin = new Point2D(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY);
      Point2D boundingBoxMax = new Point2D(Double.NEGATIVE_INFINITY, Double.NEGATIVE_INFINITY);

      List<FootstepDataMessage> dataList = corruptedFootstepDataList.getFootstepDataList();
      for (int i = 0; i < dataList.size(); i++)
      {
         FootstepDataMessage footstepData = dataList.get(i);
         double footstepX = footstepData.getLocation().getX();
         double footstepY = footstepData.getLocation().getY();

         boundingBoxMin.setX(Math.min(boundingBoxMin.getX(), footstepX));
         boundingBoxMin.setY(Math.min(boundingBoxMin.getY(), footstepY));
         boundingBoxMax.setX(Math.max(boundingBoxMax.getX(), footstepX));
         boundingBoxMax.setY(Math.max(boundingBoxMax.getY(), footstepY));
      }

      double enlarge = 0.2;
      boundingBoxMin.sub(new Point2D(enlarge, enlarge));
      boundingBoxMax.add(new Point2D(enlarge, enlarge));

      BoundingBox2D footstepContainer = new BoundingBox2D(boundingBoxMin, boundingBoxMax);

      us.ihmc.graphicsDescription.HeightMap inputHeightMap = environment.getTerrainObject3D().getHeightMapIfAvailable();
      double resolution = 0.02;
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;
      HeightMapWithPoints heightMap = createHeightMap(inputHeightMap,
                                                      footstepContainer,
                                                      resolution,
                                                      heightThreshold,
                                                      quadTreeMaxMultiLevelZChangeToFilterNoise,
                                                      maxSameHeightPointsPerNode,
                                                      maxAllowableXYDistanceForAPointToBeConsideredClose,
                                                      maxNodes);

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
         FramePoint3D position = new FramePoint3D();
         FrameQuaternion orientation = new FrameQuaternion();
         footstep.getPose(position, orientation);
         FootstepDataMessage footstepData = HumanoidMessageTools.createFootstepDataMessage(robotSide, position, orientation);
         snappedFootstepDataList.getFootstepDataList().add().set(footstepData);
      }

      // Send footsteps to controller
      simulationTestHelper.publishToController(snappedFootstepDataList);

      // Check for success
      success = success && simulationTestHelper.simulateNow(16.0);

      // TODO GITHUB WORKFLOWS
//      simulationTestHelper.createBambooVideo(getSimpleRobotName(), 2);

      // simulationTestHelper.checkNothingChanged();
      assertTrue(success);
   }

   private void vidualizeCorruptedFootsteps(FootstepDataListMessage corruptedFootstepDataList)
   {
      if (!VISUALIZE)
         return;

      List<FootstepDataMessage> dataList = corruptedFootstepDataList.getFootstepDataList();
      VisualDefinitionFactory visualFactory = new VisualDefinitionFactory();
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.Red());

      for (int i = 0; i < dataList.size(); i++)
      {
         FootstepDataMessage footstepData = dataList.get(i);
         visualFactory.identity();
         visualFactory.appendTranslation(footstepData.getLocation());
         visualFactory.appendRotation(footstepData.getOrientation());
         visualFactory.addCoordinateSystem(0.15, material);
      }
      simulationTestHelper.addStaticVisuals(visualFactory.getVisualDefinitions());
   }

   private void setupCameraForWalkingOverRamp()
   {
      Point3D cameraFix = new Point3D(5.0, -0.2, 0.89);
      Point3D cameraPosition = new Point3D(5.0, 7.8, 1.6);

      simulationTestHelper.setCamera(cameraFix, cameraPosition);
   }

   private SimpleFootstepSnapper createSimpleFootstepSnapper()
   {
      QuadTreeFootstepSnappingParameters snappingParameters = new GenericFootstepSnappingParameters();
      BasicFootstepMask footstepMask = new BasicFootstepMask(snappingParameters.getCollisionPolygon(), 0.0);

      SimpleFootstepSnapper footstepSnapper = new SimpleFootstepSnapper();
      boolean useMask = true;
      double kernelMaskSafetyBuffer = 0.15;
      double boundingBoxDimension = 0.15;
      footstepSnapper.setUseMask(useMask, kernelMaskSafetyBuffer, boundingBoxDimension);
      footstepSnapper.setMask(footstepMask);

      return footstepSnapper;
   }

   public HeightMapWithPoints createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap,
                                              BoundingBox2D testingRange,
                                              double resolution,
                                              double heightThreshold,
                                              double quadTreeMaxMultiLevelZChangeToFilterNoise,
                                              int maxSameHeightPointsPerNode,
                                              double maxAllowableXYDistanceForAPointToBeConsideredClose,
                                              int maxNodes)
   {
      double minX = testingRange.getMinPoint().getX();
      double maxX = testingRange.getMaxPoint().getX();
      double minY = testingRange.getMinPoint().getY();
      double maxY = testingRange.getMaxPoint().getY();

      ArrayList<Point3D> listOfPoints = new ArrayList<Point3D>();

      VisualDefinitionFactory visualFactory = new VisualDefinitionFactory();
      MaterialDefinition material = new MaterialDefinition(ColorDefinitions.Blue());

      for (double x = minX; x < maxX; x = x + resolution)
      {
         for (double y = minY; y < maxY; y = y + resolution)
         {
            double z = inputHeightMap.heightAt(x, y, 0.0);
            listOfPoints.add(new Point3D(x, y, z));

            if (VISUALIZE)
            {
               visualFactory.identity();
               visualFactory.appendTranslation(x, y, z + 0.001);
               visualFactory.addBox(0.002, 0.002, 0.002, material);
            }
         }
      }

      simulationTestHelper.addStaticVisuals(visualFactory.getVisualDefinitions());

      return createHeightMap(listOfPoints,
                             testingRange,
                             resolution,
                             heightThreshold,
                             quadTreeMaxMultiLevelZChangeToFilterNoise,
                             maxSameHeightPointsPerNode,
                             maxAllowableXYDistanceForAPointToBeConsideredClose,
                             maxNodes);
   }

   public HeightMapWithPoints createHeightMap(ArrayList<Point3D> listOfPoints,
                                              BoundingBox2D testingRange,
                                              double resolution,
                                              double heightThreshold,
                                              double quadTreeMaxMultiLevelZChangeToFilterNoise,
                                              int maxSameHeightPointsPerNode,
                                              double maxAllowableXYDistanceForAPointToBeConsideredClose,
                                              int maxNodes)
   {
      double minX = testingRange.getMinPoint().getX();
      double maxX = testingRange.getMaxPoint().getX();
      double minY = testingRange.getMinPoint().getY();
      double maxY = testingRange.getMaxPoint().getY();

      Box bounds = new Box(minX, minY, maxX, maxY);
      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(resolution,
                                                                                       heightThreshold,
                                                                                       quadTreeMaxMultiLevelZChangeToFilterNoise,
                                                                                       maxSameHeightPointsPerNode,
                                                                                       maxAllowableXYDistanceForAPointToBeConsideredClose,
                                                                                       -1);
      QuadTreeForGroundHeightMap heightMap = new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);

      for (Point3D point : listOfPoints)
      {
         heightMap.addPoint(point.getX(), point.getY(), point.getZ());
      }

      return heightMap;
   }

   private FootstepDataListMessage createFootstepsForWalkingUpRamp()
   {
      Pose3D[] footstepPoses = {new Pose3D(3.00, -0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(3.35, 0.1, 0.0, 0.0, 0.0, 0.0),
            new Pose3D(3.73, -0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(4.10, 0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(4.48, -0.1, 0.0, 0.0, 0.0, 0.0),
            new Pose3D(4.86, 0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(5.25, -0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(5.63, 0.1, 0.0, 0.0, 0.0, 0.0),
            new Pose3D(6.01, -0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(6.40, 0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(6.79, -0.1, 0.0, 0.0, 0.0, 0.0),
            new Pose3D(7.17, 0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(7.56, -0.1, 0.0, 0.0, 0.0, 0.0), new Pose3D(7.56, 0.1, 0.0, 0.0, 0.0, 0.0)};

      return EndToEndTestTools.generateFootstepsFromPose3Ds(RobotSide.RIGHT, footstepPoses);
   }
}
