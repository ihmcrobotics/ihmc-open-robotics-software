package us.ihmc.humanoidRobotics.footstep.footstepSnapper;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.StringTokenizer;

import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataMessage;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.Footstep;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.BoundingBox2d;
import us.ihmc.robotics.geometry.ConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationTools;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.math.frames.YoFrameConvexPolygon2d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.math.frames.YoFramePose;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.random.RandomTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundReaderAndWriter;
import us.ihmc.simulationToolkit.visualizers.FootstepVisualizer;
import us.ihmc.simulationToolkit.visualizers.QuadTreeHeightMapVisualizer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.BumpyGroundProfile;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.tools.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.continuousIntegration.IntegrationCategory;
import us.ihmc.tools.thread.ThreadTools;

public class FootstepSnapperSimulationTest
{
	@ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFootstepAndPointsFromDataFile() throws NumberFormatException, InsufficientDataException, IOException
   {
      FootstepSnappingParameters snappingParameters = new AtlasFootstepSnappingParameters();
      ConvexHullFootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);
      double maskSafetyBuffer = 0.01;
      double boundingBoxDimension = 0.3;
      footstepSnapper.setUseMask(true, maskSafetyBuffer, boundingBoxDimension);

      String baseName = "footstepListsForTesting/";
      String resourceName = baseName + "DataFromConvexHullSnapper1422988400956.txt";
      InputStream resourceAsStream = getClass().getClassLoader().getResourceAsStream(resourceName);

      FootstepPointsDataReader dataReader = new FootstepPointsDataReader(resourceAsStream);
      FootstepDataMessage footstepData = new FootstepDataMessage();
      FootSpoof spoof = new FootSpoof("basicSpoof");
      FramePose2d desiredPose = new FramePose2d(ReferenceFrame.getWorldFrame());

      List<Point3d> listOfPoints = new ArrayList<>();
      while (dataReader.hasAnotherFootstepAndPoints())
      {
         listOfPoints = dataReader.getNextSetPointsAndFootstep(footstepData);
         desiredPose.setPoseIncludingFrame(ReferenceFrame.getWorldFrame(), footstepData.getLocation().getX(), footstepData.getLocation().getY(),
                                           RotationTools.computeYaw(footstepData.getOrientation()));
         Footstep footstep = footstepSnapper.generateFootstepUsingHeightMap(desiredPose, spoof.getRigidBody(), spoof.getSoleFrame(),
                                footstepData.getRobotSide(), listOfPoints, 0.0);

         assertTrue(footstep.getFootstepType() != Footstep.FootstepType.BAD_FOOTSTEP);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testPointsFromAtlasDataFile() throws NumberFormatException, InsufficientDataException, IOException
   {
      boolean assertPositionConditions = true;
      boolean assertPointConditions = true;
      boolean visualizeAndKeepUp = false;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;


      double minX = -1.0;    // 5.0f;
      double minY = -3.0;    // 5.0f;
      double maxX = 1.0;    // 5.0f;
      double maxY = 4.0;    // 5.0f;
      Box bounds = new Box(minX, minY, maxX, maxY);

      float resolution = 0.025f;
      float heightThreshold = 0.005f;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.02;
      int maxNodes = 1000000;


      String resourceName = "pointListsForTesting/croppedFirstFewCinderBlockScans.pointList";
      InputStream resourceAsStream = getClass().getClassLoader().getResourceAsStream(resourceName);

      double maxZ = 0.6;
      int skipPoints = 0;
      int maxNumberOfPoints = 2000000;

      QuadTreeForGroundReaderAndWriter quadTreeForGroundReaderAndWriter = new QuadTreeForGroundReaderAndWriter();
      ArrayList<Point3d> points = quadTreeForGroundReaderAndWriter.readPointsFromInputStream(resourceAsStream, skipPoints, maxNumberOfPoints, bounds, maxZ);

//    SimpleFootstepSnapper footstepSnapper = createSimpleFootstepSnapper();
      FootstepSnappingParameters snappingParameters = new AtlasFootstepSnappingParameters();
      ConvexHullFootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);
      double maskSafetyBuffer = 0.01;
      double boundingBoxDimension = 0.3;
      footstepSnapper.setUseMask(true, maskSafetyBuffer, boundingBoxDimension);

      BoundingBox2d rangeOfPointsToTest = new BoundingBox2d(minX, minY, maxX, maxY);
      FootstepSnapperTestHelper helper = new FootstepSnapperTestHelper("List of Points", footstepSnapper, new Graphics3DObject(), visualizeAndKeepUp);


      helper.createHeightMap(points, rangeOfPointsToTest, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
                             maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);

      if (visualizeAndKeepUp)
      {
         helper.drawPoints(points, resolution / 2.0, YoAppearance.Grey());
      }

      double soleYaw = 0.3;
      for (double soleX = 0; soleX < 1.0; soleX = soleX + 0.2)
      {
         for (double soleY = -1.8; soleY < -1.0; soleY = soleY + 0.2)
         {
            helper.testAPoint(soleX, soleY, soleYaw, assertPositionConditions, assertPointConditions);
         }
      }

      if (visualizeAndKeepUp)
      {
         ThreadTools.sleepForever();
      }
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 300000)
   public void testSimpleFootstepSnapperOnListOfPoints() throws InsufficientDataException, IOException
   {
      boolean assertPositionConditions = false;
      boolean assertPointConditions = true;
      boolean visualizeAndKeepUp = false;


      double minX = -5.0f;
      double minY = -2.0f;
      double maxX = 6.0f;
      double maxY = 2.0f;
      Box bounds = new Box(minX, minY, maxX, maxY);

      float resolution = 0.025f;
      float heightThreshold = 0.01f;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;


      String filename = "resources/pointListsForTesting/pointList_ObstacleCourseStart_150125.pointList";

      int maxNumberOfPoints = 400000;
      QuadTreeForGroundReaderAndWriter quadTreeForGroundReaderAndWriter = new QuadTreeForGroundReaderAndWriter();
      ArrayList<Point3d> points = quadTreeForGroundReaderAndWriter.readPointsFromFile(filename, 0, maxNumberOfPoints, bounds, Double.POSITIVE_INFINITY);

//    SimpleFootstepSnapper footstepSnapper = createSimpleFootstepSnapper();
      ConvexHullFootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();
      double maskSafetyBuffer = 0.01;
      double boundingBoxDimension = 0.3;
      footstepSnapper.setUseMask(true, maskSafetyBuffer, boundingBoxDimension);

      BoundingBox2d rangeOfPointsToTest = new BoundingBox2d(minX, minY, maxX, maxY);
      FootstepSnapperTestHelper helper = new FootstepSnapperTestHelper("List of Points", footstepSnapper, new Graphics3DObject(), visualizeAndKeepUp);


      helper.createHeightMap(points, rangeOfPointsToTest, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
                             maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);

      if (visualizeAndKeepUp)
      {
         helper.drawPoints(points, resolution / 2.0, YoAppearance.Blue());
      }

//    helper.drawNodeBoundingBoxes(-0.1);
//    helper.drawHeightMap(minX, minY, maxX, maxY, resolution);
//    helper.drawAllPointsInQuadTree(resolution, YoAppearance.Purple());



      double soleYaw = 0.3;
      for (double soleX = 2.0; soleX < 4.0; soleX = soleX + 0.2)
      {
         for (double soleY = -0.5; soleY < 0.5; soleY = soleY + 0.2)
         {
            helper.testAPoint(soleX, soleY, soleYaw, assertPositionConditions, assertPointConditions);
         }
      }

      if (visualizeAndKeepUp)
      {
         ThreadTools.sleepForever();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testSimpleFootstepSnapperOnBumpyGround() throws InsufficientDataException
   {
      boolean assertPositionConditions = true;
      boolean assertPointConditions = false;
      boolean visualizeAndKeepUp = false;

      GroundProfile3D groundProfile = createBumpyGroundProfile();
      SimpleFootstepSnapper footstepSnapper = createSimpleFootstepSnapper();
      double maskSafetyBuffer = 0.01;
      double boundingBoxDimension = 0.3;
      footstepSnapper.setUseMask(true, maskSafetyBuffer, boundingBoxDimension);

      BoundingBox2d rangeOfPointsToTest = new BoundingBox2d(-1.0, -1.0, 1.0, 1.0);
      FootstepSnapperTestHelper helper = new FootstepSnapperTestHelper("Simple Bumpy Ground", footstepSnapper, null, visualizeAndKeepUp);

      double resolution = 0.02;
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;

      helper.createHeightMap(groundProfile.getHeightMapIfAvailable(), rangeOfPointsToTest, resolution, heightThreshold,
                             quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose,
                             maxNodes);

      int numberOfPointsToTest = 1000;
      helper.testRandomPoints(numberOfPointsToTest, rangeOfPointsToTest, assertPositionConditions, assertPointConditions);

      if (visualizeAndKeepUp)
      {
         ThreadTools.sleepForever();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSimpleFootstepSnapperOnSteps() throws InsufficientDataException
   {
      boolean assertPositionConditions = true;
      boolean assertPointConditions = false;
      boolean visualizeAndKeepUp = false;

      CombinedTerrainObject3D groundProfile = createStepsGroundProfile();
      SimpleFootstepSnapper footstepSnapper = createSimpleFootstepSnapper();

//    FootstepSnapper footstepSnapper = createComplexFootstepSnapper();

//    double maskSafetyBuffer = 0.01;
//    double boundingBoxDimension = 0.3;
//    footstepSnapper.setUseMask(true, maskSafetyBuffer, boundingBoxDimension);

      double centerX = -3.5;
      double centerY = 3.5;
      double halfWidth = 0.6;

      BoundingBox2d rangeOfPointsToTest = new BoundingBox2d(centerX - halfWidth, centerY - halfWidth, centerX + halfWidth, centerY + halfWidth);
      FootstepSnapperTestHelper helper = new FootstepSnapperTestHelper("Steps", footstepSnapper, groundProfile.getLinkGraphics(), visualizeAndKeepUp);

      double resolution = 0.02;
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;

      helper.createHeightMap(groundProfile, rangeOfPointsToTest, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise,
                             maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);

//    double soleX = -3.2339;
//    double soleY = 4.0189;
//    double soleYaw = -1.424;
//    helper.testAPoint(soleX, soleY, soleYaw, assertConditions);

      int numberOfPointsToTest = 1000;
      helper.testRandomPoints(numberOfPointsToTest, rangeOfPointsToTest, assertPositionConditions, assertPointConditions);

      if (visualizeAndKeepUp)
      {
         ThreadTools.sleepForever();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testConvexHullFootstepSnapperOnSteps() throws InsufficientDataException
   {
      boolean assertPositionConditions = true;
      boolean assertPointConditions = true;
      boolean visualizeAndKeepUp = false;

      CombinedTerrainObject3D groundProfile = createStepsGroundProfile();
      FootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();


      double centerX = -3.5;
      double centerY = 3.5;
      double halfWidth = 0.6;

      BoundingBox2d rangeOfPointsToTest = new BoundingBox2d(centerX - halfWidth, centerY - halfWidth, centerX + halfWidth, centerY + halfWidth);
      FootstepSnapperTestHelper helper = new FootstepSnapperTestHelper("Steps", footstepSnapper, groundProfile.getLinkGraphics(), visualizeAndKeepUp);

      double resolution = 0.02;
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;

      helper.createHeightMap(groundProfile, rangeOfPointsToTest, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise,
                             maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);

//    double soleX = -3.2339;
//    double soleY = 4.0189;
//    double soleYaw = -1.424;
//    helper.testAPoint(soleX, soleY, soleYaw, assertConditions);

      int numberOfPointsToTest = 1000;
      helper.testRandomPoints(numberOfPointsToTest, rangeOfPointsToTest, assertPositionConditions, assertPointConditions);

      if (visualizeAndKeepUp)
      {
         ThreadTools.sleepForever();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.3)
   @Test(timeout = 30000)
   public void testConvexHullFootstepSnapperOnOddTerrain() throws InsufficientDataException
   {
      boolean assertPositionConditions = true;
      boolean assertPointConditions = true;
      boolean visualizeAndKeepUp = false;

      CombinedTerrainObject3D groundProfile = createOddTerrainProfile();
      FootstepSnapper footstepSnapper = createConvexHullFootstepSnapper();


      double centerX = 0.5;
      double centerY = 0;
      double halfWidth = 0.6;

      BoundingBox2d rangeOfPointsToTest = new BoundingBox2d(centerX - halfWidth, centerY - halfWidth, centerX + halfWidth, centerY + halfWidth);
      FootstepSnapperTestHelper helper = new FootstepSnapperTestHelper("Steps", footstepSnapper, groundProfile.getLinkGraphics(), visualizeAndKeepUp);

      double resolution = 0.02;
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;

      helper.createHeightMap(groundProfile, rangeOfPointsToTest, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise,
                             maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);

//
//    double soleX = 0.024122828598846582; //0.03;
//    double soleY = -0.26659012901268947;
//    double soleYaw = 1.507840381596421; //Math.PI/2 + 0.05;
//    helper.testAPoint(soleX, soleY, soleYaw, assertPositionConditions, assertPointConditions);

      int numberOfPointsToTest = 1000;
      helper.testRandomPoints(numberOfPointsToTest, rangeOfPointsToTest, assertPositionConditions, assertPointConditions);

      if (visualizeAndKeepUp)
      {
         ThreadTools.sleepForever();
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testAdjustingFootstepSnapperOnOddTerrain() throws InsufficientDataException
   {
      boolean assertPositionConditions = false;
      boolean assertPointConditions = true;
      boolean visualizeAndKeepUp = false;

      CombinedTerrainObject3D groundProfile = createOddTerrainProfile();
      FootstepSnapper footstepSnapper = createAdjustingFootstepSnapper();


      double centerX = 0.5;
      double centerY = 0;
      double halfWidth = 0.6;

      BoundingBox2d rangeOfPointsToTest = new BoundingBox2d(centerX - halfWidth, centerY - halfWidth, centerX + halfWidth, centerY + halfWidth);
      FootstepSnapperTestHelper helper = new FootstepSnapperTestHelper("Steps", footstepSnapper, groundProfile.getLinkGraphics(), visualizeAndKeepUp);

      double resolution = 0.02;
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;

      helper.createHeightMap(groundProfile, rangeOfPointsToTest, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise,
                             maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);

//
//    double soleX = 0.024122828598846582; //0.03;
//    double soleY = -0.26659012901268947;
//    double soleYaw = 1.507840381596421; //Math.PI/2 + 0.05;
//    helper.testAPoint(soleX, soleY, soleYaw, assertPositionConditions, assertPointConditions);

      int numberOfPointsToTest = 1000;
      helper.testRandomPoints(numberOfPointsToTest, rangeOfPointsToTest, assertPositionConditions, assertPointConditions);

      if (visualizeAndKeepUp)
      {
         ThreadTools.sleepForever();
      }
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

   private ConvexHullFootstepSnapper createConvexHullFootstepSnapper()
   {
      FootstepSnappingParameters snappingParameters = new GenericFootstepSnappingParameters();
      ConvexHullFootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);

      return footstepSnapper;
   }

   private AdjustingFootstepSnapper createAdjustingFootstepSnapper()
   {
      FootstepSnappingParameters snappingParameters = new GenericFootstepSnappingParameters();
      AdjustingFootstepSnapper footstepSnapper = new AdjustingFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);

      return footstepSnapper;
   }

   private GroundProfile3D createBumpyGroundProfile()
   {
      double xAmp1 = 0.4;
      double xFreq1 = 0.1;
      double xAmp2 = 0.01;
      double xFreq2 = 0.01;
      double yAmp1 = 0.01;
      double yFreq1 = 0.2;
      double yAmp2 = 0.01;
      double yFreq2 = 0.033;
      GroundProfile3D groundProfile = new BumpyGroundProfile(xAmp1, xFreq1, xAmp2, xFreq2, yAmp1, yFreq1, yAmp2, yFreq2);

      return groundProfile;
   }

   private class FootstepPointsDataReader
   {
      BufferedReader bufferedReader;
      boolean nextLineRead = false;

      public FootstepPointsDataReader(InputStream dataFile)
      {
         bufferedReader = new BufferedReader(new InputStreamReader(dataFile));
      }

      public boolean hasAnotherFootstepAndPoints() throws IOException
      {
         if (nextLineRead)
         {
            return true;
         }

         if (bufferedReader.readLine() != null)
         {
            nextLineRead = true;

            return true;
         }

         return false;
      }

      public List<Point3d> getNextSetPointsAndFootstep(FootstepDataMessage footstepData) throws IOException
      {
         if (hasAnotherFootstepAndPoints())
         {
            String inputLine;
            List<Point3d> listOfPoints = new ArrayList<Point3d>();
            Point3d position = new Point3d();
            Quat4d orientation = new Quat4d();
            double yaw;

            nextLineRead = false;

            while ((inputLine = bufferedReader.readLine()) != null)
            {
               StringTokenizer tokenizer = new StringTokenizer(inputLine, "(,:) ");
               int numberOfTokens = tokenizer.countTokens();
               if (numberOfTokens == 2)
               {
                  nextLineRead = true;

                  break;
               }

               if (numberOfTokens == 3)
               {
                  double x = Double.parseDouble(tokenizer.nextToken());
                  double y = Double.parseDouble(tokenizer.nextToken());
                  double z = Double.parseDouble(tokenizer.nextToken());
                  listOfPoints.add(new Point3d(x, y, z));
               }

               if (numberOfTokens == 12)
               {
                  for (int i = 0; i < 4; i++)
                  {
                     tokenizer.nextToken();
                  }

                  String footside = tokenizer.nextToken();
                  if (footside == "l_foot")
                     footstepData.robotSide = RobotSide.LEFT;
                  if (footside == "r_foot")
                     footstepData.robotSide = RobotSide.RIGHT;

                  for (int i = 5; i < 8; i++)
                  {
                     tokenizer.nextToken();
                  }

                  position.setX(Double.parseDouble(tokenizer.nextToken()));
                  position.setY(Double.parseDouble(tokenizer.nextToken()));
                  position.setZ(Double.parseDouble(tokenizer.nextToken()));
                  footstepData.location = position;
               }

               if (numberOfTokens == 9)
               {
                  tokenizer.nextToken();
                  yaw = Double.parseDouble(tokenizer.nextToken());
                  RotationTools.computeQuaternionFromYawAndZNormal(yaw, new Vector3d(0.0, 0.0, 1.0), orientation);
                  footstepData.orientation = (orientation);
               }
            }

            return listOfPoints;
         }

         footstepData = null;

         return null;
      }
   }


   protected class FootstepSnapperTestHelper
   {
      private final FootstepSnapper footstepSnapper;
      private final Random random = new Random(1776L);

      private QuadTreeForGroundHeightMap heightMap;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final SimulationConstructionSet scs;
      private final FootstepVisualizer footstepVisualizer;

      private final boolean visualize;

      private final YoVariableRegistry registry = new YoVariableRegistry("HeightMapBestFitPlaneCalculatorTest");
      private final DoubleYoVariable soleX = new DoubleYoVariable("soleX", registry);
      private final DoubleYoVariable soleY = new DoubleYoVariable("soleY", registry);
      private final DoubleYoVariable soleZ = new DoubleYoVariable("soleZ", registry);
      private final DoubleYoVariable soleYaw = new DoubleYoVariable("soleYaw", registry);
      private final YoFramePose planePose = new YoFramePose("planePose", "", worldFrame, registry);
      private final YoFramePoint queryPoint = new YoFramePoint("query", "", worldFrame, registry);
      private final YoFramePoint planePoint = new YoFramePoint("planePoint", "", worldFrame, registry);
      private BagOfBalls pointListBalls = null;


      public FootstepSnapperTestHelper(String description, FootstepSnapper footstepSnapper, Graphics3DObject linkGraphics, boolean visualize)
              throws InsufficientDataException
      {
         this.footstepSnapper = footstepSnapper;
         this.visualize = visualize;

         if (visualize)
         {
            int maxNumberOfFootstepsPerSide = 10;
            int maxContactPointsPerFoot = 8;

//          footstepVisualizer = new FootstepVisualizer(null, null, maxNumberOfFootstepsPerSide, maxContactPointsPerFoot, description);
            footstepVisualizer = new FootstepVisualizer(null, linkGraphics, maxNumberOfFootstepsPerSide, maxContactPointsPerFoot, description);
            scs = footstepVisualizer.getSimulationConstructionSet();    // new SimulationConstructionSet(robot);

            Robot robot = footstepVisualizer.getRobot();
            robot.getRobotsYoVariableRegistry().addChild(registry);
            YoGraphicsListRegistry yoGraphicsListRegistry = footstepVisualizer.getGraphicsListRegistry();

            ConvexPolygon2d polygon2d = new ConvexPolygon2d(new double[][]
            {
               {0.1, 0.1}, {0.1, -0.1}, {-0.1, -0.1}, {-0.1, 0.1}
            });

            YoFrameConvexPolygon2d yoFrameConvexPolygon2d = new YoFrameConvexPolygon2d("plane", "", worldFrame, 4, registry);
            yoFrameConvexPolygon2d.setConvexPolygon2d(polygon2d);

//          YoGraphicPolygon polygonViz = new YoGraphicPolygon("plane", yoFrameConvexPolygon2d, planePose, 1.0, YoAppearance.Gold());
//          yoGraphicsListRegistry.registerYoGraphic("Plane", polygonViz);

            YoGraphicPosition queryPointViz = new YoGraphicPosition("queryPoint", queryPoint, 0.02, YoAppearance.Red());
            yoGraphicsListRegistry.registerYoGraphic("Plane", queryPointViz);

            YoGraphicPosition planePointViz = new YoGraphicPosition("planePoint", planePoint, 0.02, YoAppearance.Green());
            yoGraphicsListRegistry.registerYoGraphic("Plane", planePointViz);

            pointListBalls = new BagOfBalls(1000, 0.01, registry, yoGraphicsListRegistry);
            footstepVisualizer.startVisualizer();
         }
         else
         {
            footstepVisualizer = null;
            scs = null;
         }
      }




      public void createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap, BoundingBox2d testingRange, double resolution, double heightThreshold,
                                  double quadTreeMaxMultiLevelZChangeToFilterNoise, int maxSameHeightPointsPerNode,
                                  double maxAllowableXYDistanceForAPointToBeConsideredClose, int maxNodes)
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

               if (visualize)
               {
                  Graphics3DObject staticLinkGraphics = new Graphics3DObject();
                  staticLinkGraphics.translate(new Vector3d(x, y, z + 0.001));
                  staticLinkGraphics.addCube(0.002, 0.002, 0.002, YoAppearance.Blue());
                  scs.addStaticLinkGraphics(staticLinkGraphics);
               }
            }
         }

         createHeightMap(listOfPoints, testingRange, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
                         maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);
      }


      public void createHeightMap(ArrayList<Point3d> listOfPoints, BoundingBox2d testingRange, double resolution, double heightThreshold,
                                  double quadTreeMaxMultiLevelZChangeToFilterNoise, int maxSameHeightPointsPerNode,
                                  double maxAllowableXYDistanceForAPointToBeConsideredClose, int maxNodes)
      {
         double minX = testingRange.getMinPoint().getX();
         double maxX = testingRange.getMaxPoint().getX();
         double minY = testingRange.getMinPoint().getY();
         double maxY = testingRange.getMaxPoint().getY();

         Box bounds = new Box(minX, minY, maxX, maxY);
         QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(resolution, heightThreshold,
                                                             quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
                                                             maxAllowableXYDistanceForAPointToBeConsideredClose, -1);
         heightMap = new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);

         for (Point3d point : listOfPoints)
         {
            heightMap.addPoint(point.getX(), point.getY(), point.getZ());
         }
      }

      public Graphics3DNode drawPoints(ArrayList<Point3d> points, double resolution, AppearanceDefinition appearance)
      {
         return QuadTreeHeightMapVisualizer.drawPoints(scs, points, resolution, appearance);
      }

      private Graphics3DNode drawHeightMap(double minX, double minY, double maxX, double maxY, float resolution)
      {
         return QuadTreeHeightMapVisualizer.drawHeightMap(heightMap, scs, minX, minY, maxX, maxY, resolution);
      }

      private Graphics3DNode drawAllPointsInQuadTree(double sizeToDrawCubes, AppearanceDefinition appearance)
      {
         return QuadTreeHeightMapVisualizer.drawAllPointsInQuadTree(heightMap, sizeToDrawCubes, scs, appearance);
      }

      private void drawNodeBoundingBoxes(double heightToDrawAt)
      {
         QuadTreeHeightMapVisualizer.drawNodeBoundingBoxes(heightMap, scs, heightToDrawAt);
      }

      private void testAPoint(double soleX, double soleY, double soleYaw, boolean assertPositionConditions, boolean assertPointConditions)
              throws InsufficientDataException
      {
         this.soleX.set(soleX);
         this.soleY.set(soleY);
         this.soleYaw.set(soleYaw);

         testAPoint(assertPositionConditions, assertPointConditions);
      }

      public void testAPoint(double soleX, double soleY, double soleYaw, boolean assertPositionConditions, boolean assertPointConditions, ContactablePlaneBody footstepBody)
            throws InsufficientDataException
      {
         this.soleX.set(soleX);
         this.soleY.set(soleY);
         this.soleYaw.set(soleYaw);

         testAPoint(assertPositionConditions, assertPointConditions, footstepBody);
      }

      private void testRandomPoints(int numberOfPointsToTest, BoundingBox2d rangeOfPointsToTest, boolean assertPositionConditions,
                                    boolean assertPointConditions)
              throws InsufficientDataException
      {
         double minX = rangeOfPointsToTest.getMinPoint().getX();
         double maxX = rangeOfPointsToTest.getMaxPoint().getX();

         double minY = rangeOfPointsToTest.getMinPoint().getY();
         double maxY = rangeOfPointsToTest.getMaxPoint().getY();

         for (int i = 0; i < numberOfPointsToTest; i++)
         {
            soleX.set(RandomTools.generateRandomDouble(random, minX, maxX));
            soleY.set(RandomTools.generateRandomDouble(random, minY, maxY));
            soleYaw.set(RandomTools.generateRandomDouble(random, Math.PI));

            testAPoint(assertPositionConditions, assertPointConditions);
         }
      }

      private void testAPoint(boolean assertPositionConditions, boolean assertPointConditions) throws InsufficientDataException
      {
         FootSpoof footSpoof = new FootSpoof("footSpoof");
         testAPoint(assertPositionConditions, assertPointConditions, footSpoof);
      }

      public void testAPoint(boolean assertPositionConditions, boolean assertPointConditions, ContactablePlaneBody footstepBody) throws InsufficientDataException
      {
         queryPoint.set(soleX.getDoubleValue(), soleY.getDoubleValue(), 0.2);

         RobotSide robotSide = RobotSide.generateRandomRobotSide(random);

         Footstep generatedSnappedFootstep = footstepSnapper.generateSnappedFootstep(soleX.getDoubleValue(), soleY.getDoubleValue(), soleYaw.getDoubleValue(),
               footstepBody.getRigidBody(), footstepBody.getSoleFrame(), robotSide, heightMap);

         ReferenceFrame soleFrame = generatedSnappedFootstep.getSoleReferenceFrame();
         FramePoint solePosition = new FramePoint(soleFrame);
         FrameOrientation soleOrientation = new FrameOrientation(soleFrame);
         FrameVector soleNormal = new FrameVector(soleFrame, 0.0, 0.0, 1.0);
         solePosition.changeFrame(worldFrame);
         soleOrientation.changeFrame(worldFrame);
         soleNormal.changeFrame(worldFrame);

         if (assertPositionConditions)
         {
            assertEquals(soleX.getDoubleValue(), solePosition.getX(), 1e-7);
            assertEquals(soleY.getDoubleValue(), solePosition.getY(), 1e-7);
            assertEquals(soleYaw.getDoubleValue(), soleOrientation.getYaw(), 1e-7);
         }

         soleZ.set(solePosition.getZ());

         Point3d planePosition = solePosition.getPointCopy();
         Vector3d planeNormal = soleNormal.getVectorCopy();

         planePose.setPosition(planePosition);

         Quat4d planeOrientation = new Quat4d();
         RotationTools.computeQuaternionFromYawAndZNormal(soleYaw.getDoubleValue(), planeNormal, planeOrientation);
         planePose.setOrientation(planeOrientation);

//       planePosition.set(planePosition.x, planePosition.y, planePosition.z - 0.01); adversary code, move foot plane down a little
         planePoint.set(planePosition);
         Plane3d solePlane = new Plane3d(planePosition, planeNormal);

         if (assertPointConditions)
         {
            double tolerance = 0.01;
            assertTrue((generatedSnappedFootstep.getPredictedContactPoints() == null) || (generatedSnappedFootstep.getPredictedContactPoints().size() > 2));
            boolean pointsBelowPlane = pointsBelowPlane(footstepSnapper.getPointList(), solePlane, tolerance);
            assertTrue("queryPoint = " + queryPoint + " yaw = " + soleYaw.getDoubleValue() + " planeNormal = " + planeNormal + ", pointsBelowPlane = "
                       + pointsBelowPlane, (planeNormal.getZ() >= 0.98) || pointsBelowPlane);
         }

         if (visualize)
         {
            pointListBalls.reset();

            List<Point3d> pointList = footstepSnapper.getPointList();
            for (Point3d point : pointList)
            {
               double heightMapZ = heightMap.getHeightAtPoint(point.getX(), point.getY());
               pointListBalls.setBall(new FramePoint(worldFrame, point.getX(), point.getY(), point.getZ()));

//             pointListBalls.setBall(new FramePoint(worldFrame, point.getX(), point.getY(), heightMapZ));
            }

            footstepVisualizer.visualizeFootstep(footstepBody, generatedSnappedFootstep);

            scs.tickAndUpdate();
         }
      }
   }


   private boolean pointsBelowPlane(List<Point3d> point3ds, Plane3d plane, double tolerance)
   {
      for (Point3d point3d : point3ds)
      {
         if (!pointBelowPlane(point3d, plane, tolerance))
         {
            return false;
         }
      }

      return true;
   }

   private boolean pointBelowPlane(Point3d point, Plane3d plane, double tolerance)
   {
      double distanceAlongNormal = plane.signedDistance(point);
      if (distanceAlongNormal > tolerance)
         return false;

      return true;
   }


   private CombinedTerrainObject3D createSingleBoxGroundProfile()
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("stairs");

      AppearanceDefinition color = YoAppearance.DarkGray();
      combinedTerrainObject.addBox(0.0, 0.0, 1.0, 1.0, 1.0, color);
      combinedTerrainObject.addBox(-100.0, -100.0, 100.0, 100.0, 0.001, color);

      return combinedTerrainObject;
   }

   private CombinedTerrainObject3D createOddTerrainProfile()
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("stairs");

      AppearanceDefinition color = YoAppearance.DarkGray();
      combinedTerrainObject.addBox(-100.0, -100.0, 100.0, 100.0, 0.001, color);

      combinedTerrainObject.addBox(0.0, 0.0, 0.5, 0.5, 0.5, color);
      combinedTerrainObject.addBox(0.57, 0.00, 1.0, 0.1, 0.5, color);
      combinedTerrainObject.addBox(0.57, 0.15, 1.0, 0.2, 0.5, color);
      combinedTerrainObject.addBox(0.57, 0.25, 1.0, 0.3, 0.5, color);
      combinedTerrainObject.addBox(0.57, 0.35, 1.0, 0.4, 0.5, color);
      combinedTerrainObject.addBox(0.57, 0.45, 1.0, 0.5, 0.5, color);
      combinedTerrainObject.addRamp(0, -0.5, 0.8, 0.0, 0.5, color);
      combinedTerrainObject.addBox(.8, -0.5, 1.0, 0.0, 0.5, color);

      return combinedTerrainObject;
   }


   private CombinedTerrainObject3D createStepsGroundProfile()
   {
      CombinedTerrainObject3D combinedTerrainObject = new CombinedTerrainObject3D("stairs");

      AppearanceDefinition color = YoAppearance.DarkGray();
      double courseAngle = 135;
      int numberOfSteps = 3;
      double rise = 0.2;
      double startDistance = 4.0;
      double run = 0.4;

      for (int i = 0; i < numberOfSteps; i++)
      {
         double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (i * run), 0}, courseAngle);
         setUpWall(combinedTerrainObject, newPoint, 3.0, run, rise * (i + 1), courseAngle, color);
      }

      {
         double[] newPoint = rotateAroundOrigin(new double[] {startDistance + (numberOfSteps * run), 0}, courseAngle);
         setUpWall(combinedTerrainObject, newPoint, 3.0, run, rise * (numberOfSteps - 1 + 1), courseAngle, color);
      }

      for (int i = 1; i < numberOfSteps + 1; i++)
      {
         double offset = numberOfSteps * run;
         double[] newPoint = rotateAroundOrigin(new double[] {offset + startDistance + (i * run), 0}, courseAngle);
         setUpWall(combinedTerrainObject, newPoint, 3.0, run, rise * (-i + numberOfSteps + 1), courseAngle, color);
      }

      return combinedTerrainObject;
   }

   private static double[] rotateAroundOrigin(double[] xy, double angdeg)
   {
      double x = xy[0];
      double y = xy[1];
      double[] newPoint = new double[2];
      double angRad = Math.toRadians(angdeg);
      newPoint[0] = x * Math.cos(angRad) - y * Math.sin(angRad);
      newPoint[1] = y * Math.cos(angRad) + x * Math.sin(angRad);

      return newPoint;
   }

   private static void setUpWall(CombinedTerrainObject3D combinedTerrainObject, double[] xy, double width, double length, double height, double yawDegrees,
                                 AppearanceDefinition app)
   {
      double x = xy[0];
      double y = xy[1];
      RigidBodyTransform location = new RigidBodyTransform();
      location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

      location.setTranslation(new Vector3d(x, y, height / 2));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, length, width, height), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }

}
