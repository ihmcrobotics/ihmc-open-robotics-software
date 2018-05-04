package us.ihmc.atlas.roughTerrainWalking;

import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;

import org.junit.Test;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.commonWalkingControlModules.configurations.FootstepParameters;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.humanoidRobotics.footstep.FootSpoof;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.AtlasFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.ConvexHullFootstepSnapper;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.FootstepSnapperSimulationTest;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.QuadTreeFootstepSnappingParameters;
import us.ihmc.humanoidRobotics.footstep.footstepSnapper.SimpleFootstepValueFunction;
import us.ihmc.robotics.geometry.InsufficientDataException;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundReaderAndWriter;
import us.ihmc.commons.thread.ThreadTools;

/**
 * Created by agrabertilton on 3/4/15.
 */
public class AtlasFootstepSnapperTest extends FootstepSnapperSimulationTest
{
   @Override
   @ContinuousIntegrationTest(estimatedDuration = 1.0)
   @Test(timeout = 30000)
   public void testAdjustingFootstepSnapperOnOddTerrain() throws InsufficientDataException
   {
      super.testAdjustingFootstepSnapperOnOddTerrain();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testConvexHullFootstepSnapperOnOddTerrain() throws InsufficientDataException
   {
      super.testConvexHullFootstepSnapperOnOddTerrain();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testConvexHullFootstepSnapperOnSteps() throws InsufficientDataException
   {
      super.testConvexHullFootstepSnapperOnSteps();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testFootstepAndPointsFromDataFile() throws NumberFormatException, InsufficientDataException, IOException
   {
      super.testFootstepAndPointsFromDataFile();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testSimpleFootstepSnapperOnBumpyGround() throws InsufficientDataException
   {
      super.testSimpleFootstepSnapperOnBumpyGround();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 30000)
   public void testSimpleFootstepSnapperOnListOfPoints() throws InsufficientDataException, IOException
   {
      super.testSimpleFootstepSnapperOnListOfPoints();
   }

   @Override
   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testSimpleFootstepSnapperOnSteps() throws InsufficientDataException
   {
      super.testSimpleFootstepSnapperOnSteps();
   }

	@Override
   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testPointsFromAtlasDataFile() throws NumberFormatException, InsufficientDataException, IOException
   {
      boolean assertPositionConditions = true;
      boolean assertPointConditions = false;
      boolean visualizeAndKeepUp = false;
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.SCS, false);
      FootstepParameters atlasFootstepParameters = robotModel.getWalkingControllerParameters().getSteppingParameters();
      QuadTreeFootstepSnappingParameters snappingParameters = new AtlasFootstepSnappingParameters();
      FootSpoof footSpoof = new FootSpoof("footSpoof", 0.0, 0.0, 0.0, atlasFootstepParameters.getFootForwardOffset(), atlasFootstepParameters.getFootBackwardOffset(), atlasFootstepParameters.getToeWidth()/2.0, 0.0);

      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;

      double minX = -5.0;    // 5.0f;
      double minY = -2.0;    // 5.0f;
      double maxX = 0.0;    // 5.0f;
      double maxY = 1.0;    // 5.0f;
      Box bounds = new Box(minX, minY, maxX, maxY);

      float resolution = 0.025f;
      float heightThreshold = 0.005f;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.02;
      int maxNodes = 1000000;

      String resourceName = "lidarCaptures/LidarDefault_201503020358.txt";
      InputStream resourceAsStream = getClass().getClassLoader().getResourceAsStream(resourceName);

      double maxZ = 0.6;
      int skipPoints = 0;
      int maxNumberOfPoints = 2000000;

      QuadTreeForGroundReaderAndWriter quadTreeForGroundReaderAndWriter = new QuadTreeForGroundReaderAndWriter();
      ArrayList<Point3D> points = quadTreeForGroundReaderAndWriter.readPointsFromInputStream(resourceAsStream, skipPoints, maxNumberOfPoints, bounds, maxZ);

      ConvexHullFootstepSnapper footstepSnapper = new ConvexHullFootstepSnapper(new SimpleFootstepValueFunction(snappingParameters), snappingParameters);
      double maskSafetyBuffer = 0.01;
      double boundingBoxDimension = 0.3;
      footstepSnapper.setUseMask(true, maskSafetyBuffer, boundingBoxDimension);

      BoundingBox2D rangeOfPointsToTest = new BoundingBox2D(minX, minY, maxX, maxY);
      FootstepSnapperTestHelper helper = new FootstepSnapperTestHelper("RealCinderblocks", footstepSnapper, new Graphics3DObject(), visualizeAndKeepUp);


      helper.createHeightMap(points, rangeOfPointsToTest, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
            maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);

      if (visualizeAndKeepUp)
      {
         helper.drawPoints(points, resolution / 2.0, YoAppearance.Grey());
      }

      double soleYaw = 0.2;
      double soleX = -0.0;
      double soleY = 0;
      double horzontalOffset = 0.2;
      double originalDistance = 0.9;
      double stepLength = 0.4;
      //y = .2x

      for (int i = 0; i < 6; i++)
      {
         double xStart = -1 * (originalDistance + i * stepLength) * Math.cos(soleYaw);
         double yStart = 0.2 * xStart;
         //left foot
         helper.testAPoint(xStart + Math.sin(soleYaw)*horzontalOffset, yStart - Math.cos(soleYaw) * horzontalOffset, soleYaw, assertPositionConditions, assertPointConditions, footSpoof);

         //right foot
         helper.testAPoint(xStart - Math.sin(soleYaw)*horzontalOffset, yStart + Math.cos(soleYaw) * horzontalOffset, soleYaw, assertPositionConditions, assertPointConditions, footSpoof);
      }

      if (visualizeAndKeepUp)
      {
         ThreadTools.sleepForever();
      }
   }

}
