package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import org.junit.Ignore;
import org.junit.Test;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.graphics3DAdapter.structure.Graphics3DNode;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.dataStructures.quadTree.SimplifiedQuadTree;
import us.ihmc.utilities.math.dataStructures.HeightMap;
import us.ihmc.utilities.math.geometry.BoundingBox2d;
import us.ihmc.utilities.math.geometry.Box3d;
import us.ihmc.utilities.math.geometry.Plane3d;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.graphics.BagOfBalls;
import us.ihmc.yoUtilities.graphics.YoGraphicPosition;
import us.ihmc.yoUtilities.graphics.YoGraphicsListRegistry;
import us.ihmc.yoUtilities.math.frames.YoFramePoint;

public class SimplifiedGroundOnlyQuadTreeTest
{
   private static final boolean DO_ASSERTS = false;

   
   @Test
   public void testSimpleCaseOne()
   {
      float minX = -10.0f;
      float minY = -10.0f;
      float maxX = 10.0f;
      float maxY = 10.0f;
      float resolution = 0.49f;
      float heightThreshold = 0.001f;
      double maxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      SimplifiedQuadTree quadTree = new SimplifiedQuadTree(minX, minY, maxX, maxY, resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode);

      Double returnNullObject = quadTree.get(0.0f, 0.0f);
      assertNull(returnNullObject);

      // Put a single point at zero

      Double valueAtZero = new Double(1.5);
      quadTree.put(0.0f, 0.0f, valueAtZero);

      Double returnValueAtZero = quadTree.get(0.0f, 0.0f);
      assertEquals(valueAtZero, returnValueAtZero, 1e-7);

      Double returnValueOutOfBounds = quadTree.get(100.0f, -720.0f);
      assertNull(returnValueOutOfBounds);

      Double returnValueAwayFromZero = quadTree.get(3.0f, -7.2f);
      assertEquals(valueAtZero, returnValueAwayFromZero, 1e-7);

      // Put a point away from zero
      Double valueAtOneOne = new Double(2.7);
      quadTree.put(1.0f, 1.0f, valueAtOneOne);

      returnValueAtZero = quadTree.get(0.0f, 0.0f);
      assertEquals(valueAtZero, returnValueAtZero, 1e-7);

      Double returnValueAtOneOne = quadTree.get(1.0f, 1.0f);
      assertEquals(valueAtOneOne, returnValueAtOneOne, 1e-7);
   }
   
   
   @Ignore
   @Test
   public void testPointsFromAFile() throws NumberFormatException, IOException
   {
      double minX = -5.0f;
      double minY = -5.0f;
      double maxX = 5.0f;
      double maxY = 5.0f;
      float resolution = 0.02f;
      float heightThreshold = 0.002f;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxNodes = 1000000;

//      SimplifiedGroundOnlyQuadTree quadTree = new SimplifiedGroundOnlyQuadTree(minX, minY, maxX, maxY, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise );

      int maxBalls = 200;
      QuadTreeTestHelper testHelper = new QuadTreeTestHelper(new BoundingBox2d(minX, minY, maxX, maxY), maxBalls);
      testHelper.setResolutionParameters(resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxNodes);

//      String filename = "resources/pointListsForTesting/pointList_150121PM0235.pointList";
//      String filename = "resources/pointListsForTesting/pointList_150121PM05.pointList";
//      String filename = "resources/pointListsForTesting/pointListBadWalkingToTheLeft_150121PM0535.pointList";
      String filename = "resources/pointListsForTesting/pointList150122_DRCObstacleCourse.pointList";
     

      int maxNumberOfPoints = 400000;
      ArrayList<Point3d> points = SimplifiedGroundOnlyQuadTree.readPointsFromFile(filename, maxNumberOfPoints);

      
      int pointsPerBallUpdate = 10000;
      boolean drawPointsInBlue = false;
      testHelper.createHeightMapFromAListOfPoints(points, drawPointsInBlue , pointsPerBallUpdate);
      
//      testHelper.drawHeightOfOriginalPointsInPurple(points, 1);
      Graphics3DNode handle = testHelper.drawNodeBoundingBoxes(-0.1);
      
      
//      testHelper.drawHeightMap(minX, minY, maxX, maxY, resolution * 4.0);
      
      testHelper.displaySimulationConstructionSet();

      
      
//      testHelper.doATest(points, pointsPerBallUpdate);
      ThreadTools.sleepForever();
   }
   
   
   
 @Ignore
   @Test
   public void testThreePointsOnALine()
   {
      float minX = -0.04f;
      float minY = -0.04f;
      float maxX = 1.0f;
      float maxY = 0.06f;
      float resolution = 0.02f;
      float heightThreshold = 0.002f;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;

      //      CleanQuadTree quadTree = new CleanQuadTree(minX, minY, maxX, maxY, resolution, heightThreshold);
      SimplifiedGroundOnlyQuadTree quadTree = new SimplifiedGroundOnlyQuadTree(minX, minY, maxX, maxY, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode);

      float height0 = 0.0f;
      float height1 = 0.0f;
      float height2 = 0.2f;

      quadTree.put(0.0f, 0.0f, height0);
      quadTree.put(0.1f, 0.0f, height1);
      quadTree.put(0.2f, 0.0f, height2);

      double returnHeight0 = quadTree.get(0.0f, 0.0f);
      double returnHeight1 = quadTree.get(0.1f, 0.0f);
      double returnHeight2 = quadTree.get(0.2f, 0.0f);

      assertEquals(height0, returnHeight0, 1e-7);
      assertEquals(height1, returnHeight1, 1e-7);
      assertEquals(height2, returnHeight2, 1e-7);
   }

   @Ignore
   @Test
   public void testOnALineOfPoints()
   {
      ArrayList<Point3d> points = new ArrayList<Point3d>();
      points.add(new Point3d(0.0, 0.0, 0.0));
      points.add(new Point3d(0.1, 0.0, 0.0));
      points.add(new Point3d(0.2, 0.0, 0.2));
//      points.add(new Point3d(0.3, 0.0, 0.2));

      double resolution = 0.02;

      BoundingBox2d boundingBox = new BoundingBox2d(-0.04, -0.04, 1.0, 0.06);
      int pointsPerBallUpdate = 1;
      testOnAListOfPoints(points, pointsPerBallUpdate, boundingBox, resolution);

      ThreadTools.sleepForever();
   }

   @Ignore
   @Test
   public void testOnSomeSlopes()
   {
      double halfWidth = 0.5;
      double resolution = 0.1;

      Point3d center = new Point3d(0.0, 0.0, 0.3);
      Vector3d normal = new Vector3d(0.1, 0.2, 0.8);
      testOnASlope(center, normal, halfWidth, resolution);

      center = new Point3d(0.0, 0.0, 0.3);
      normal = new Vector3d(1.0, 1.0, 1.0);
      testOnASlope(center, normal, halfWidth, resolution);

      center = new Point3d(0.0, 0.0, 0.3);
      normal = new Vector3d(-1.0, 1.0, 1.0);
      testOnASlope(center, normal, halfWidth, resolution);

//    ThreadTools.sleepForever();
   }

   @Ignore
   @Test
   public void testOnSomeStairCases()
   {
      double halfWidth = 0.6;
      double resolution = 0.02;

      Point3d center = new Point3d(0.0, 0.0, 0.3);
      double stairSeparation = 0.2;
      double oneStairLandingHeight = 0.0;

      Vector3d normal = new Vector3d(0.3, -0.3, 1.0);
      testOnAStaircase(center, normal, halfWidth, resolution, stairSeparation, oneStairLandingHeight);

//    normal = new Vector3d(0.3, 0.3, 1.0);
//    testOnAStaircase(center, normal, halfWidth, resolution, stairSeparation, oneStairLandingHeight);
//    
//    normal = new Vector3d(-0.3, 0.3, 1.0);
//    testOnAStaircase(center, normal, halfWidth, resolution, stairSeparation, oneStairLandingHeight);
//    
//    normal = new Vector3d(-0.3, -0.3, 1.0);
//    testOnAStaircase(center, normal, halfWidth, resolution, stairSeparation, oneStairLandingHeight);

      ThreadTools.sleepForever();
   }

   @Ignore
   @Test
   public void testUsingStairGroundProfile()
   {
      CombinedTerrainObject3D groundProfile = createStepsGroundProfile();

      double centerX = -3.5;
      double centerY = 3.5;
      double halfWidth = 0.6;

      double minX = centerX - halfWidth;
      double minY = centerY - halfWidth;
      double maxX = centerX + halfWidth;
      double maxY = centerY + halfWidth;

      BoundingBox2d boundingBox = new BoundingBox2d(minX, minY, maxX, maxY);

      double resolution = 0.02;
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxNodes = 1000000;

      QuadTreeTestHelper testHelper = new QuadTreeTestHelper(boundingBox, (int) ((halfWidth/resolution) * (halfWidth/resolution)));      
      testHelper.setResolutionParameters(resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxNodes);

      ArrayList<Point3d> points = testHelper.createAListOfPointsFromAGroundProfile(groundProfile, minX, minY, maxX, maxY, resolution);
      int pointsPerBallUpdate = 1;
      testHelper.doATest(points, pointsPerBallUpdate);

      // TODO: Get this to pass!
      testHelper.assertPointsLieOnHeightMap(points);

      ThreadTools.sleepForever();
   }


   private void testOnAStaircase(Point3d center, Vector3d normal, double halfWidth, double resolution, double stairSeparation, double oneStairLandingHeight)
   {
      normal.normalize();

      BoundingBox2d boundingBox = new BoundingBox2d(center.getX() - halfWidth, center.getY() - halfWidth, center.getX() + halfWidth, center.getY() + halfWidth);
      Plane3d plane3d = new Plane3d(center, normal);
      ArrayList<Point3d> points = generatePointsForStairs(plane3d, halfWidth, resolution, stairSeparation, oneStairLandingHeight);

//      Collections.shuffle(points);
      
      int pointsPerBallUpdate = 1;
      testOnAListOfPoints(points, pointsPerBallUpdate, boundingBox, resolution);
   }

   private void testOnASlope(Point3d center, Vector3d normal, double halfWidth, double resolution)
   {
      normal.normalize();

      BoundingBox2d boundingBox = new BoundingBox2d(center.getX() - halfWidth, center.getY() - halfWidth, center.getX() + halfWidth, center.getY() + halfWidth);
      Plane3d plane3d = new Plane3d(center, normal);
      ArrayList<Point3d> points = generatePointsForSlope(plane3d, halfWidth, resolution);
      
      int pointsPerBallUpdate = 1;
      testOnAListOfPoints(points, pointsPerBallUpdate, boundingBox, resolution);
   }

   private void testOnAListOfPoints(ArrayList<Point3d> points, int pointsPerBallUpdate, BoundingBox2d rangeOfPoints, double resolution)
   {
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxNodes = 1000000;

      QuadTreeTestHelper testHelper = new QuadTreeTestHelper(rangeOfPoints, points.size());
      testHelper.setResolutionParameters(resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxNodes);

      testHelper.doATest(points, pointsPerBallUpdate);
      testHelper.assertPointsLieOnHeightMap(points);
   }


   private static ArrayList<Point3d> generatePointsForStairs(Plane3d plane3d, double halfWidth, double stepSize, double stairSeparation,
           double oneStairLandingHeight)
   {
      ArrayList<Point3d> ret = generatePointsForSlope(plane3d, halfWidth, stepSize);
      formStaircaseWithPointsOnAPlane(ret, stairSeparation, oneStairLandingHeight);

      return ret;
   }

   private static ArrayList<Point3d> generatePointsForSlope(Plane3d plane3d, double halfWidth, double stepSize)
   {
      Point3d centerPoint = plane3d.getPointCopy();

      double minX = centerPoint.getX() - halfWidth;
      double minY = centerPoint.getY() - halfWidth;
      double maxX = centerPoint.getX() + halfWidth;
      double maxY = centerPoint.getY() + halfWidth;

      ArrayList<Point3d> points = new ArrayList<Point3d>();

      for (double x = minX; x < maxX; x = x + stepSize)
      {
         for (double y = minY; y < maxY; y = y + stepSize)
         {
            double z = plane3d.getZOnPlane(x, y);
            points.add(new Point3d(x, y, z));
         }
      }

      return points;
   }

   private static void formStaircaseWithPointsOnAPlane(ArrayList<Point3d> pointsList, double stairSeparation, double oneStairLandingHeight)
   {
      for (Point3d point3d : pointsList)
      {
         double z = point3d.getZ();

         double newZ = Math.floor((z - oneStairLandingHeight) / stairSeparation) * stairSeparation;
         point3d.setZ(newZ);
      }
   }


   private static class QuadTreeTestHelper
   {
      private Robot robot = new Robot("TestQuadTree");
      private SimulationConstructionSet scs = new SimulationConstructionSet(robot);
      private YoVariableRegistry registry = robot.getRobotsYoVariableRegistry();
      
      private final BagOfBalls bagOfBalls;
      private final YoFramePoint queryPoint;
      
//      private SimplifiedGroundOnlyQuadTree heightMap;
      private QuadTreeHeightMapInterface heightMap;
//    private final double centerX, centerY;
//    private final double minX, minY, maxX, maxY;
      private final BoundingBox2d rangeOfPointsToTest;

      private double resolution = 0.1;
      private double heightThreshold = 0.002;
      private double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      private int maxSameHeightPointsPerNode = 20;
      private int maxNodes = 1000000;

      
      public QuadTreeTestHelper(BoundingBox2d rangeOfPointsToTest, int maxNumberOfBallsInBag)
      {
         //         int maxBagOfBallsSize = 500;
         //         int numberOfBalls = Integer.min(points.size(), maxBagOfBallsSize);

         scs.setGroundVisible(false);
         this.rangeOfPointsToTest = rangeOfPointsToTest;

         YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

         double ballSize = resolution * 0.35;

         bagOfBalls = new BagOfBalls(maxNumberOfBallsInBag, ballSize, registry, yoGraphicsListRegistry);
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         queryPoint = new YoFramePoint("queryPoint", worldFrame, registry);

         YoGraphicPosition queryViz = new YoGraphicPosition("Query", queryPoint, 1.1 * ballSize, YoAppearance.Red());
         yoGraphicsListRegistry.registerYoGraphic("Query", queryViz);
         scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      }

      public void setResolutionParameters(double resolution, double heightThreshold, double quadTreeMaxMultiLevelZChangeToFilterNoise, int maxNodes)
      {
         this.resolution = resolution;
         this.heightThreshold = heightThreshold;
         this.quadTreeMaxMultiLevelZChangeToFilterNoise = quadTreeMaxMultiLevelZChangeToFilterNoise;
         this.maxNodes = maxNodes;
      }

      public HeightMap getHeightMap()
      {
         return heightMap;
      }

      public void doATest(ArrayList<Point3d> points, int pointsPerBallUpdate)
      {
         boolean drawPointsInBlue = true;
         createHeightMapFromAListOfPoints(points, drawPointsInBlue , pointsPerBallUpdate);
         drawPointsWithinAreaInSCS(pointsPerBallUpdate);
         drawHeightOfOriginalPointsInPurple(points, pointsPerBallUpdate);
         displaySimulationConstructionSet();
      }
      
      private void displaySimulationConstructionSet()
      {
         scs.startOnAThread();
      }

      
      
      private void drawHeightMap(double minX, double minY, double maxX, double maxY, double resolution)
      {
         AppearanceDefinition[] rainbow = YoAppearance.getStandardRoyGBivRainbow();

         Graphics3DObject heightMapGraphic = new Graphics3DObject();

         for (double x = minX; x<maxX; x = x + resolution)
         {
            for (double y = minY; y<maxY; y = y + resolution)
            {
               double z = heightMap.heightAtPoint(x, y);

               if (!Double.isNaN(z))
               {
                  AppearanceDefinition appearance = rainbow[((int) (z / resolution)) % rainbow.length];

                  heightMapGraphic.identity();
                  heightMapGraphic.translate(x, y, z);
                  heightMapGraphic.addCube(resolution, resolution, resolution, appearance);
               }
            }
         }
         scs.addStaticLinkGraphics(heightMapGraphic);
      }
      
      private Graphics3DNode drawNodeBoundingBoxes(double heightToDrawAt)
      {
         if (heightMap instanceof SimplifiedGroundOnlyQuadTree)
         {
            Graphics3DNode handle = SimplifiedGroundOnlyQuadTreeVisualizer.drawNodeBoundingBoxes((SimplifiedGroundOnlyQuadTree) heightMap, scs, heightToDrawAt);
            return handle;
         }
         
         return null;
      }
      
      private void drawHeightOfOriginalPointsInPurple(ArrayList<Point3d> points, int pointsPerBallUpdate)
      {
         int count = 0;
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         for (Point3d point : points)
         {
            count ++;
            if (count >= pointsPerBallUpdate)
            {
               count = 0;

               double z = heightMap.heightAtPoint(point.getX(), point.getY());

               staticLinkGraphics.identity();
               staticLinkGraphics.translate(new Vector3d(point.getX(), point.getY(), z + 0.001));

               double cubeSize = resolution * 0.35;
               staticLinkGraphics.addCube(cubeSize, cubeSize, cubeSize / 3.0, YoAppearance.Purple());
            }
         }
         scs.addStaticLinkGraphics(staticLinkGraphics);
      }

      private void drawPointsWithinAreaInSCS(int pointsPerBallUpdate)
      {
         Point2d centerPoint = new Point2d();
         rangeOfPointsToTest.getCenterPointCopy(centerPoint);
         List<Point3d> allPointsWithinArea = heightMap.getAllPointsWithinArea(centerPoint.getX(), centerPoint.getY(), 10.0, 10.0);
         
         int count = 0;
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         for (Point3d point3d : allPointsWithinArea)
         {
            count ++;
            if (count >= pointsPerBallUpdate)
            {
               count = 0;

               //            Graphics3DObject staticLinkGraphics = new Graphics3DObject();
               staticLinkGraphics.identity();
               double cubeSize = resolution * 0.5;
               staticLinkGraphics.translate(new Vector3d(point3d.getX(), point3d.getY(), point3d.getZ() -cubeSize / 2.0 + 0.001));
               staticLinkGraphics.addCube(cubeSize, cubeSize, cubeSize, YoAppearance.Chartreuse());
            }
         }
         scs.addStaticLinkGraphics(staticLinkGraphics);
      }


      public void assertPointsLieOnHeightMap(ArrayList<Point3d> points)
      {
         if (DO_ASSERTS)
         {
            for (Point3d point : points)
            {
               double heightMapZ = heightMap.heightAtPoint(point.getX(), point.getY());
               assertEquals(point.getZ(), heightMapZ, 1e-7);
            }
         }
      }

      public ArrayList<Point3d> createAListOfPointsFromAGroundProfile(GroundProfile3D groundProfile, BoundingBox2d testingRange, double resolution)
      {
         double minX = testingRange.getMinPoint().getX();
         double maxX = testingRange.getMaxPoint().getX();
         double minY = testingRange.getMinPoint().getY();
         double maxY = testingRange.getMaxPoint().getY();

         return createAListOfPointsFromAGroundProfile(groundProfile, minX, minY, maxX, maxY, resolution);
      }

      public ArrayList<Point3d> createAListOfPointsFromAGroundProfile(GroundProfile3D groundProfile, double minX, double minY, double maxX, double maxY,
              double resolution)
      {
         ArrayList<Point3d> points = new ArrayList<Point3d>();
         for (double x = minX; x < maxX; x = x + resolution)
         {
            for (double y = minY; y < maxY; y = y + resolution)
            {
               double z = groundProfile.getHeightMapIfAvailable().heightAt(x, y, 0.0);
               points.add(new Point3d(x, y, z));
            }
         }

         return points;
      }


      private void createHeightMapFromAListOfPoints(ArrayList<Point3d> points, boolean drawPointsInBlue, int pointsPerBallUpdate)
      {
         double minX = rangeOfPointsToTest.getMinPoint().getX();
         double maxX = rangeOfPointsToTest.getMaxPoint().getX();
         double minY = rangeOfPointsToTest.getMinPoint().getY();
         double maxY = rangeOfPointsToTest.getMaxPoint().getY();

       
         heightMap = new SimplifiedGroundOnlyQuadTree(minX, minY, maxX, maxY, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode);
         
//      CleanQuadTreeHeightMap heightMap = new CleanQuadTreeHeightMap(minX, minY, maxX, maxY, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise);
//         CleanQuadTreeHeightMap heightMap = new CleanQuadTreeHeightMap(minX - resolution, minY - resolution, maxX + resolution, maxY + resolution, resolution,
//                                               heightThreshold);

//         heightMap.checkRepInvarients();
//       heightMap = new QuadTreeHeightMap(minX, minY, maxX, maxY, resolution, heightThreshold);
//       heightMap = new GroundOnlyQuadTree(rangeOfPointsToTest, resolution, heightThreshold, maxNodes);

         int pointsPerBall = 0;
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();

         for (Point3d point : points)
         {
            queryPoint.set(point);

            heightMap.addPoint(point.getX(), point.getY(), point.getZ());

            if (drawPointsInBlue)
            {
               staticLinkGraphics.identity();
               staticLinkGraphics.translate(new Vector3d(point.getX(), point.getY(), point.getZ() + 0.001));
               double cubeSize = resolution * 0.35;
               staticLinkGraphics.addCube(cubeSize, cubeSize, cubeSize / 3.0, YoAppearance.Blue());
            }
            
            pointsPerBall++;
            if (pointsPerBall >= pointsPerBallUpdate)
            {
               pointsPerBall = 0;

               if (scs != null)
               {
                  bagOfBalls.reset();

                  for (Point3d checkPoint : points)
                  {
                     double z2 = heightMap.heightAtPoint(checkPoint.getX(), checkPoint.getY());
                     bagOfBalls.setBall(checkPoint.getX(), checkPoint.getY(), z2);
                  }

                  scs.tickAndUpdate();
               }
            }
         }
         
         scs.addStaticLinkGraphics(staticLinkGraphics);
      }
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
      location.rotZ(Math.toRadians(yawDegrees));

      location.setTranslation(new Vector3d(x, y, height / 2));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, length, width, height), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }




}
