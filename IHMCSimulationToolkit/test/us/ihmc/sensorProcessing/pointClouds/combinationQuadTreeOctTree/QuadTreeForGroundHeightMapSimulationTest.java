package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import static org.junit.Assert.*;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.yoGraphics.BagOfBalls;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.shapes.Box3d;
import us.ihmc.robotics.geometry.shapes.Plane3d;
import us.ihmc.robotics.math.frames.YoFramePoint;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.simulationToolkit.visualizers.QuadTreeHeightMapVisualizer;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.CombinedTerrainObject3D;
import us.ihmc.simulationconstructionset.util.ground.RotatableBoxTerrainObject;
import us.ihmc.tools.thread.ThreadTools;

public class QuadTreeForGroundHeightMapSimulationTest
{
   private static final boolean DO_ASSERTS = true;

	@ContinuousIntegrationTest(estimatedDuration = 0.1, categoriesOverride = IntegrationCategory.EXCLUDE)
	@Test(timeout = 300000)
   public void testPointsFromAFile() throws NumberFormatException, IOException
   {
      boolean visualizeAndKeepUp = false;
      
      double minX = -1.0; //5.0f;
      double minY = -3.0; //5.0f;
      double maxX = 1.0; //5.0f;
      double maxY = 4.0; //5.0f;
      
      Box bounds = new Box(minX, minY, maxX, maxY);
      
      float resolution = 0.025f;
      float heightThreshold = 0.005f;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.02;
      int maxNodes = 1000000;

//      SimplifiedGroundOnlyQuadTree quadTree = new SimplifiedGroundOnlyQuadTree(minX, minY, maxX, maxY, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise );

      int maxBalls = 200;
      QuadTreeTestHelper testHelper = new QuadTreeTestHelper(new BoundingBox2D(minX, minY, maxX, maxY), maxBalls, visualizeAndKeepUp);
      testHelper.setResolutionParameters(resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxNodes);

//      String filename = "resources/pointListsForTesting/pointList150122_DRCObstacleCourse.pointList";
      String filename = "resources/pointListsForTesting/firstMinuteCinderBlockScans.fullPointList";
      
      
     double maxZ = 0.6;

     int skipPoints = 0;
     int maxNumberOfPoints = 2000000;

     QuadTreeForGroundReaderAndWriter quadTreeForGroundReaderAndWriter = new QuadTreeForGroundReaderAndWriter();
     ArrayList<Point3D> points = quadTreeForGroundReaderAndWriter.readPointsFromFile(filename, skipPoints, maxNumberOfPoints, bounds, maxZ);

      int pointsPerBallUpdate = 10000;
      boolean drawPointsInBlue = false;
      testHelper.createHeightMapFromAListOfPoints(points, drawPointsInBlue , pointsPerBallUpdate);
      
//      helper.createHeightMap(points, rangeOfPointsToTest, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);

      if (visualizeAndKeepUp)
      {
//         testHelper.drawPoints(points, resolution/2.0, YoAppearance.Blue());
      
//      testHelper.drawHeightOfOriginalPointsInPurple(points, 1);
//      Graphics3DNode handle = testHelper.drawNodeBoundingBoxes(-0.1);
//      testHelper.drawHeightMap(minX, minY, maxX, maxY, resolution);
      testHelper.drawAllPointsInQuadTree(resolution/2.0, YoAppearance.Purple());
      
      testHelper.displaySimulationConstructionSet();
      }
      
      
//      testHelper.doATest(points, pointsPerBallUpdate);
      if (visualizeAndKeepUp)
         ThreadTools.sleepForever();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOnSomeSlopes()
   {
      boolean visualizeAndKeepUp = false;

      double halfWidth = 0.5;
      double resolution = 0.1;

      Point3D center = new Point3D(0.0, 0.0, 0.3);
      Vector3D normal = new Vector3D(0.1, 0.2, 0.8);
      testOnASlope(center, normal, halfWidth, resolution, visualizeAndKeepUp);

      center = new Point3D(0.0, 0.0, 0.3);
      normal = new Vector3D(1.0, 1.0, 1.0);
      testOnASlope(center, normal, halfWidth, resolution, visualizeAndKeepUp);

      center = new Point3D(0.0, 0.0, 0.3);
      normal = new Vector3D(-1.0, 1.0, 1.0);
      testOnASlope(center, normal, halfWidth, resolution, visualizeAndKeepUp);

      if (visualizeAndKeepUp) ThreadTools.sleepForever();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOnSomeStairCases()
   {
      boolean visualizeAndKeepUp = false;

      double halfWidth = 0.6;
      double resolution = 0.02;

      Point3D center = new Point3D(0.0, 0.0, 0.3);
      double stairSeparation = 0.2;
      double oneStairLandingHeight = 0.0;

      Vector3D normal = new Vector3D(0.3, -0.3, 1.0);
      testOnAStaircase(center, normal, halfWidth, resolution, stairSeparation, oneStairLandingHeight, visualizeAndKeepUp);

//    normal = new Vector3d(0.3, 0.3, 1.0);
//    testOnAStaircase(center, normal, halfWidth, resolution, stairSeparation, oneStairLandingHeight);
//    
//    normal = new Vector3d(-0.3, 0.3, 1.0);
//    testOnAStaircase(center, normal, halfWidth, resolution, stairSeparation, oneStairLandingHeight);
//    
//    normal = new Vector3d(-0.3, -0.3, 1.0);
//    testOnAStaircase(center, normal, halfWidth, resolution, stairSeparation, oneStairLandingHeight);

      if (visualizeAndKeepUp) ThreadTools.sleepForever();
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testUsingStairGroundProfile()
   {
      boolean visualizeAndKeepUp = false;
      
      CombinedTerrainObject3D groundProfile = createStepsGroundProfile();

      double centerX = -3.5;
      double centerY = 3.5;
      double halfWidth = 0.6;

      double minX = centerX - halfWidth;
      double minY = centerY - halfWidth;
      double maxX = centerX + halfWidth;
      double maxY = centerY + halfWidth;

      BoundingBox2D boundingBox = new BoundingBox2D(minX, minY, maxX, maxY);
 
      double resolution = 0.02;
      double heightThreshold = 0.002;
      double maxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 10;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;

      QuadTreeForGroundParameters parameters = new QuadTreeForGroundParameters(resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, -1);

      QuadTreeTestHelper testHelper = new QuadTreeTestHelper(boundingBox, (int) ((halfWidth/resolution) * (halfWidth/resolution)), visualizeAndKeepUp);      
      testHelper.setResolutionParameters(resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise, maxNodes);

      ArrayList<Point3D> points = testHelper.createAListOfPointsFromAGroundProfile(groundProfile, minX, minY, maxX, maxY, resolution);
      int pointsPerBallUpdate = 1;
      
      boolean drawPointsInBlue = false;
      testHelper.createHeightMapFromAListOfPoints(points, drawPointsInBlue, pointsPerBallUpdate);

      // TODO: Get this to pass!
      testHelper.assertPointsLieOnHeightMap(points);

      if (visualizeAndKeepUp) ThreadTools.sleepForever();
   }
  

   private QuadTreeTestHelper testOnAStaircase(Point3D center, Vector3D normal, double halfWidth, double resolution, double stairSeparation, double oneStairLandingHeight, boolean visualize)
   {
      normal.normalize();

      BoundingBox2D boundingBox = new BoundingBox2D(center.getX() - halfWidth, center.getY() - halfWidth, center.getX() + halfWidth, center.getY() + halfWidth);
      Plane3d plane3d = new Plane3d(center, normal);
      ArrayList<Point3D> points = generatePointsForStairs(plane3d, halfWidth, resolution, stairSeparation, oneStairLandingHeight);

//      Collections.shuffle(points);
      
      int pointsPerBallUpdate = 1;
      QuadTreeTestHelper testHelper = testOnAListOfPoints(points, pointsPerBallUpdate, boundingBox, resolution, visualize);
      
      if (visualize)
      {
         testHelper.drawPoints(points, resolution/2.0, YoAppearance.Blue());

         ArrayList<Point3D> allPointsInQuadTree = testHelper.getAllPointsInQuadTree();
         testHelper. drawPoints(allPointsInQuadTree, resolution*0.6, YoAppearance.Chartreuse());
      }

//      testHelper. drawPointsWithinAreaInSCS(pointsPerBallUpdate, YoAppearance.Chartreuse());
//      testHelper.drawHeightOfOriginalPointsInPurple(points, pointsPerBallUpdate);
//      testHelper.drawHeightMap(boundingBox, resolution);
      
      return testHelper;
   }

   private QuadTreeTestHelper testOnASlope(Point3D center, Vector3D normal, double halfWidth, double resolution, boolean visualize)
   {
      normal.normalize();

      BoundingBox2D boundingBox = new BoundingBox2D(center.getX() - halfWidth, center.getY() - halfWidth, center.getX() + halfWidth, center.getY() + halfWidth);
      Plane3d plane3d = new Plane3d(center, normal);
      ArrayList<Point3D> points = generatePointsForSlope(plane3d, halfWidth, resolution);
      
      int pointsPerBallUpdate = 1;
      return testOnAListOfPoints(points, pointsPerBallUpdate, boundingBox, resolution, visualize);
   }

   private QuadTreeTestHelper testOnAListOfPoints(ArrayList<Point3D> points, int pointsPerBallUpdate, BoundingBox2D rangeOfPoints, double resolution, boolean visualize)
   {
      double heightThreshold = 0.002;
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxNodes = 1000000;

      QuadTreeTestHelper testHelper = new QuadTreeTestHelper(rangeOfPoints, points.size(), visualize);
      testHelper.setResolutionParameters(resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxNodes);

      boolean drawPointsInBlue = false;
      testHelper.createHeightMapFromAListOfPoints(points, drawPointsInBlue, pointsPerBallUpdate);
      
      testHelper.assertPointsLieOnHeightMap(points);
      
      return testHelper;
   }


   private static ArrayList<Point3D> generatePointsForStairs(Plane3d plane3d, double halfWidth, double stepSize, double stairSeparation,
           double oneStairLandingHeight)
   {
      ArrayList<Point3D> ret = generatePointsForSlope(plane3d, halfWidth, stepSize);
      formStaircaseWithPointsOnAPlane(ret, stairSeparation, oneStairLandingHeight);

      return ret;
   }

   private static ArrayList<Point3D> generatePointsForSlope(Plane3d plane3d, double halfWidth, double stepSize)
   {
      Point3D centerPoint = plane3d.getPointCopy();

      double minX = centerPoint.getX() - halfWidth;
      double minY = centerPoint.getY() - halfWidth;
      double maxX = centerPoint.getX() + halfWidth;
      double maxY = centerPoint.getY() + halfWidth;

      ArrayList<Point3D> points = new ArrayList<Point3D>();

      for (double x = minX; x < maxX; x = x + stepSize)
      {
         for (double y = minY; y < maxY; y = y + stepSize)
         {
            double z = plane3d.getZOnPlane(x, y);
            points.add(new Point3D(x, y, z));
         }
      }

      return points;
   }

   private static void formStaircaseWithPointsOnAPlane(ArrayList<Point3D> pointsList, double stairSeparation, double oneStairLandingHeight)
   {
      for (Point3D point3d : pointsList)
      {
         double z = point3d.getZ();

         double newZ = Math.floor((z - oneStairLandingHeight) / stairSeparation) * stairSeparation;
         point3d.setZ(newZ);
      }
   }


   private static class QuadTreeTestHelper
   {
      private final boolean visualize;
      private YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

      private final Robot robot;
      private final SimulationConstructionSet scs;
      
      private final BagOfBalls bagOfBalls;
      private final YoFramePoint queryPoint;
      
//      private SimplifiedGroundOnlyQuadTree heightMap;
      private QuadTreeHeightMapInterface heightMap;
//    private final double centerX, centerY;
//    private final double minX, minY, maxX, maxY;
      private final BoundingBox2D rangeOfPointsToTest;

      private double resolution = 0.1;
      private double heightThreshold = 0.002;
      private double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      private int maxSameHeightPointsPerNode = 10;
      private double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      private int maxNodes = 1000000;

      
      public QuadTreeTestHelper(BoundingBox2D rangeOfPointsToTest, int maxNumberOfBallsInBag, boolean visualize)
      {
         this.visualize = visualize;
         
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         queryPoint = new YoFramePoint("queryPoint", worldFrame, registry);
         this.rangeOfPointsToTest = rangeOfPointsToTest;

         if (visualize)
         {
            robot = new Robot("TestQuadTree");
            robot.getRobotsYoVariableRegistry().addChild(registry);
            scs = new SimulationConstructionSet(robot);
            
            scs.setGroundVisible(false);
            
            YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();

            double ballSize = resolution * 0.35;

            bagOfBalls = new BagOfBalls(maxNumberOfBallsInBag, ballSize, registry, yoGraphicsListRegistry);
            
            YoGraphicPosition queryViz = new YoGraphicPosition("Query", queryPoint, 1.1 * ballSize, YoAppearance.Red());
            yoGraphicsListRegistry.registerYoGraphic("Query", queryViz);
            scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
         }
         else
         {
            robot = null;
            scs = null;
            bagOfBalls = null;
         }
         
      }

      public ArrayList<Point3D> getAllPointsInQuadTree()
      {
         ArrayList<Point3D> pointsToReturn = new ArrayList<Point3D>();
         ((QuadTreeForGroundHeightMap) heightMap).getStoredPoints(pointsToReturn);
         return pointsToReturn;
         
      }

      public void setResolutionParameters(double resolution, double heightThreshold, double quadTreeMaxMultiLevelZChangeToFilterNoise, int maxNodes)
      {
         this.resolution = resolution;
         this.heightThreshold = heightThreshold;
         this.quadTreeMaxMultiLevelZChangeToFilterNoise = quadTreeMaxMultiLevelZChangeToFilterNoise;
         this.maxNodes = maxNodes;
      }

      public QuadTreeHeightMapInterface getHeightMap()
      {
         return heightMap;
      }

      private void displaySimulationConstructionSet()
      {
         if (visualize) scs.startOnAThread();
      } 
        
      
      public Graphics3DNode drawPoints(ArrayList<Point3D> points, double resolution, AppearanceDefinition appearance)
      {
         return QuadTreeHeightMapVisualizer.drawPoints(scs, points, resolution, appearance);       
      }
      
      public Graphics3DNode drawHeightMap(BoundingBox2D rangeOfPoints, double resolution2)
      {
         return QuadTreeHeightMapVisualizer.drawHeightMap(heightMap, scs, rangeOfPoints, resolution);  
         
      }
      
      private Graphics3DNode drawHeightMap(double minX, double minY, double maxX, double maxY, float resolution)
      {
         return QuadTreeHeightMapVisualizer.drawHeightMap(heightMap, scs, minX, minY, maxX, maxY, resolution);  
      }

      private Graphics3DNode drawAllPointsInQuadTree(double sizeToDrawCubes, AppearanceDefinition appearance)
      {
         return QuadTreeHeightMapVisualizer.drawAllPointsInQuadTree(heightMap, sizeToDrawCubes, scs, appearance);
      }
      
      private Graphics3DNode drawNodeBoundingBoxes(double heightToDrawAt)
      {
         if (heightMap instanceof QuadTreeForGroundHeightMap)
         {
            Graphics3DNode handle = QuadTreeHeightMapVisualizer.drawNodeBoundingBoxes((QuadTreeForGroundHeightMap) heightMap, scs, heightToDrawAt);
            return handle;
         }

         return null;     
      }
      
      private void drawHeightOfOriginalPointsInPurple(ArrayList<Point3D> points, int pointsPerBallUpdate)
      {
         if (!visualize) return;
         
         int count = 0;
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         for (Point3D point : points)
         {
            count ++;
            if (count >= pointsPerBallUpdate)
            {
               count = 0;

               double z = heightMap.getHeightAtPoint(point.getX(), point.getY());

               staticLinkGraphics.identity();
               staticLinkGraphics.translate(new Vector3D(point.getX(), point.getY(), z + 0.001));

               double cubeSize = resolution * 0.35;
               staticLinkGraphics.addCube(cubeSize, cubeSize, cubeSize / 3.0, YoAppearance.Purple());
            }
         }
         scs.addStaticLinkGraphics(staticLinkGraphics);
      }

      private Graphics3DNode drawPointsWithinAreaInSCS(int pointsPerBallUpdate, AppearanceDefinition appearance)
      {
         if (!visualize) return null;
         
         Point2D centerPoint = new Point2D();
         rangeOfPointsToTest.getCenterPoint(centerPoint);
         List<Point3D> allPointsWithinArea = heightMap.getAllPointsWithinArea(centerPoint.getX(), centerPoint.getY(), 10.0, 10.0);
         
         int count = 0;
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();
         for (Point3D point3d : allPointsWithinArea)
         {
            count ++;
            if (count >= pointsPerBallUpdate)
            {
               count = 0;

               staticLinkGraphics.identity();
               double cubeSize = resolution * 0.5;
               staticLinkGraphics.translate(new Vector3D(point3d.getX(), point3d.getY(), point3d.getZ() -cubeSize / 2.0 + 0.001));
               staticLinkGraphics.addCube(cubeSize, cubeSize, cubeSize, appearance);
            }
         }
         return scs.addStaticLinkGraphics(staticLinkGraphics);
      }


      public void assertPointsLieOnHeightMap(ArrayList<Point3D> points)
      {
         if (DO_ASSERTS)
         {
            for (Point3D point : points)
            {
               double heightMapZ = heightMap.getHeightAtPoint(point.getX(), point.getY());
               assertEquals(point.getZ(), heightMapZ, 1e-7);
            }
         }
      }

      public ArrayList<Point3D> createAListOfPointsFromAGroundProfile(GroundProfile3D groundProfile, BoundingBox2D testingRange, double resolution)
      {
         double minX = testingRange.getMinPoint().getX();
         double maxX = testingRange.getMaxPoint().getX();
         double minY = testingRange.getMinPoint().getY();
         double maxY = testingRange.getMaxPoint().getY();

         return createAListOfPointsFromAGroundProfile(groundProfile, minX, minY, maxX, maxY, resolution);
      }

      public ArrayList<Point3D> createAListOfPointsFromAGroundProfile(GroundProfile3D groundProfile, double minX, double minY, double maxX, double maxY,
              double resolution)
      {
         ArrayList<Point3D> points = new ArrayList<Point3D>();
         for (double x = minX; x < maxX; x = x + resolution)
         {
            for (double y = minY; y < maxY; y = y + resolution)
            {
               double z = groundProfile.getHeightMapIfAvailable().heightAt(x, y, 0.0);
               points.add(new Point3D(x, y, z));
            }
         }

         return points;
      }


      private void createHeightMapFromAListOfPoints(ArrayList<Point3D> points, boolean drawPointsInBlue, int pointsPerBallUpdate)
      {
         double minX = rangeOfPointsToTest.getMinPoint().getX();
         double maxX = rangeOfPointsToTest.getMaxPoint().getX();
         double minY = rangeOfPointsToTest.getMinPoint().getY();
         double maxY = rangeOfPointsToTest.getMaxPoint().getY();

         Box bounds = new Box(minX, minY, maxX, maxY);
         QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, -1);
         heightMap = new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);
         
         int pointsPerBall = 0;
         Graphics3DObject staticLinkGraphics = new Graphics3DObject();

         for (Point3D point : points)
         {
            queryPoint.set(point);

            boolean pointWasAdded = heightMap.addPoint(point.getX(), point.getY(), point.getZ());

            if (visualize)
            {
               if (drawPointsInBlue)
               {
                  staticLinkGraphics.identity();
                  staticLinkGraphics.translate(new Vector3D(point.getX(), point.getY(), point.getZ() + 0.001));
                  double cubeSize = resolution * 0.35;
                  if (pointWasAdded) staticLinkGraphics.addCube(cubeSize, cubeSize, cubeSize / 3.0, YoAppearance.Blue());
                  else staticLinkGraphics.addCube(cubeSize, cubeSize, cubeSize / 3.0, YoAppearance.Red());
               }

               pointsPerBall++;
               if (pointsPerBall >= pointsPerBallUpdate)
               {
                  pointsPerBall = 0;

                  if (scs != null)
                  {
                     bagOfBalls.reset();

                     for (Point3D checkPoint : points)
                     {
                        double z2 = heightMap.getHeightAtPoint(checkPoint.getX(), checkPoint.getY());
                        bagOfBalls.setBall(checkPoint.getX(), checkPoint.getY(), z2);
                     }

                     scs.tickAndUpdate();
                  }
               }
            }
         }

         if (visualize)
         {
            scs.addStaticLinkGraphics(staticLinkGraphics);
            displaySimulationConstructionSet();
         }

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
      location.setRotationYawAndZeroTranslation(Math.toRadians(yawDegrees));

      location.setTranslation(new Vector3D(x, y, height / 2));
      RotatableBoxTerrainObject newBox = new RotatableBoxTerrainObject(new Box3d(location, length, width, height), app);
      combinedTerrainObject.addTerrainObject(newBox);
   }




}
