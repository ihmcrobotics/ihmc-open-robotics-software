package us.ihmc.robotics.quadTree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.ArrayList;
import java.util.Collection;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.continuousIntegration.IntegrationCategory;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.random.RandomGeometry;

public class QuadTreeForGroundTest
{
   @ContinuousIntegrationTest(estimatedDuration = 0.0, categoriesOverride = IntegrationCategory.EXCLUDE)
   @Test(timeout = 30000)
   public void testGetAllPoints()
   {
      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(0.01, Double.MAX_VALUE, Double.MAX_VALUE, Integer.MAX_VALUE, 0.0, -1);
      QuadTreeForGround tree = new QuadTreeForGround(new Box(-1, -1, 1, 1), quadTreeParameters);

      Collection<Point3D> points = new ArrayList<>();

      //ensure 
      for (double x = -0.5; x < 0.5; x += quadTreeParameters.getResolution() * 2)
      {
         for (double y = -0.5; y < 0.5; y += quadTreeParameters.getResolution() * 2)
         {
            final double z = 0.1;
            Point3D p = new Point3D(x, y, z);
            points.add(p);
            tree.put(x, y, 0.1);
         }
      }

      ArrayList<Point3D> retrievedPoints = new ArrayList<>();
      tree.getStoredPoints(retrievedPoints);
      assertTrue(points.containsAll(retrievedPoints));

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testCommonCaseWithNoFilteringOrPointAveraging()
   {
      Random random = new Random(1178L);
      Box bounds = new Box(-10.0, -8.0, 10.0, 12.0);

      // Set resolution to zero and max values to infinity to just make sure every point is kept as it is.
      double resolution = 0.0;
      double heightThreshold = 0.1f;
      double maxMultiLevelZChangeToFilterNoise = Double.POSITIVE_INFINITY;
      int maxSameHeightPointsPerNode = Integer.MAX_VALUE;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = Double.POSITIVE_INFINITY;

      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise,
            maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, -1);
      QuadTreeForGround quadTree = new QuadTreeForGround(bounds, quadTreeParameters);

      assertEquals(bounds.maxX, quadTree.getMaxX(), 1e-7);
      assertEquals(bounds.maxY, quadTree.getMaxY(), 1e-7);
      assertEquals(bounds.minX, quadTree.getMinX(), 1e-7);
      assertEquals(bounds.minY, quadTree.getMinY(), 1e-7);
      assertTrue(quadTreeParameters == quadTree.getQuadTreeParameters());

      quadTree.setHeightThreshold(0.33);
      assertEquals(0.33, quadTree.getQuadTreeParameters().getHeightThreshold(), 1e-7);
      quadTree.setHeightThreshold(0.1);

      int numberOfPoints = 1000;
      double minZ = -1.0;
      double maxZ = 1.0;

      ArrayList<Point3D> points = generateRandomPoints(random, numberOfPoints, bounds, minZ, maxZ);
      for (Point3D point : points)
      {
         quadTree.put(point.getX(), point.getY(), point.getZ());

         Box boundsAroundPoint = new Box(point.getX() - resolution / 2.0, point.getY() - resolution / 2.0, point.getX() + resolution / 2.0, point.getY()
               + resolution / 2.0);
         ArrayList<Point3D> pointsToCheck = new ArrayList<Point3D>();
         quadTree.getAllPointsWithinBounds(boundsAroundPoint, pointsToCheck);

         assertTrue(isPointValueInList(point, pointsToCheck));

         Point3D closestPoint = new Point3D();
         quadTree.getClosestPoint(point.getX(), point.getY(), closestPoint);
         assertFalse(point == closestPoint);
         EuclidCoreTestTools.assertTuple3DEquals("point = " + point + ", closestPoint = " + closestPoint, point, closestPoint, 1e-7);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testPointLimiter()
   {

      Box bounds = new Box(-10f, -10, 10, 10);
      QuadTreeForGroundParameters parameters = new QuadTreeForGroundParameters(0.49f, 0.001f, 0.2, 4, Double.POSITIVE_INFINITY, 32);
      QuadTreeForGround quadTree = new QuadTreeForGround(bounds, parameters);

      // Add 16 nodes in NE quadrant
      for(int j = 1; j < 5; j++)
      {
         for(int i = 0; i < 4; i++)
         {
            quadTree.put(j, j, i * 0.01);
         }         
      }
      
      // Add 16 nodes in SW quadrant
      for(int j = 1; j < 5; j++)
      {
         for(int i = 0; i < 4; i++)
         {
            quadTree.put(-j, -j, i * 0.01);
         }         
      }
      
      
      ArrayList<QuadTreeForGroundNode> childrenToPack = new ArrayList<>();
      
      quadTree.getRootNode().getChildrenNodes(childrenToPack);
      QuadTreeForGroundNode NW = childrenToPack.get(0);
      QuadTreeForGroundNode NE = childrenToPack.get(1);
      QuadTreeForGroundNode SE = childrenToPack.get(2);
      QuadTreeForGroundNode SW = childrenToPack.get(3);

      assertTrue(NE.hasChildren());
      assertTrue(SW.hasChildren());
      assertFalse(NW.hasChildren());
      assertFalse(SE.hasChildren());
      
      
      // Add 16 nodes in SW quadrant
      for(int j = 1; j < 5; j++)
      {
         for(int i = 0; i < 4; i++)
         {
            quadTree.put(-1.5, -j, i * 0.01);
            assertEquals(parameters.getMaximumNumberOfPoints(), quadTree.getNumberOfPoints());

         }         
      }
      
      assertFalse(NE.hasChildren());
      assertTrue(SW.hasChildren());
      assertFalse(NW.hasChildren());
      assertFalse(SE.hasChildren());

      
      // Add 16 nodes in SE quadrant
      for(int j = 1; j < 5; j++)
      {
         for(int i = 0; i < 4; i++)
         {
            quadTree.put(j, -j, i * 0.01);
            assertEquals(parameters.getMaximumNumberOfPoints(), quadTree.getNumberOfPoints());
         }         
      }
      
      assertTrue(SE.hasChildren());
      assertTrue(SW.hasChildren());
      assertFalse(NW.hasChildren());
      assertFalse(NE.hasChildren());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSimpleCasesOne()
   {
      float minX = -10.0f;
      float minY = -10.0f;
      float maxX = 10.0f;
      float maxY = 10.0f;
      float resolution = 0.49f;
      float heightThreshold = 0.001f;
      double maxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = Double.POSITIVE_INFINITY;

      QuadTreeForGround quadTree = new QuadTreeForGround(minX, minY, maxX, maxY, resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise,
            maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose);
      assertEquals(1, quadTree.getNumberOfQuads());
      quadTree.checkRepInvarients();
      assertFalse(quadTree.getRootNode().hasChildren());

      double returnNullObject = quadTree.getHeightAtPoint(0.0f, 0.0f);
      assertTrue(Double.isNaN(returnNullObject));

      // Put a single point at zero

      Double valueAtZero = new Double(1.5);
      quadTree.put(0.0, 0.0f, valueAtZero);
      assertEquals(1, quadTree.getNumberOfQuads());
      quadTree.checkRepInvarients();
      assertFalse(quadTree.getRootNode().hasChildren());

      Double returnValueAtZero = quadTree.getHeightAtPoint(0.0f, 0.0f);
      assertEquals(valueAtZero, returnValueAtZero, 1e-7);
      quadTree.checkRepInvarients();

      Double returnValueOutOfBounds = quadTree.getHeightAtPoint(100.0f, -720.0f);
      assertTrue(Double.isNaN(returnValueOutOfBounds));

      Double returnValueAwayFromZero = quadTree.getHeightAtPoint(3.0f, -7.2f);
      assertEquals(valueAtZero, returnValueAwayFromZero, 1e-7);

      // Put a point away from zero
      Double valueAtNegativeOneOne = new Double(2.7);
      quadTree.put(-1.0f, 1.0f, valueAtNegativeOneOne);
      assertEquals(4, quadTree.getNumberOfQuads());
      quadTree.checkRepInvarients();
      assertTrue(quadTree.getRootNode().hasChildren());

      returnValueAtZero = quadTree.getHeightAtPoint(0.0f, 0.0f);
      assertEquals(valueAtZero, returnValueAtZero, 1e-7);
      quadTree.checkRepInvarients();

      Double returnValueAtNegativeOneOne = quadTree.getHeightAtPoint(-1.0f, 1.0f);
      assertEquals(valueAtNegativeOneOne, returnValueAtNegativeOneOne, 1e-7);

      // Clear and make sure previous points return NaN
      quadTree.clear();
      quadTree.checkRepInvarients();

      returnValueAtZero = quadTree.getHeightAtPoint(0.0f, 0.0f);
      returnValueAtNegativeOneOne = quadTree.getHeightAtPoint(-1.0f, 1.0f);
      assertTrue(Double.isNaN(returnValueAtZero));
      assertTrue(Double.isNaN(returnValueAtNegativeOneOne));
      quadTree.checkRepInvarients();

      assertEquals(1, quadTree.getNumberOfQuads());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testWithFilteringAndPointAveraging()
   {
      Random random = new Random(1398L);
      Box bounds = new Box(-5.0, -5.0, 5.0, 5.0);

      double resolution = 0.1;
      double heightThreshold = 0.1f;
      double maxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 5;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;

      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise,
            maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, -1);
      QuadTreeForGround quadTree = new QuadTreeForGround(bounds, quadTreeParameters);

      double minZ = -1.0;
      double maxZ = 1.0;

      double x = 3.3;
      double y = -1.7;

      // Put a bunch of points in the same node and verify that they are averaged properly

      // As long as the first maxSameHeightPointsPerNode are within heightThreshold of the first, just gives the average height:
      assertEquals(1, quadTree.getNumberOfQuads());

      double z1 = 1.0;
      quadTree.put(x, y, z1);
      assertEquals(z1, quadTree.getHeightAtPoint(x, y), 1e-7);
      assertEquals(1, quadTree.getNumberOfQuads());
      ArrayList<Point3D> allPoints = new ArrayList<Point3D>();
      quadTree.getStoredPoints(allPoints);
      assertEquals(1, allPoints.size());

      Point3D firstPoint = allPoints.get(0);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(x, y, z1), firstPoint, 1e-7);

      double z2 = 1.01;
      quadTree.put(x + 0.001, y, z2);
      assertEquals((z1 + z2) / 2.0, quadTree.getHeightAtPoint(x, y), 1e-7);
      assertEquals(16, quadTree.getNumberOfQuads()); // 16 here since divides to a reasonable resolution (4 times res or so)
      allPoints.clear();
      quadTree.getStoredPoints(allPoints);
      assertEquals(2, allPoints.size());
      Point3D firstPointAgain = allPoints.get(0);
      EuclidCoreTestTools.assertTuple3DEquals(new Point3D(x, y, z1), firstPointAgain, 1e-7);

      double z3 = 1.02;
      quadTree.put(x, y, z3);
      assertEquals((z1 + z2 + z3) / 3.0, quadTree.getHeightAtPoint(x, y), 1e-7);
      allPoints.clear();
      quadTree.getStoredPoints(allPoints);
      assertEquals(3, allPoints.size());

      double z4 = 1.03;
      quadTree.put(x, y, z4);
      assertEquals((z1 + z2 + z3 + z4) / 4.0, quadTree.getHeightAtPoint(x, y), 1e-7);
      allPoints.clear();
      quadTree.getStoredPoints(allPoints);
      assertEquals(4, allPoints.size());

      double z5 = 1.04;
      quadTree.put(x, y, z5);
      assertEquals((z1 + z2 + z3 + z4 + z5) / 5.0, quadTree.getHeightAtPoint(x, y), 1e-7);
      allPoints.clear();
      quadTree.getStoredPoints(allPoints);
      assertEquals(5, allPoints.size());
      assertEquals(16, quadTree.getNumberOfQuads()); // Still 16 here.

      // But then, on the next one, they'll get split all the way down to the minimum resolution:
      double z6 = 1.03;
      quadTree.put(x, y, z6);
      double average = (z1 + z2 + z3 + z4 + z5 + z6) / 6.0;
      double heightAt = quadTree.getHeightAtPoint(x, y);

      assertTrue(Math.abs(average - heightAt) > 1e-3); // Won't be average since just 5 points now.

      allPoints.clear();
      quadTree.getStoredPoints(allPoints);
      assertEquals(5, allPoints.size()); // Still only 5.
      assertEquals(16, quadTree.getNumberOfQuads()); // Still 16 here.
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestPoint()
   {
      Random random = new Random(1776L);
      Box bounds = new Box(-10.0, -10.0, 10.0, 10.0);

      double minZ = -5.0;
      double maxZ = 5.0;

      double resolution = 0.02;
      double heightThreshold = 0.002;
      double maxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;

      QuadTreeForGroundParameters parameters = new QuadTreeForGroundParameters(resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise,
            maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, -1);
      int numberOfPoints = 1000;
      QuadTreeForGround quadTree = generateRandomQuadTree(random, bounds, parameters, minZ, maxZ, numberOfPoints);

      ArrayList<Point3D> points = new ArrayList<Point3D>();
      quadTree.getStoredPoints(points);

      //    System.out.println("The quad tree has " + points.size() + " points.");
      int numberOfTests = 100;
      Point3D closestPoint = new Point3D();

      for (int i = 0; i < numberOfTests; i++)
      {
         double xQuery = RandomNumbers.nextDouble(random, bounds.minX, bounds.maxX);
         double yQuery = RandomNumbers.nextDouble(random, bounds.minY, bounds.maxY);

         quadTree.getClosestPoint(xQuery, yQuery, closestPoint);

         double distanceSquared = distanceXYSquared(xQuery, yQuery, closestPoint);

         for (Point3D point : points)
         {
            if ((point != closestPoint) && (distanceXYSquared(xQuery, yQuery, point) < distanceSquared))
            {
               fail("Not closest point! Query = " + xQuery + ", " + yQuery + " thinks closest point is " + closestPoint + " but a closer point is " + point);
            }
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetAllPointsWithinBounds()
   {
      Random random = new Random(1178L);
      Box bounds = new Box(-20.0, -0.0, 20.0, 3.0);

      double minZ = -7.0;
      double maxZ = 3.0;

      double resolution = 0.05;
      double heightThreshold = 0.1f;
      double maxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 3;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;

      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise,
            maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, -1);

      int numberOfPoints = 1000;
      QuadTreeForGround quadTree = generateRandomQuadTree(random, bounds, quadTreeParameters, minZ, maxZ, numberOfPoints);

      ArrayList<Point3D> allPoints = new ArrayList<Point3D>();
      quadTree.getStoredPoints(allPoints);

      ArrayList<Point3D> pointsToCheck = new ArrayList<Point3D>();
      quadTree.getAllPointsWithinBounds(bounds, pointsToCheck);

      assertEquals(allPoints.size(), pointsToCheck.size());

      for (Point3D point : allPoints)
      {
         Box boundsAroundPoint = new Box(point.getX() - resolution / 2.0, point.getY() - resolution / 2.0, point.getX() + resolution / 2.0, point.getY()
               + resolution / 2.0);
         pointsToCheck.clear();
         quadTree.getAllPointsWithinBounds(boundsAroundPoint, pointsToCheck);

         boolean pointValueInList = isPointValueInList(point, pointsToCheck);
         assertTrue(pointValueInList);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testGetAllPointsWithinDistance()
   {
      Random random = new Random(1984L);
      Box bounds = new Box(-10.0, -10.0, 10.0, 10.0);

      double minZ = -5.0;
      double maxZ = 5.0;

      double resolution = 0.02;
      double heightThreshold = 0.002;
      double maxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;

      QuadTreeForGroundParameters parameters = new QuadTreeForGroundParameters(resolution, heightThreshold, maxMultiLevelZChangeToFilterNoise,
            maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, -1);
      int numberOfPoints = 1000;
      QuadTreeForGround quadTree = generateRandomQuadTree(random, bounds, parameters, minZ, maxZ, numberOfPoints);

      ArrayList<Point3D> allPoints = new ArrayList<Point3D>();
      quadTree.getStoredPoints(allPoints);

      ArrayList<Point3D> pointsWithinDistance = new ArrayList<Point3D>();

      // If distance is negative, return none of them.
      quadTree.getAllPointsWithinDistance(bounds.centreX, bounds.centreY, -0.01, pointsWithinDistance);
      assertTrue(pointsWithinDistance.isEmpty());

      // If distance is Infinity, return all of them.
      quadTree.getAllPointsWithinDistance(bounds.centreX, bounds.centreY, Double.POSITIVE_INFINITY, pointsWithinDistance);
      assertEquals(allPoints.size(), pointsWithinDistance.size());

      int numberOfTests = 100;

      for (int i = 0; i < numberOfTests; i++)
      {
         pointsWithinDistance.clear();

         double xQuery = RandomNumbers.nextDouble(random, bounds.minX, bounds.maxX);
         double yQuery = RandomNumbers.nextDouble(random, bounds.minY, bounds.maxY);
         double distance = RandomNumbers.nextDouble(random, 0.0, 5.0);

         quadTree.getAllPointsWithinDistance(xQuery, yQuery, distance, pointsWithinDistance);

         ArrayList<Point3D> checkPointsWithinDistance = naiiveGetAllPointsWithinDistance(allPoints, xQuery, yQuery, distance);

         if (pointsWithinDistance.size() != checkPointsWithinDistance.size())
         {
            fail("pointsWithinDistance.size() != checkPointsWithinDistance.size()");
         }

         for (Point3D pointWithinDistance : pointsWithinDistance)
         {
            if (!checkPointsWithinDistance.contains(pointWithinDistance))
            {
               fail("Doesn't contain point! Query = " + xQuery + ", " + yQuery + " pointWithinDistance = " + pointWithinDistance);
            }
         }
      }
   }

   private boolean isPointValueInList(Point3D pointToCheck, ArrayList<Point3D> pointList)
   {
      for (Point3D pointInList : pointList)
      {
         if (pointToCheck.distanceSquared(pointInList) < 1e-10)
            return true;
      }

      return false;
   }

   private ArrayList<Point3D> naiiveGetAllPointsWithinDistance(ArrayList<Point3D> inputPoints, double x, double y, double distance)
   {
      ArrayList<Point3D> pointsToReturn = new ArrayList<Point3D>();

      for (Point3D inputPoint : inputPoints)
      {
         if (distanceXYSquared(x, y, inputPoint) < distance * distance)
         {
            pointsToReturn.add(inputPoint);
         }
      }

      return pointsToReturn;
   }

   private QuadTreeForGround generateRandomQuadTree(Random random, Box bounds, QuadTreeForGroundParameters parameters, double minZ, double maxZ,
         int numberOfPoints)
   {
      QuadTreeForGround quadTree = new QuadTreeForGround(bounds, parameters);
      addRandomPointsToQuadTree(random, numberOfPoints, bounds, minZ, maxZ, quadTree);

      return quadTree;
   }

   private void addRandomPointsToQuadTree(Random random, int numberOfPoints, Box bounds, double minZ, double maxZ, QuadTreeForGround quadTree)
   {
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D point = RandomGeometry.nextPoint3D(random, bounds.minX, bounds.minY, minZ, bounds.maxX, bounds.maxY, maxZ);
         quadTree.put(point.getX(), point.getY(), point.getZ());
      }
   }

   private ArrayList<Point3D> generateRandomPoints(Random random, int numberOfPoints, Box bounds, double minZ, double maxZ)
   {
      ArrayList<Point3D> pointsToReturn = new ArrayList<Point3D>();
      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3D point = RandomGeometry.nextPoint3D(random, bounds.minX, bounds.minY, minZ, bounds.maxX, bounds.maxY, maxZ);
         pointsToReturn.add(point);
      }

      return pointsToReturn;
   }

   private double distanceXYSquared(double x, double y, Point3D point)
   {
      if (point == null)
         return Double.POSITIVE_INFINITY;

      return ((x - point.getX()) * (x - point.getX()) + (y - point.getY()) * (y - point.getY()));
   }

}
