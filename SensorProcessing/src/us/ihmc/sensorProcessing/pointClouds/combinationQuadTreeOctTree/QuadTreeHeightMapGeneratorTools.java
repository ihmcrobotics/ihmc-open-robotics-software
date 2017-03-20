package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;

/**
 * Created by agrabertilton on 2/12/15.
 */
public class QuadTreeHeightMapGeneratorTools
{
   public static QuadTreeForGroundHeightMap createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap, BoundingBox2D testingRange, double resolution)
   {
      double heightThreshold = 0.002;
      return createHeightMap(inputHeightMap, testingRange, resolution, heightThreshold);
   }

   public static QuadTreeForGroundHeightMap createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap, BoundingBox2D testingRange, double resolution, double heightThreshold)
   {
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;
      return createHeightMap(inputHeightMap, testingRange, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);
   }



   public static QuadTreeForGroundHeightMap createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap, BoundingBox2D testingRange, double resolution, double heightThreshold,
                               double quadTreeMaxMultiLevelZChangeToFilterNoise, int maxSameHeightPointsPerNode,
                               double maxAllowableXYDistanceForAPointToBeConsideredClose, int maxNodes)
   {
      double minX = testingRange.getMinPoint().getX();
      double maxX = testingRange.getMaxPoint().getX();
      double minY = testingRange.getMinPoint().getY();
      double maxY = testingRange.getMaxPoint().getY();

      ArrayList<Point3D> listOfPoints = new ArrayList<Point3D>();

      for (double x = minX; x < maxX; x = x + resolution)
      {
         for (double y = minY; y < maxY; y = y + resolution)
         {
            double z = inputHeightMap.heightAt(x, y, 0.0);
            listOfPoints.add(new Point3D(x, y, z));
         }
      }

      return createHeightMap(listOfPoints, testingRange, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
            maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);
   }


   public static QuadTreeForGroundHeightMap createHeightMap(ArrayList<Point3D> listOfPoints, BoundingBox2D testingRange, double resolution, double heightThreshold,
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

      QuadTreeForGroundHeightMap heightMap = new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);

      for (Point3D point : listOfPoints)
      {
         heightMap.addPoint(point.getX(), point.getY(), point.getZ());
      }

      return heightMap;
   }
}
