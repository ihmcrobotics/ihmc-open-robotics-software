package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;

import javax.vecmath.Point3d;

import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.robotics.geometry.BoundingBox2d;

/**
 * Created by agrabertilton on 2/12/15.
 */
public class QuadTreeHeightMapGeneratorTools
{
   public static QuadTreeForGroundHeightMap createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap, BoundingBox2d testingRange, double resolution)
   {
      double heightThreshold = 0.002;
      return createHeightMap(inputHeightMap, testingRange, resolution, heightThreshold);
   }

   public static QuadTreeForGroundHeightMap createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap, BoundingBox2d testingRange, double resolution, double heightThreshold)
   {
      double quadTreeMaxMultiLevelZChangeToFilterNoise = 0.2;
      int maxSameHeightPointsPerNode = 20;
      double maxAllowableXYDistanceForAPointToBeConsideredClose = 0.2;
      int maxNodes = 1000000;
      return createHeightMap(inputHeightMap, testingRange, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode, maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);
   }



   public static QuadTreeForGroundHeightMap createHeightMap(us.ihmc.graphicsDescription.HeightMap inputHeightMap, BoundingBox2d testingRange, double resolution, double heightThreshold,
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
         }
      }

      return createHeightMap(listOfPoints, testingRange, resolution, heightThreshold, quadTreeMaxMultiLevelZChangeToFilterNoise, maxSameHeightPointsPerNode,
            maxAllowableXYDistanceForAPointToBeConsideredClose, maxNodes);
   }


   public static QuadTreeForGroundHeightMap createHeightMap(ArrayList<Point3d> listOfPoints, BoundingBox2d testingRange, double resolution, double heightThreshold,
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

      for (Point3d point : listOfPoints)
      {
         heightMap.addPoint(point.getX(), point.getY(), point.getZ());
      }

      return heightMap;
   }
}
