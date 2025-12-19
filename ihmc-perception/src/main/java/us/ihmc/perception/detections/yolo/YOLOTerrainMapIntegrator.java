package us.ihmc.perception.detections.yolo;

import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.detections.InstantDetection;

import java.util.List;

public class YOLOTerrainMapIntegrator
{
   private static final double OBSTACLE_HEIGHT = 1000.0;
   private final double markingRadius; // Radius around detection point to mark as obstacle

   public YOLOTerrainMapIntegrator(double markingRadius)
   {
      this.markingRadius = markingRadius;
   }

   /**
    * Marks cells in the terrain map as obstacles based on YOLO detections.
    * Sets height to 100.0 for all cells within markingRadius of each detection's point cloud points.
    * 
    * @param terrainMapData The terrain map to modify
    * @param detections List of YOLO instant detections containing point clouds
    */
   public void integrateDetectionsIntoTerrainMap(TerrainMapData terrainMapData, List<InstantDetection> detections)
   {
      if (terrainMapData == null || detections == null || detections.isEmpty())
         return;

      for (InstantDetection detection : detections)
      {
         if (detection instanceof YOLOv8InstantDetection)
         {
            YOLOv8InstantDetection yoloDetection = (YOLOv8InstantDetection) detection;
            List<Point3D32> pointCloud = yoloDetection.getObjectPointCloud();
            
            if (pointCloud == null || pointCloud.isEmpty())
               continue;

            // Mark all points in the point cloud as obstacles
            for (Point3D32 point : pointCloud)
            {
               if (point.containsNaN())
                  continue;

               // Convert point to terrain map coordinates
               double x = point.getX();
               double y = point.getY();
               
               // Mark the cell and surrounding cells within radius
               markObstacleRegion(terrainMapData, x, y);
            }
         }
      }
   }

   /**
    * Marks a circular region around the given point as an obstacle in the terrain map.
    */
   private void markObstacleRegion(TerrainMapData terrainMapData, double centerX, double centerY)
   {
      double cellSize = terrainMapData.getCellSize();
      int radiusCells = (int) Math.ceil(markingRadius / cellSize);
      
      int centerXIndex = HeightMapTools.coordinateToIndex(centerX, 
                                                          terrainMapData.getGridCenterX(), 
                                                          cellSize, 
                                                          terrainMapData.getCenterIndex());
      int centerYIndex = HeightMapTools.coordinateToIndex(centerY, 
                                                          terrainMapData.getGridCenterY(), 
                                                          cellSize, 
                                                          terrainMapData.getCenterIndex());

      // Mark cells in a circular pattern
      for (int dx = -radiusCells; dx <= radiusCells; dx++)
      {
         for (int dy = -radiusCells; dy <= radiusCells; dy++)
         {
            double offsetX = dx * cellSize;
            double offsetY = dy * cellSize;
            double distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);
            
            if (distance <= markingRadius)
            {
               int xIndex = centerXIndex + dx;
               int yIndex = centerYIndex + dy;
               
               // Check bounds
               if (xIndex >= 0 && xIndex < terrainMapData.getCellsPerAxis() &&
                   yIndex >= 0 && yIndex < terrainMapData.getCellsPerAxis())
               {
                  int key = HeightMapTools.indicesToKey(xIndex, yIndex, terrainMapData.getCenterIndex());
                  terrainMapData.getHeightMap()[key] = (float) OBSTACLE_HEIGHT;
               }
            }
         }
      }
   }

   public double getMarkingRadius()
   {
      return markingRadius;
   }
}
