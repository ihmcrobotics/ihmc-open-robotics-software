package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.gpuMapping.HeightMapData;
import us.ihmc.perception.gpuMapping.HeightMapMessageTools;
import us.ihmc.perception.gpuMapping.HeightMapTools;
import us.ihmc.perception.gpuMapping.TerrainMapData;
import us.ihmc.perception.detections.InstantDetection;
import us.ihmc.perception.gpuMapping.TerrainMapMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;

import java.util.List;

public class YOLOTerrainMapIntegrator
{
   private static final double OBSTACLE_HEIGHT = 1.0;
   private static final double INNER_MARKING_RADIUS = 0.1;
   private static final double OUTER_MARKING_RADIUS = 0.35;

   private final TerrainMapData yoloTerrain;
   private final ROS2Publisher<HeightMapMessage> yoloHeightMapPublisher;
   private final ROS2Publisher<TerrainMapMessage> yoloTerrainMapPublisher;
   private long yoloHeightMapSequenceId = 0;
   private long yoloTerrainMapSequenceId = 0;

   public YOLOTerrainMapIntegrator(ROS2Node ros2Node, double cellSize, double width)
   {
      yoloTerrain = new TerrainMapData(cellSize, width, 0.0, 0.0);

      yoloHeightMapPublisher  = ros2Node.createPublisher(PerceptionAPI.YOLO_HEIGHT_MAP);
      yoloTerrainMapPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_TERRAIN_MAP);
   }

   /**
    * Marks cells in the terrain map as obstacles based on YOLO detections.
    * Sets height to 1.0 for all cells within markingRadius of each detection's point cloud points.
    *
    * @param detections List of YOLO instant detections containing point clouds
    */
   public void integrateDetectionsIntoTerrainMap(List<InstantDetection> detections)
   {
      if (detections == null || detections.isEmpty())
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
               markObstacleRegion(yoloTerrain, x, y);
            }
         }
      }
   }

   public void clearYoloTerrain()
   {
      float[] heights = yoloTerrain.getHeightMap();
      float[] scores  = yoloTerrain.getTraversabilityScoreMap();
      for (int i = 0; i < heights.length; i++)
      {
         heights[i] = 0.0f;
         scores[i]  = 1.0f;
      }
   }

   /**
    * Marks a circular region around the given point as an obstacle in the terrain map.
    */
   private void markObstacleRegion(TerrainMapData terrainMapData, double centerX, double centerY)
   {
      double cellSize = terrainMapData.getCellSize();
      int radiusCells = (int) Math.ceil(OUTER_MARKING_RADIUS / cellSize);

      int centerXIndex = HeightMapTools.coordinateToIndex(centerX,
                                                          terrainMapData.getGridCenterX(),
                                                          cellSize,
                                                          terrainMapData.getCenterIndex());
      int centerYIndex = HeightMapTools.coordinateToIndex(centerY,
                                                          terrainMapData.getGridCenterY(),
                                                          cellSize,
                                                          terrainMapData.getCenterIndex());

      float[] heightMap = terrainMapData.getHeightMap();
      float[] traversabilityScoreMap = terrainMapData.getTraversabilityScoreMap();
      int cellsPerAxis = terrainMapData.getCellsPerAxis();
      int centerIndex = terrainMapData.getCenterIndex();

      double minDistance = INNER_MARKING_RADIUS;
      double maxDistance = OUTER_MARKING_RADIUS;
      double bandWidth   = maxDistance - minDistance;

      for (int dx = -radiusCells; dx <= radiusCells; dx++)
      {
         for (int dy = -radiusCells; dy <= radiusCells; dy++)
         {
            double offsetX = dx * cellSize;
            double offsetY = dy * cellSize;
            double distance = Math.sqrt(offsetX * offsetX + offsetY * offsetY);

            if (distance <= maxDistance)
            {
               int xIndex = centerXIndex + dx;
               int yIndex = centerYIndex + dy;

               if (xIndex >= 0 && xIndex < cellsPerAxis &&
                   yIndex >= 0 && yIndex < cellsPerAxis)
               {
                  int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);

                  // Traversability score: 0 at inner, 1 at outer ---
                  float score;
                  if (distance < minDistance || bandWidth <= 1e-6)
                  {
                     score = 0.0f;
                  }
                  else
                  {
                     score = (float) ((distance - minDistance) / bandWidth); // 0→1
                     if (score < 0.0f) score = 0.0f;
                     if (score > 1.0f) score = 1.0f;
                  }
                  if (score < traversabilityScoreMap[key])
                     traversabilityScoreMap[key] = score;

                  // Height map: OBSTACLE_HEIGHT at inner, 0 at outer ---
                  float height;
                  if (distance < minDistance || bandWidth <= 1e-6)
                  {
                     height = (float) OBSTACLE_HEIGHT;
                  }
                  else
                  {
                     double t = (distance - minDistance) / bandWidth; // 0 at inner, 1 at outer
                     t = Math.max(0.0, Math.min(1.0, t));
                     height = (float) ((1.0 - t) * OBSTACLE_HEIGHT); // linear to 0
                  }

                  // If multiple obstacles overlap, keep the max height
                  if (height > heightMap[key])
                     heightMap[key] = height;
               }
            }
         }
      }
   }

   public void publishTerrainMaps()
   {
      // Terrain (obstacle layer)
      TerrainMapMessage terrainMsg = new TerrainMapMessage();
      TerrainMapMessageTools.toMessage(yoloTerrain, terrainMsg);
      terrainMsg.setSequenceId(yoloTerrainMapSequenceId++);
      yoloTerrainMapPublisher.publish(terrainMsg);

      // Height map (just the same heights, packaged as HeightMapMessage)
      HeightMapMessage heightMsg = new HeightMapMessage();
      toHeightMapMessageFromTerrain(yoloTerrain, heightMsg);
      heightMsg.setSequenceId(yoloHeightMapSequenceId++);
      yoloHeightMapPublisher.publish(heightMsg);
   }

   public static void toHeightMapMessageFromTerrain(TerrainMapData terrainMapData, HeightMapMessage messageToPack)
   {
      HeightMapData heightMapData = new HeightMapData(terrainMapData.getCellSize(),
                                                      terrainMapData.getMapSize(),
                                                      terrainMapData.getGridCenterX(),
                                                      terrainMapData.getGridCenterY());
      heightMapData.setHeights(terrainMapData.getHeightMap());
      HeightMapMessageTools.toMessage(heightMapData, messageToPack);
   }

   public TerrainMapData getTerrainMap()
   {
      return yoloTerrain;
   }
}
