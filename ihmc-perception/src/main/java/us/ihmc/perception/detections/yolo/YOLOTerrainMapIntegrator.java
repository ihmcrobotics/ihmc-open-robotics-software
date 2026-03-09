package us.ihmc.perception.detections.yolo;

import perception_msgs.msg.dds.HeightMapMessage;
import perception_msgs.msg.dds.TerrainMapMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
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

   private final double terrainWidth;
   private final TerrainMapData yoloTerrain;
   private volatile TerrainMapData latestSnapshotTerrain;
   private final ROS2Publisher<HeightMapMessage> yoloHeightMapPublisher;
   private final ROS2Publisher<TerrainMapMessage> yoloTerrainMapPublisher;
   private long yoloHeightMapSequenceId = 0;
   private long yoloTerrainMapSequenceId = 0;

   public YOLOTerrainMapIntegrator(ROS2Node ros2Node, double cellSize, double width, ReferenceFrame cameraFrame)
   {
      this.terrainWidth = width;
      yoloTerrain = new TerrainMapData(cellSize, width, 0, 0);

      yoloHeightMapPublisher  = ros2Node.createPublisher(PerceptionAPI.YOLO_HEIGHT_MAP);
      yoloTerrainMapPublisher = ros2Node.createPublisher(PerceptionAPI.YOLO_TERRAIN_MAP);
   }

   public void updateTerrainCenterFromCamera(ReferenceFrame cameraFrame)
   {
      RigidBodyTransform cameraToWorld = cameraFrame.getTransformToRoot();

      double cameraX = cameraToWorld.getTranslationX();
      double cameraY = cameraToWorld.getTranslationY();

      // Forward offset (camera X axis) by width/2 - 0.25
      double forwardOffset = terrainWidth / 2.0 - 0.25;

      // Assuming yaw-only rotation; camera X axis projected into world XY
      double cosYaw = cameraToWorld.getM00(); // world X component of camera X
      double sinYaw = cameraToWorld.getM10(); // world Y component of camera X

      double gridCenterX = cameraX + forwardOffset * cosYaw;
      double gridCenterY = cameraY + forwardOffset * sinYaw;

      yoloTerrain.setGridCenterX(gridCenterX);
      yoloTerrain.setGridCenterY(gridCenterY);
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
         if (detection instanceof YOLOv8InstantDetection yoloDetection)
         {
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
      float[] scores  = yoloTerrain.getObstacleClearanceScoreMap();
      float[] traversabilityScores  = yoloTerrain.getTraversabilityScoreMap();
      for (int i = 0; i < heights.length; i++)
      {
         heights[i] = 0.0f;
         scores[i]  = 1.0f;
         traversabilityScores[i] = 1.0f;
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
      float[] obstacleClearanceScoreMap = terrainMapData.getObstacleClearanceScoreMap();
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
            double distance = EuclidCoreTools.norm(offsetX, offsetY);

            int xIndex = centerXIndex + dx;
            int yIndex = centerYIndex + dy;

            if (distance <= maxDistance && xIndex >= 0 && xIndex < cellsPerAxis && yIndex >= 0 && yIndex < cellsPerAxis)
            {
               int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);

               // Traversability score: 0 at inner, 1 at outer ---
               float score = 0.0f;
               // Height map: OBSTACLE_HEIGHT at inner, 0 at outer ---
               float height = (float) OBSTACLE_HEIGHT;

               if (distance > minDistance && bandWidth > 1e-6)
               {
                  // Compute interpolated score
                  score = (float) EuclidCoreTools.clamp((distance - minDistance) / bandWidth, 0.0, 1.0); // 0→1

                  // Compute interpolated height
                  height = (float) ((1.0 - score) * OBSTACLE_HEIGHT); // linear 1→0
               }

               // If multiple obstacles overlap, keep the min score
               if (score < obstacleClearanceScoreMap[key])
               {
                  obstacleClearanceScoreMap[key] = score;
                  traversabilityScoreMap[key] = score;
               }

               // If multiple obstacles overlap, keep the max height
               if (height > heightMap[key])
                  heightMap[key] = height;
            }
         }
      }
   }

   public void publishTerrainMaps()
   {
      // Create a fresh snapshot from yoloTerrain
      TerrainMapData snapshot = new TerrainMapData(yoloTerrain.getCellSize(),
                                                   yoloTerrain.getMapSize(),
                                                   yoloTerrain.getGridCenterX(),
                                                   yoloTerrain.getGridCenterY());
      copyTerrainContents(yoloTerrain, snapshot);

      HeightMapMessage heightMsg = new HeightMapMessage();
      toHeightMapMessageFromTerrain(snapshot, heightMsg);
      heightMsg.setSequenceId(yoloHeightMapSequenceId++);
      yoloHeightMapPublisher.publish(heightMsg);

      // Atomically make this snapshot available to other threads
      latestSnapshotTerrain = snapshot;
   }

   private static void copyTerrainContents(TerrainMapData source, TerrainMapData target)
   {
      // Basic metadata already set by constructor (cell size, center, mapSize)
      target.setGridCenterX(source.getGridCenterX());
      target.setGridCenterY(source.getGridCenterY());

      float[] srcHeights = source.getHeightMap();
      float[] srcTrav    = source.getTraversabilityScoreMap();
      float[] srcObs     = source.getObstacleClearanceScoreMap();

      float[] dstHeights = target.getHeightMap();
      float[] dstTrav    = target.getTraversabilityScoreMap();
      float[] dstObs     = target.getObstacleClearanceScoreMap();

      // Arrays are same length by construction
      System.arraycopy(srcHeights, 0, dstHeights, 0, srcHeights.length);
      System.arraycopy(srcTrav,    0, dstTrav,    0, srcTrav.length);
      System.arraycopy(srcObs,     0, dstObs,     0, srcObs.length);
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
      TerrainMapData snapshot = latestSnapshotTerrain;
      if (snapshot != null)
         return snapshot;
      else
         return yoloTerrain;
   }
}
