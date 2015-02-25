package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.communication.packets.sensing.PointCloudPacket;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.GroundOnlyQuadTree;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeHeightMapInterface;
import us.ihmc.userInterface.util.DecayingResolutionFilter;
import us.ihmc.utilities.dataStructures.quadTree.Box;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGroundParameters;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class DepthDataFilter
{
   private static final boolean USE_SIMPLIFIED_QUAD_TREE = true;
   public static final double QUAD_TREE_EXTENT = 200;

   private final QuadTreeHeightMapInterface quadTree;
   private final DecayingResolutionFilter nearScan;



   protected DepthDataFilterParameters parameters;

   // Adjustment which is applied to LIDAR points.  Must be applied while points are still in LIDAR frame.  Allows users correct for errors
   // See DRCManualLidarTransform and DRCLidarVisualizationManager. This is a bit of a hack but less likely to have unintended consequences.
   private final RigidBodyTransform worldToCorrected = new RigidBodyTransform();


   public DepthDataFilter(ReferenceFrame headFrame)
   {
      this.parameters = DepthDataFilterParameters.getDefaultParameters();
      nearScan = new DecayingResolutionFilter(parameters.nearScanResolution, parameters.nearScanDecayMillis, parameters.nearScanCapacity);
      quadTree = setupGroundOnlyQuadTree(parameters);
   }

   public void setWorldToCorrected(RigidBodyTransform adjustment)
   {
      this.worldToCorrected.set(adjustment);
   }

   public static QuadTreeHeightMapInterface setupGroundOnlyQuadTree(DepthDataFilterParameters parameters)
   {
      // SphericalLinearResolutionProvider resolutionProvider = new SphericalLinearResolutionProvider(new FramePoint(headFrame,
      // 0.0, 0.0, -2.0), DRCConfigParameters.LIDAR_RESOLUTION_SPHERE_INNER_RADIUS*3,
      // DRCConfigParameters.LIDAR_RESOLUTION_SPHERE_INNER_RESOLUTION, DRCConfigParameters.LIDAR_RESOLUTION_SPHERE_OUTER_RADIUS*3,
      // DRCConfigParameters.LIDAR_RESOLUTION_SPHERE_OUTER_RESOLUTION);

      if (USE_SIMPLIFIED_QUAD_TREE)
      {
         Box bounds = new Box(-QUAD_TREE_EXTENT, -QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, QUAD_TREE_EXTENT);
         QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(DepthDataFilterParameters.GRID_RESOLUTION,
                                                             parameters.quadtreeHeightThreshold, parameters.quadTreeMaxMultiLevelZChangeToFilterNoise,
                                                             parameters.maxSameHeightPointsPerNode,
                                                             parameters.maxAllowableXYDistanceForAPointToBeConsideredClose);

         return new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);
      }

      return new GroundOnlyQuadTree(-QUAD_TREE_EXTENT, -QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, DepthDataFilterParameters.GRID_RESOLUTION,
                                    parameters.quadtreeHeightThreshold, 100000);
   }



   public boolean addPoint(Point3d point, Point3d sensorOrigin)
   {
      boolean send = false;
      if(!pointInRange(point, sensorOrigin))
         return false;

      // This is here so the user can manually correct for calibration errors.  It should only be not identity in the user interface
      if (DepthDataFilterParameters.LIDAR_ADJUSTMENT_ACTIVE)
         worldToCorrected.transform(point);

      if (parameters.nearScan && isValidNearScan(point, sensorOrigin))
      {
         send = nearScan.add(point.x, point.y, point.z) || send;
      }

      if (isValidPoint(point, sensorOrigin))
      {
         if (isPossibleGround(point, sensorOrigin))
         {
            send = quadTree.addPoint(point.x, point.y, point.z) || send;
         }
      }

      return send;
   }

   protected boolean isValidNearScan(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = true;
      valid &= point.z < lidarOrigin.z + parameters.nearScanZMaxAboveHead;

      Point3d center = new Point3d(lidarOrigin.x, lidarOrigin.y, point.z);
      valid &= point.distance(center) < parameters.nearScanRadius;


      return valid;
   }

   protected boolean isValidPoint(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = true;

      valid &= point.getZ() < lidarOrigin.getZ() + parameters.octreeZMaxAboveHead;;

      return valid;
   }

   protected boolean isPossibleGround(Point3d point, Point3d lidarOrigin)
   {
      final double footZ = 0;

      return (point.z - footZ) < parameters.quadTreeZMax;
   }

   private boolean pointInRange(Point3d point, Point3d sensorOrigin)
   {
      double dist = sensorOrigin.distance(point);

      return (dist > parameters.minRange) && (dist < parameters.maxRange);
   }

   public void clearLidarData(DepthDataTree lidarTree)
   {
      nearScan.clear();

      switch (lidarTree)
      {
         case DECAY_POINT_CLOUD :
            nearScan.clear();
            break;

         case QUADTREE :
            quadTree.clearTree(Double.NaN); 

            break;

         default :
            throw new RuntimeException("Unknown tree");
      }
   }

   public void setParameters(DepthDataFilterParameters parameters)
   {
      this.parameters = parameters;

      nearScan.setResolution(parameters.nearScanResolution);
      nearScan.setCapacity(parameters.nearScanCapacity);
      nearScan.setDecay(parameters.nearScanDecayMillis);

      quadTree.setHeightThreshold(parameters.quadtreeHeightThreshold);

   }

   public QuadTreeHeightMapInterface getQuadTree()
   {
      return quadTree;
   }
   
   public DecayingResolutionFilter getNearScan()
   {
      return nearScan;
   }
   
   public DepthDataFilterParameters getParameters()
   {
      return parameters;
   }   
   
   @Deprecated
   public PointCloudPacket filterPolarLidarScan(LidarScan lidarScan)
   {
      ArrayList<Point3d> points = new ArrayList<>();
      for (int i = 0; i < lidarScan.size(); i++)
      {
         addLidarScan(lidarScan, i, points);
      }
      Point3d origin = new Point3d();
      Vector3d worldVector = new Vector3d();
      lidarScan.getAverageTransform().get(worldVector);
      origin.set(worldVector);
      return new PointCloudPacket(origin, points.toArray(new Point3d[points.size()]), lidarScan.params.timestamp);

   }
   
   @Deprecated
   private void addLidarScan(LidarScan lidarScan, int i, ArrayList<Point3d> points)
   {
      Point3d lidarOrigin = new Point3d();
      lidarScan.getAverageTransform().transform(lidarOrigin);

  
         Point3d point = lidarScan.getPoint(i);
         if(addPoint(point, lidarOrigin))
         {
            points.add(point);
         }
   }
}
