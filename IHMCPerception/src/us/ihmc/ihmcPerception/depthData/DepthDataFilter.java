package us.ihmc.ihmcPerception.depthData;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.communication.packets.sensing.FilteredPointCloudPacket;
import us.ihmc.communication.packets.sensing.PointCloudPacket;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.GroundOnlyQuadTree;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeHeightMapInterface;
import us.ihmc.userInterface.util.DecayingResolutionFilter;
import us.ihmc.utilities.dataStructures.hyperCubeTree.Octree;
import us.ihmc.utilities.dataStructures.hyperCubeTree.OneDimensionalBounds;
import us.ihmc.utilities.dataStructures.hyperCubeTree.SphericalLinearResolutionProvider;
import us.ihmc.utilities.dataStructures.quadTree.Box;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGroundParameters;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;

public class DepthDataFilter
{
   private static final boolean USE_SIMPLIFIED_QUAD_TREE = true;

   public static final int OCTREE_MIN_BOXES = 20000;
   public static final int OCTREE_MAX_BOXES = 100000;
   public static final double QUAD_TREE_EXTENT = 200;

   private final QuadTreeHeightMapInterface quadTree;
   private final Octree octree;
   private final DecayingResolutionFilter nearScan;



   protected DepthDataFilterParameters parameters;

   // Adjustment which is applied to LIDAR points.  Must be applied while points are still in LIDAR frame.  Allows users correct for errors
   // See DRCManualLidarTransform and DRCLidarVisualizationManager. This is a bit of a hack but less likely to have unintended consequences.
   private final RigidBodyTransform worldToCorrected = new RigidBodyTransform();


   public DepthDataFilter(ReferenceFrame headFrame)
   {
      this.parameters = DepthDataFilterParameters.getDefaultParameters();
      nearScan = new DecayingResolutionFilter(parameters.nearScanResolution, parameters.nearScanDecayMillis, parameters.nearScanCapacity);
      quadTree = setupGroundOnlyQuadTree();
      octree = setupOctree(headFrame);
      quadTree.setOctree(octree);
   }

   public void setWorldToCorrected(RigidBodyTransform adjustment)
   {
      this.worldToCorrected.set(adjustment);
   }

   private QuadTreeHeightMapInterface setupGroundOnlyQuadTree()
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

   private Octree setupOctree(ReferenceFrame headFrame)
   {
      if (parameters.USE_RESOLUTION_SPHERE)
      {
         SphericalLinearResolutionProvider resolutionProvider = new SphericalLinearResolutionProvider(new FramePoint(headFrame,
                                                                   parameters.LIDAR_RESOLUTION_SPHERE_DISTANCE_FROM_HEAD, 0.0,
                                                                   0.0), parameters.LIDAR_RESOLUTION_SPHERE_INNER_RADIUS,
                                                                      parameters.LIDAR_RESOLUTION_SPHERE_INNER_RESOLUTION,
                                                                      parameters.LIDAR_RESOLUTION_SPHERE_OUTER_RADIUS,
                                                                      parameters.LIDAR_RESOLUTION_SPHERE_OUTER_RESOLUTION);

         return new Octree(new OneDimensionalBounds[] {new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT),
                 new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT),
                 new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT)}, resolutionProvider);
      }

      return new Octree(new OneDimensionalBounds[] {new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT),
              new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT),
              new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT)}, DepthDataFilterParameters.OCTREE_RESOLUTION_WHEN_NOT_USING_RESOLUTION_SPHERE);
   }

   public PointCloudPacket filterPolarLidarScan(LidarScan lidarScan)
   {
      ArrayList<Point3d> points = new ArrayList<>();
      for (int i = 0; i < lidarScan.size(); i++)
      {
         addPoint(lidarScan, i, points);
      }
      Point3d origin = new Point3d();
      Vector3d worldVector = new Vector3d();
      lidarScan.getAverageTransform().get(worldVector);
      origin.set(worldVector);
      return new PointCloudPacket(origin, points.toArray(new Point3d[points.size()]), lidarScan.params.timestamp);

   }
   
   private void addPoint(LidarScan lidarScan, int i, ArrayList<Point3d> points)
   {
      Point3d lidarOrigin = new Point3d();
      lidarScan.getAverageTransform().transform(lidarOrigin);

      if (rayInRange(lidarScan.getRange(i)))
      {
         Point3d point = lidarScan.getPoint(i);

         if(addPoint(point, lidarOrigin))
         {
            points.add(point);
         }
      }
   }

   public boolean addPoint(Point3d point, RigidBodyTransform transform)
   {
      Point3d sensorOrigin = new Point3d();
      transform.transform(sensorOrigin);
      if (pointInRange(point, sensorOrigin))
         return addPoint(point, sensorOrigin);
      else
         return false;

   }

   public boolean addPoint(Point3d point, Point3d sensorOrigin)
   {
      boolean send = false;

      // This is here so the user can manually correct for calibration errors.  It should only be not identity in the user interface
      if (DepthDataFilterParameters.LIDAR_ADJUSTMENT_ACTIVE)
         worldToCorrected.transform(point);

      if (parameters.nearScan && isValidNearScan(point, sensorOrigin))
      {
         send = nearScan.add(point.x, point.y, point.z) || send;
      }

      if (isValidOctree(point, sensorOrigin))
      {
         if (isPossibleGround(point, sensorOrigin))
         {
            send = quadTree.addPoint(point.x, point.y, point.z) || send;
         }
         else
         {
            send = quadTree.addPointToOctree(point.x, point.y, point.z) || send;
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

   protected boolean isValidOctree(Point3d point, Point3d lidarOrigin)
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

   private boolean rayInRange(double rayLength)
   {
      return (rayLength > parameters.minRange) && (rayLength < parameters.maxRange);
   }

   private boolean pointInRange(Point3d point, Point3d sensorOrigin)
   {
      double dist = sensorOrigin.distance(point);

      return (dist > parameters.minRange) && (dist < parameters.maxRange);
   }

   // TODO get rid of this and integrate with parameters
   public void setLidarState(LidarState lidarState)
   {
      if (lidarState == LidarState.ENABLE)
      {
         quadTree.setUpdateOctree(true);
      }
   }

   public void clearLidarData(DepthDataTree lidarTree)
   {
      nearScan.clear();

      switch (lidarTree)
      {
         case OCTREE :
            octree.clearTree();

            break;

         case QUADTREE :
            quadTree.clearTree();

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

   public Octree getOctree()
   {
      return octree;
   }

   public DecayingResolutionFilter getNearScan()
   {
      return nearScan;
   }
   
   public DepthDataFilterParameters getParameters()
   {
      return parameters;
   }

   public FilteredPointCloudPacket filterAndTransformPointCloud(PointCloudPacket pointCloud, RigidBodyTransform transformToWorld)
   {
      Point3d[] points = pointCloud.getPoints();
      ArrayList<Point3d> filteredPoints = new ArrayList<Point3d>();

      for (int i = 0; i < points.length; i++)
      {
         transformToWorld.transform(points[i]);

         if (addPoint(points[i], transformToWorld))
         {
            filteredPoints.add(points[i]);
         }
      }

      return new FilteredPointCloudPacket(pointCloud.origin, filteredPoints, transformToWorld, pointCloud.getTimeStamp());
   }
}
