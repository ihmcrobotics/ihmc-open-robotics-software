package us.ihmc.darpaRoboticsChallenge.networkProcessor.depthData;

import java.util.ArrayList;

import us.ihmc.utilities.math.geometry.RigidBodyTransform;

import javax.vecmath.Point3d;

import us.ihmc.SdfLoader.SDFFullRobotModel;
import us.ihmc.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.communication.packets.sensing.FilteredPointCloudPacket;
import us.ihmc.communication.packets.sensing.PointCloudPacket;
import us.ihmc.communication.packets.sensing.SparseLidarScanPacket;
import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.GroundOnlyQuadTree;
import us.ihmc.userInterface.util.DecayingResolutionFilter;
import us.ihmc.utilities.dataStructures.hyperCubeTree.Octree;
import us.ihmc.utilities.dataStructures.hyperCubeTree.OneDimensionalBounds;
import us.ihmc.utilities.dataStructures.hyperCubeTree.SphericalLinearResolutionProvider;
import us.ihmc.utilities.lidar.polarLidar.AbstractLidarScan;
import us.ihmc.utilities.lidar.polarLidar.LidarScan;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.robotSide.RobotSide;

public class DepthDataFilter
{
   public static final int OCTREE_MIN_BOXES = 20000;
   public static final int OCTREE_MAX_BOXES = 100000;
   public static final double QUAD_TREE_EXTENT = 200;

   private final GroundOnlyQuadTree quadTree;
   private final Octree octree;
   private final DecayingResolutionFilter nearScan;

   private final RobotBoundingBoxes robotBoundingBoxes;
   private final SDFFullRobotModel fullRobotModel;

   DepthDataFilterParameters parameters;

   // Adjustment which is applied to LIDAR points.  Must be applied while points are still in LIDAR frame.  Allows users correct for errors
   // See DRCManualLidarTransform and DRCLidarVisualizationManager. This is a bit of a hack but less likely to have unintended consequences.
   RigidBodyTransform worldToCorrected = new RigidBodyTransform();

   public DepthDataFilter(RobotBoundingBoxes robotBoundingBoxes, SDFFullRobotModel fullRobotModel)
   {
      this.robotBoundingBoxes = robotBoundingBoxes;
      this.fullRobotModel = fullRobotModel;

      this.parameters = DepthDataFilterParameters.getDefaultParameters();

      nearScan = new DecayingResolutionFilter(parameters.nearScanResolution, parameters.nearScanDecayMillis, parameters.nearScanCapacity);

      ReferenceFrame head = fullRobotModel.getHead().getBodyFixedFrame();
      quadTree = getGroundOnlyQuadTree(head);
      octree = getOctree(head);
      quadTree.setOctree(octree);
   }

   public void setWorldToCorrected(RigidBodyTransform adjustment) {
      this.worldToCorrected.set(adjustment);
   }

   public GroundOnlyQuadTree getGroundOnlyQuadTree(ReferenceFrame headFrame)
   {
      // SphericalLinearResolutionProvider resolutionProvider = new SphericalLinearResolutionProvider(new FramePoint(headFrame,
      // 0.0, 0.0, -2.0), DRCConfigParameters.LIDAR_RESOLUTION_SPHERE_INNER_RADIUS*3,
      // DRCConfigParameters.LIDAR_RESOLUTION_SPHERE_INNER_RESOLUTION, DRCConfigParameters.LIDAR_RESOLUTION_SPHERE_OUTER_RADIUS*3,
      // DRCConfigParameters.LIDAR_RESOLUTION_SPHERE_OUTER_RESOLUTION);
      return new GroundOnlyQuadTree(-QUAD_TREE_EXTENT, -QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, DRCConfigParameters.GRID_RESOLUTION,
            parameters.quadtreeHeightThreshold, 100000);
   }

   public Octree getOctree(ReferenceFrame headFrame)
   {
      if (parameters.USE_RESOLUTION_SPHERE)
      {
         SphericalLinearResolutionProvider resolutionProvider = new SphericalLinearResolutionProvider(new FramePoint(headFrame,
               parameters.LIDAR_RESOLUTION_SPHERE_DISTANCE_FROM_HEAD, 0.0, 0.0), parameters.LIDAR_RESOLUTION_SPHERE_INNER_RADIUS,
               parameters.LIDAR_RESOLUTION_SPHERE_INNER_RESOLUTION, parameters.LIDAR_RESOLUTION_SPHERE_OUTER_RADIUS,
               parameters.LIDAR_RESOLUTION_SPHERE_OUTER_RESOLUTION);

         return new Octree(new OneDimensionalBounds[] { new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT),
               new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT), new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT) },
               resolutionProvider);
      }

      return new Octree(new OneDimensionalBounds[] { new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT),
            new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT), new OneDimensionalBounds(-QUAD_TREE_EXTENT, QUAD_TREE_EXTENT) },
            DRCConfigParameters.OCTREE_RESOLUTION_WHEN_NOT_USING_RESOLUTION_SPHERE);
   }

   public SparseLidarScanPacket filterPolarLidarScan(LidarScan lidarScan)
   {
      ArrayList<Integer> indexes = new ArrayList<Integer>();
      for (int i = 0; i < lidarScan.size(); i++)
      {
         if (addPoint(lidarScan, i))
         {
            indexes.add(i);
         }
      }

      return new SparseLidarScanPacket(lidarScan, indexes);
   }

   public boolean addPoint(AbstractLidarScan lidarScan, int i)
   {
      Point3d lidarOrigin = new Point3d();
      lidarScan.getAverageTransform().transform(lidarOrigin);
      boolean send = false;

      if (rayInRange(lidarScan.getRange(i)))
      {
         Point3d point = lidarScan.getPoint(i);

         // This is here so the user can manually correct for calibration errors.  It should only be not identity in the user interface
         if( DRCConfigParameters.LIDAR_ADJUSTMENT_ACTIVE )
            worldToCorrected.transform(point);

         if (parameters.nearScan && isValidNearScan(point, lidarOrigin))
         {
            send = nearScan.add(point.x, point.y, point.z) || send;
         }

         if (isValidOctree(point, lidarOrigin))
         {
            if (isPossibleGround(point, lidarOrigin)) {
               send = quadTree.addPoint(point.x, point.y, point.z) || send;
            }
            else {
               send = quadTree.addPointToOctree(point.x, point.y, point.z) || send;
            }
         }
      }

      return send;
   }
   
   public boolean addPoint(Point3d point, RigidBodyTransform transform)
   {
      Point3d sensorOrigin = new Point3d();
      transform.transform(sensorOrigin);
      boolean send = false;

      if (pointInRange(point,sensorOrigin))
      {
         // This is here so the user can manually correct for calibration errors.  It should only be not identity in the user interface
         if( DRCConfigParameters.LIDAR_ADJUSTMENT_ACTIVE )
            worldToCorrected.transform(point);

         if (parameters.nearScan && isValidNearScan(point, sensorOrigin))
         {
            send = nearScan.add(point.x, point.y, point.z) || send;
         }

         if (isValidOctree(point, sensorOrigin))
         {
            if (isPossibleGround(point, sensorOrigin)) {
               send = quadTree.addPoint(point.x, point.y, point.z) || send;
            }
            else {
               send = quadTree.addPointToOctree(point.x, point.y, point.z) || send;
            }
         }
      } 

      return send;
   }

   public boolean isValidNearScan(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = true;
      valid &= point.z > getMidFootPoint().z + parameters.nearScanZMinAboveFeet;
      valid &= point.z < lidarOrigin.z + parameters.nearScanZMaxAboveHead;

      Point3d center = new Point3d(lidarOrigin.x, lidarOrigin.y, point.z);
      valid &= point.distance(center) < parameters.nearScanRadius;

      valid &= Math.abs(getAngleToPelvis(point, lidarOrigin)) < parameters.nearScanRadians;
      valid &= parameters.nearScanCollisions || robotBoundingBoxes.isValidPoint(lidarOrigin, point);

      return valid;
   }

   //TODO: isAheadOfPelvis must be commented out when debugging val currently
   public boolean isValidOctree(Point3d point, Point3d lidarOrigin)
   {
      boolean valid = true;

      valid &= point.getZ() < lidarOrigin.getZ() + parameters.octreeZMaxAboveHead;;
      valid &= isAheadOfPelvis(point);
      valid &= robotBoundingBoxes.isValidPoint(lidarOrigin, point);

      return valid;
   }
   
   public Point3d getMidFootPoint() 
   {
      RigidBodyTransform temp = new RigidBodyTransform();
      Point3d left = new Point3d();
      Point3d avg = new Point3d();

      fullRobotModel.getFoot(RobotSide.LEFT).getBodyFixedFrame().getTransformToDesiredFrame(temp, ReferenceFrame.getWorldFrame());
      temp.transform(left);
      fullRobotModel.getFoot(RobotSide.RIGHT).getBodyFixedFrame().getTransformToDesiredFrame(temp, ReferenceFrame.getWorldFrame());
      temp.transform(avg);
      
      avg.add(left);
      avg.scale(0.5);
      return avg;
   }
   
   public boolean isPossibleGround(Point3d point, Point3d lidarOrigin) {
      Point3d footAvg = getMidFootPoint();
      
      double footZ = footAvg.z;
      footAvg.setZ(point.z);
      
      double maxHeight = parameters.quadTreeZAboveFeet + point.distance(footAvg) * parameters.quadTreeZSlope;
      if (maxHeight > parameters.quadTreeZMax) {
         maxHeight = parameters.quadTreeZMax;
      }
      
      return (point.z - footZ) < maxHeight;
   }

   public double getAngleToPelvis(Point3d point, Point3d lidarOrigin)
   {
      RigidBodyTransform tf = new RigidBodyTransform();
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tf, fullRobotModel.getPelvis().getBodyFixedFrame());
      Point3d tfPoint = new Point3d(point);
      tf.transform(tfPoint);

      return Math.atan2(tfPoint.y, tfPoint.x);
   }

   private boolean isAheadOfPelvis(Point3d point)
   {
      RigidBodyTransform tf = new RigidBodyTransform();
      ReferenceFrame.getWorldFrame().getTransformToDesiredFrame(tf, fullRobotModel.getPelvis().getBodyFixedFrame());
      Point3d tfPoint = new Point3d(point);
      tf.transform(tfPoint);

      return tfPoint.x > parameters.xCutoffPelvis;
   }

   private boolean rayInRange(double rayLength)
   {
      return rayLength > parameters.minRange && rayLength < parameters.maxRange;
   }
   
   private boolean pointInRange(Point3d point, Point3d sensorOrigin)
   {
      double dist = sensorOrigin.distance(point);
      return dist > parameters.minRange && dist < parameters.maxRange;
   }

   // TODO get rid of this and integrate with parameters
   public void setLidarState(LidarState lidarState)
   {
      if (lidarState == LidarState.ENABLE)
      {
         quadTree.setUpdateOctree(true);
      }
      else if (lidarState == LidarState.ENABLE_QUAD_TREE_ONLY)
      {
         quadTree.setUpdateOctree(false);
      }
   }

   public void clearLidarData(DepthDataTree lidarTree)
   {
      nearScan.clear();
      switch (lidarTree)
      {
      case OCTREE:
         octree.clearTree();
         break;

      case QUADTREE:
         quadTree.clearTree();
         break;

      default:
         throw new RuntimeException("Unknown tree");
      }
   }

   public void setParameters(DepthDataFilterParameters parameters)
   {
      this.parameters = parameters;

      nearScan.setResolution(parameters.nearScanResolution);
      nearScan.setCapacity(parameters.nearScanCapacity);
      nearScan.setDecay(parameters.nearScanDecayMillis);

      quadTree.setHeighThreshold(parameters.quadtreeHeightThreshold);

      robotBoundingBoxes.setScale(parameters.boundingBoxScale);
   }

   public GroundOnlyQuadTree getQuadTree()
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
         if (addPoint(points[i],transformToWorld))
         {
            filteredPoints.add(points[i]);
         }
      }

      return new FilteredPointCloudPacket(filteredPoints, transformToWorld, pointCloud.getTimeStamp());
   }
}
