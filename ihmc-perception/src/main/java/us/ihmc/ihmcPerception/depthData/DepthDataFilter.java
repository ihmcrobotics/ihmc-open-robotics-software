package us.ihmc.ihmcPerception.depthData;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.robotics.dataStructures.DecayingResolutionFilter;
import us.ihmc.robotics.quadTree.Box;
import us.ihmc.robotics.quadTree.QuadTreeForGroundParameters;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeForGroundHeightMap;
import us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree.QuadTreeHeightMapInterface;

public class DepthDataFilter
{

   public static final double QUAD_TREE_EXTENT = 200;
   protected final QuadTreeHeightMapInterface quadTree;
   protected final DecayingResolutionFilter nearScan;
   protected DepthDataFilterParameters parameters;

   // Adjustment which is applied to LIDAR points.  Must be applied while points are still in LIDAR frame.  Allows users correct for errors
   // See DRCManualLidarTransform and DRCLidarVisualizationManager. This is a bit of a hack but less likely to have unintended consequences.
   private final RigidBodyTransform worldToCorrected = new RigidBodyTransform();


   public DepthDataFilter()
   {
      this(DepthDataFilterParameters.getDefaultParameters());
   }
   
   public DepthDataFilter(DepthDataFilterParameters parameters)
   {
      this.parameters = parameters;
      nearScan = new DecayingResolutionFilter(parameters.nearScanResolution, parameters.nearScanDecayMillis, parameters.nearScanCapacity);
      quadTree = setupGroundOnlyQuadTree(parameters);

   }

   public static QuadTreeForGroundHeightMap setupGroundOnlyQuadTree(DepthDataFilterParameters parameters)
   {
      Box bounds = new Box(-QUAD_TREE_EXTENT, -QUAD_TREE_EXTENT, QUAD_TREE_EXTENT, QUAD_TREE_EXTENT);
      QuadTreeForGroundParameters quadTreeParameters = new QuadTreeForGroundParameters(DepthDataFilterParameters.GRID_RESOLUTION,
            parameters.quadtreeHeightThreshold, parameters.quadTreeMaxMultiLevelZChangeToFilterNoise, parameters.maxSameHeightPointsPerNode,
            parameters.maxAllowableXYDistanceForAPointToBeConsideredClose, parameters.maximumNumberOfPoints);

      return new QuadTreeForGroundHeightMap(bounds, quadTreeParameters);
   }


   public void setWorldToCorrected(RigidBodyTransform adjustment)
   {
      this.worldToCorrected.set(adjustment);
   }

   public boolean addNearScanPoint(Point3D point, Point3D sensorOrigin)
   {
      boolean send = false;
      if(!pointInRange(point, sensorOrigin))
         return false;

      // This is here so the user can manually correct for calibration errors.  It should only be not identity in the user interface
      if (DepthDataFilterParameters.LIDAR_ADJUSTMENT_ACTIVE)
         worldToCorrected.transform(point);

      if (parameters.nearScan && isValidNearScan(point, sensorOrigin))
      {
         send = nearScan.add(point.getX(), point.getY(), point.getZ()) || send;
      }

      return send;
   }
   public boolean addQuatreePoint(Point3D point, Point3D sensorOrigin)
   {
      boolean send = false;
      if(!pointInRange(point, sensorOrigin))
         return false;

      // This is here so the user can manually correct for calibration errors.  It should only be not identity in the user interface
      if (DepthDataFilterParameters.LIDAR_ADJUSTMENT_ACTIVE)
         worldToCorrected.transform(point);

      if (isValidPoint(point, sensorOrigin))
      {
         if (isPossibleGround(point, sensorOrigin))
         {
            send = quadTree.addPoint(point.getX(), point.getY(), point.getZ()) || send;
         }
      }

      return send;
   }

   public DecayingResolutionFilter getNearScan()
   {
      return nearScan;
   }

   public void setParameters(DepthDataFilterParameters parameters)
   {
      this.parameters = parameters;

      nearScan.setResolution(parameters.nearScanResolution);
      nearScan.setCapacity(parameters.nearScanCapacity);
      nearScan.setDecay(parameters.nearScanDecayMillis);

      quadTree.setHeightThreshold(parameters.quadtreeHeightThreshold);

   }

   public boolean addPoint(Point3D point,Point3D sensorOrigin)
   {
      return addNearScanPoint(point, sensorOrigin)| addQuatreePoint(point, sensorOrigin);
   }
   

   protected boolean isValidNearScan(Point3D point, Point3D lidarOrigin)
   {
      boolean valid = true;
      valid &= point.getZ() < lidarOrigin.getZ() + parameters.nearScanZMaxAboveHead;

      Point3D center = new Point3D(lidarOrigin.getX(), lidarOrigin.getY(), point.getZ());
      valid &= point.distance(center) < parameters.nearScanRadius;


      return valid;
   }

   protected boolean isValidPoint(Point3D point, Point3D lidarOrigin)
   {
      boolean valid = true;

      valid &= point.getZ() < lidarOrigin.getZ() + parameters.octreeZMaxAboveHead;;

      return valid;
   }

   protected boolean isPossibleGround(Point3D point, Point3D lidarOrigin)
   {
      final double footZ = 0;

      return (point.getZ() - footZ) < parameters.quadTreeZMax;
   }

   private boolean pointInRange(Point3D point, Point3D sensorOrigin)
   {
      double dist = sensorOrigin.distance(point);

      return (dist > parameters.minRange) && (dist < parameters.maxRange);
   }

}
