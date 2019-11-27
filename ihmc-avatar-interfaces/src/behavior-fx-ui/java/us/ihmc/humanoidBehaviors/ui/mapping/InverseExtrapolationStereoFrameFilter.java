package us.ihmc.humanoidBehaviors.ui.mapping;

import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class InverseExtrapolationStereoFrameFilter
{
   private static final boolean debug = true;

   private static final double RATIO_INLIER_POINTS = 0.75;
   private static final double ALLOWABLE_DISTANCE = 0.015;
   private static final double RATIO_OUT_OF_BOUND = 0.3;
   
   private static final double OCTREE_NODE_DISTANCE = 0.02;
   private static final double OCTREE_NODE_NORMAL_ANGLE = 15.0 / 180.0 * Math.PI;

   public boolean intersectionFilter(StereoVisionPointCloudMessage message, EnvironmentMap map)
   {
      PlanarRegionsList planarRegionsList = map.getPlanarRegionsMap();

      Point3D[] points = getPointsFromMessage(message);
      RigidBodyTransform sensorPose = getSensorPoseFromMessage(message);
      RigidBodyTransform previousSensorPose = map.getLatestSensorPoses();
      Point3D[] pointsInPreviousView = EnvironmentMappingTools.createPointsInPreviousView(sensorPose, previousSensorPose, points);

      double intersectionViewRatio = (double) (pointsInPreviousView.length / (double) points.length);
      if (debug)
         System.out.println("ratio in bound ! " + intersectionViewRatio);

      if (intersectionViewRatio < RATIO_OUT_OF_BOUND)
         return true;

      int numberOfPlanarRegions = planarRegionsList.getNumberOfPlanarRegions();
      int numberOfInlierPoints = 0;
      for (int i = 0; i < pointsInPreviousView.length; i++)
      {
         double minimumDistance = Double.MAX_VALUE;
         for (int j = 0; j < numberOfPlanarRegions; j++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegion(j);
            Plane3D plane = planarRegion.getPlane();
            minimumDistance = Math.min(plane.distance(pointsInPreviousView[i]), minimumDistance);
         }

         if (minimumDistance < ALLOWABLE_DISTANCE)
         {
            numberOfInlierPoints++;
         }
      }
      double inliersRatio = (double) (numberOfInlierPoints / (double) pointsInPreviousView.length);
      if (debug)
         System.out.println("RATIO_INLIER_POINTS " + (inliersRatio > RATIO_INLIER_POINTS) + " " + inliersRatio);

      if (inliersRatio > RATIO_INLIER_POINTS)
         return true;

      return false;
   }
   
   public boolean octreeNodeFilter(StereoVisionPointCloudMessage message, EnvironmentMap map)
   {
      PlanarRegionsList planarRegionsList = map.getPlanarRegionsMap();
      NormalOcTree occtreeData = EnvironmentMappingTools.computeOctreeData(message, map.getOctreeResolution());
      
      List<Plane3D> octreeNodePlanes = EnvironmentMappingTools.createOctreeNodePlanes(occtreeData);

      return false;
   }

   private Point3D[] getPointsFromMessage(StereoVisionPointCloudMessage message)
   {
      int numberOfPoints = message.getColors().size();
      Point3D[] pointCloud = new Point3D[numberOfPoints];
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointCloud[i] = new Point3D();
         MessageTools.unpackScanPoint(message, i, pointCloud[i]);
      }
      return pointCloud;
   }

   private RigidBodyTransform getSensorPoseFromMessage(StereoVisionPointCloudMessage message)
   {
      return new RigidBodyTransform(message.getSensorOrientation(), message.getSensorPosition());
   }

}
