package us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.octreeBasedSurfaceElement;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidBehaviors.ui.mapping.IhmcSLAMTools;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.AbstractSLAM;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrame;
import us.ihmc.humanoidBehaviors.ui.mapping.ihmcSlam.SLAMFrameOptimizerCostFunction;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class OctreeICPSLAMFrame extends SLAMFrame
{
   private NormalOcTree octreeNodesInPreviousView;
   private List<IhmcSurfaceElement> mergeableSurfaceElements = new ArrayList<>();

   private static final double VALID_PLANES_RATIO_THRESHOLD = 0.1;

   private static final double MAXIMUM_DISTANCE_OF_SIMILARITY = 0.1;
   private static final double MAXIMUM_ANGLE_OF_SIMILARITY = Math.toRadians(10.0);

   public OctreeICPSLAMFrame(StereoVisionPointCloudMessage message)
   {
      super(message);
   }

   public OctreeICPSLAMFrame(SLAMFrame frame, StereoVisionPointCloudMessage message)
   {
      super(frame, message);
   }

   public NormalOcTree getOctreeNodesInPreviousView()
   {
      return octreeNodesInPreviousView;
   }

   public List<IhmcSurfaceElement> getMergeableSurfaceElements()
   {
      return mergeableSurfaceElements;
   }

   public void computeOctreeInPreviousView(double octreeResolution)
   {
      if (isFirstFrame())
         octreeNodesInPreviousView = null;

      double[][] vertex = new double[pointCloudToSensorFrame.length][2];

      for (int i = 0; i < pointCloudToSensorFrame.length; i++)
      {
         vertex[i][0] = pointCloudToSensorFrame[i].getX();
         vertex[i][1] = pointCloudToSensorFrame[i].getY();
      }
      Vertex2DSupplier supplier = Vertex2DSupplier.asVertex2DSupplier(vertex);
      ConvexPolygon2D window = new ConvexPolygon2D(supplier);

      boolean ignorePreviousOrientation = true;
      RigidBodyTransformReadOnly previousSensorPoseToWorld;
      if (ignorePreviousOrientation)
      {
         previousSensorPoseToWorld = previousFrame.getSensorPose();
         //previousSensorPoseToWorld.setRotation(optimizedSensorPoseToWorld.getRotation());
      }
      else
      {
         previousSensorPoseToWorld = previousFrame.getSensorPose();
      }

      Point3D[] pointCloudToWorld = new Point3D[pointCloudToSensorFrame.length];
      for (int i = 0; i < pointCloudToWorld.length; i++)
      {
         pointCloudToWorld[i] = new Point3D(pointCloudToSensorFrame[i]);
         sensorPoseToWorld.transform(pointCloudToWorld[i]);
      }
      Point3D[] convertedPointsToPreviousSensorPose = IhmcSLAMTools.createConvertedPointsToSensorPose(previousSensorPoseToWorld, pointCloudToWorld);
      boolean[] isInPreviousView = new boolean[convertedPointsToPreviousSensorPose.length];
      int numberOfPointsInPreviousView = 0;
      for (int i = 0; i < convertedPointsToPreviousSensorPose.length; i++)
      {
         Point3D point = convertedPointsToPreviousSensorPose[i];
         isInPreviousView[i] = false;
         if (window.isPointInside(point.getX(), point.getY()))
         {
            isInPreviousView[i] = true;
            numberOfPointsInPreviousView++;
         }
      }

      Point3D[] pointsInPreviousView = new Point3D[numberOfPointsInPreviousView];
      int index = 0;
      for (int i = 0; i < pointCloudToSensorFrame.length; i++)
      {
         if (isInPreviousView[i])
         {
            pointsInPreviousView[index] = new Point3D(pointCloudToWorld[i]);
            index++;
         }
      }

      octreeNodesInPreviousView = IhmcSLAMTools.computeOctreeData(pointsInPreviousView, previousSensorPoseToWorld.getTranslation(), octreeResolution);
   }

   public void computeMergeableSurfaceElements(PlanarRegionsList planarRegionsMap, double octreeResolution, double validRatio, double maximumDistance,
                                               double maximumAngle)
   {
      //TDOO: check.
      boolean useSufficientScoreDecision = true;
      double allowableDistanceIn2D = 0.05;

      NormalOcTree octree = octreeNodesInPreviousView;
      if (octree == null)
         return;

      mergeableSurfaceElements.clear();

      int numberOfPlanarRegions = planarRegionsMap.getNumberOfPlanarRegions();
      NormalOcTreeMessage normalOctreeMessage = OcTreeMessageConverter.convertToMessage(octree);
      UIOcTree octreeForViz = new UIOcTree(normalOctreeMessage);
      int numberOfNodes = 0;

      for (UIOcTreeNode uiOcTreeNode : octreeForViz)
      {
         if (!uiOcTreeNode.isNormalSet() || !uiOcTreeNode.isHitLocationSet())
            continue;

         numberOfNodes++;
         Vector3D planeNormal = new Vector3D();
         Point3D pointOnPlane = new Point3D();

         uiOcTreeNode.getNormal(planeNormal);
         uiOcTreeNode.getHitLocation(pointOnPlane);
         Plane3D octreePlane = new Plane3D(pointOnPlane, planeNormal);

         int indexBestPlanarRegion = -1;
         double minimumScore = Double.MAX_VALUE;
         double minimumPositionScore = Double.MAX_VALUE;
         double minimumAngleScore = Double.MAX_VALUE;
         for (int j = 0; j < numberOfPlanarRegions; j++)
         {
            PlanarRegion planarRegion = planarRegionsMap.getPlanarRegion(j);
            Plane3D plane = planarRegion.getPlane();
            double positionDistance = plane.distance(octreePlane.getPoint());
            double angleDistance = Math.abs(Math.acos(Math.abs(planarRegion.getPlane().getNormal().dot(octreePlane.getNormal()))));
            double score = positionDistance / maximumDistance + angleDistance / maximumAngle;
            if (score < minimumScore)
            {
               minimumScore = score;
               minimumPositionScore = positionDistance;
               minimumAngleScore = angleDistance;
               indexBestPlanarRegion = j;
            }
         }

         PlanarRegion closestPlanarRegion = planarRegionsMap.getPlanarRegion(indexBestPlanarRegion);
         ConvexPolygon2D convexHull = closestPlanarRegion.getConvexHull();

         RigidBodyTransformReadOnly transformToLocal = closestPlanarRegion.getTransformToLocal();
         Point3D localPoint = new Point3D();
         transformToLocal.transform(octreePlane.getPointCopy(), localPoint);
         double distanceToPlanarRegionIn2D = convexHull.distance(new Point2D(localPoint.getX(), localPoint.getY()));
         boolean isAllowable = distanceToPlanarRegionIn2D < allowableDistanceIn2D;

         double sufficientRatio = 1.0;
         if (useSufficientScoreDecision)
            sufficientRatio = 0.5;
         if (isAllowable && minimumScore < 1.0 && minimumPositionScore < sufficientRatio && minimumAngleScore < sufficientRatio)
         {
            IhmcSurfaceElement surfaceElement = new IhmcSurfaceElement(octreeResolution);
            surfaceElement.setPlane(octreePlane);
            surfaceElement.setMergeablePlanarRegion(closestPlanarRegion);
            mergeableSurfaceElements.add(surfaceElement);
         }
      }

      double ratio = (double) mergeableSurfaceElements.size() / (numberOfNodes + 1);

      if (ratio < validRatio || mergeableSurfaceElements.size() == 0)
         setMergeableFrame(false);
      else
         setMergeableFrame(true);
   }

   @Override
   public SLAMFrameOptimizerCostFunction createCostFunction(AbstractSLAM slam)
   {
      computeOctreeInPreviousView(slam.getOctreeResolution());
      computeMergeableSurfaceElements(slam.getPlanarRegionsMap(), slam.getOctreeResolution(), VALID_PLANES_RATIO_THRESHOLD, MAXIMUM_DISTANCE_OF_SIMILARITY,
                                      MAXIMUM_ANGLE_OF_SIMILARITY);
      List<IhmcSurfaceElement> surfaceElements = getMergeableSurfaceElements();

      OctreeICPSLAMFrameOptimizerCostFunction function = new OctreeICPSLAMFrameOptimizerCostFunction(surfaceElements, getInitialSensorPoseToWorld());

      return function;
   }
}
