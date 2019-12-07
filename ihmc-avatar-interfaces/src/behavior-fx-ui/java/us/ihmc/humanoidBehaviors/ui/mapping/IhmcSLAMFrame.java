package us.ihmc.humanoidBehaviors.ui.mapping;

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
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTree;
import us.ihmc.robotEnvironmentAwareness.ui.UIOcTreeNode;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class IhmcSLAMFrame
{
   private final IhmcSLAMFrame previousFrame;

   // from message.
   private final RigidBodyTransformReadOnly originalSensorPoseToWorld;

   // fixedDiff(parent.originalSensorPoseToWorld vs this.originalSensorPoseToWorld).
   private final RigidBodyTransformReadOnly transformFromPreviousFrame;

   // parent.optimizedSensorPoseToWorld * transformFromPreviousFrame.
   private final RigidBodyTransformReadOnly sensorPoseToWorld;

   // SLAM result.
   private final RigidBodyTransform slamTransformer = new RigidBodyTransform();

   // this.sensorPoseToWorld * this.slamTransformer.
   private final RigidBodyTransform optimizedSensorPoseToWorld = new RigidBodyTransform();

   private final Point3DReadOnly[] originalPointCloudToWorld; // For comparison after mapping.
   private final Point3DReadOnly[] pointCloudToSensorFrame;
   private final Point3D[] optimizedPointCloudToWorld;

   public IhmcSLAMFrame(StereoVisionPointCloudMessage message)
   {
      previousFrame = null;

      originalSensorPoseToWorld = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      transformFromPreviousFrame = new RigidBodyTransform(originalSensorPoseToWorld);
      sensorPoseToWorld = new RigidBodyTransform(originalSensorPoseToWorld);
      optimizedSensorPoseToWorld.set(originalSensorPoseToWorld);

      originalPointCloudToWorld = IhmcSLAMTools.extractPointsFromMessage(message);
      pointCloudToSensorFrame = IhmcSLAMTools.createConvertedPointsToSensorPose(originalSensorPoseToWorld, originalPointCloudToWorld);
      optimizedPointCloudToWorld = new Point3D[pointCloudToSensorFrame.length];
      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
         optimizedPointCloudToWorld[i] = new Point3D(pointCloudToSensorFrame[i]);

      update();
   }

   public IhmcSLAMFrame(IhmcSLAMFrame frame, StereoVisionPointCloudMessage message)
   {
      previousFrame = frame;

      originalSensorPoseToWorld = IhmcSLAMTools.extractSensorPoseFromMessage(message);

      RigidBodyTransform transformDiff = new RigidBodyTransform(originalSensorPoseToWorld);
      transformDiff.preMultiplyInvertOther(frame.originalSensorPoseToWorld);
      transformFromPreviousFrame = new RigidBodyTransform(transformDiff);

      RigidBodyTransform transformToWorld = new RigidBodyTransform(frame.optimizedSensorPoseToWorld);
      transformToWorld.multiply(transformFromPreviousFrame);
      sensorPoseToWorld = new RigidBodyTransform(transformToWorld);

      originalPointCloudToWorld = IhmcSLAMTools.extractPointsFromMessage(message);
      pointCloudToSensorFrame = IhmcSLAMTools.createConvertedPointsToSensorPose(originalSensorPoseToWorld, originalPointCloudToWorld);
      optimizedPointCloudToWorld = new Point3D[pointCloudToSensorFrame.length];
      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
         optimizedPointCloudToWorld[i] = new Point3D(pointCloudToSensorFrame[i]);

      update();
   }

   public void updateSLAM(RigidBodyTransform driftCorrectionTransform)
   {
      slamTransformer.set(driftCorrectionTransform);
      update();
   }

   private void update()
   {
      optimizedSensorPoseToWorld.set(sensorPoseToWorld);
      optimizedSensorPoseToWorld.multiply(slamTransformer);

      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
      {
         optimizedPointCloudToWorld[i].set(pointCloudToSensorFrame[i]);
         optimizedSensorPoseToWorld.transform(optimizedPointCloudToWorld[i]);
      }
   }

   public Point3DReadOnly[] getOriginalPointCloud()
   {
      return originalPointCloudToWorld;
   }

   public RigidBodyTransformReadOnly getOriginalSensorPose()
   {
      return originalSensorPoseToWorld;
   }

   public RigidBodyTransformReadOnly getInitialSensorPoseToWorld()
   {
      return sensorPoseToWorld;
   }

   public Point3DReadOnly[] getPointCloud()
   {
      return optimizedPointCloudToWorld;
   }

   public RigidBodyTransformReadOnly getSensorPose()
   {
      return optimizedSensorPoseToWorld;
   }

   private NormalOcTree octreeNodesInPreviousView;
   private List<IhmcSurfaceElement> mergeableSurfaceElements = new ArrayList<>();
   private boolean mergeable = false;

   public NormalOcTree getOctreeNodesInPreviousView()
   {
      return octreeNodesInPreviousView;
   }

   public List<IhmcSurfaceElement> getMergeableSurfaceElements()
   {
      return mergeableSurfaceElements;
   }

   public boolean isMergeableFrame()
   {
      return mergeable;
   }

   public void computeOctreeInPreviousView(double octreeResolution)
   {
      if (previousFrame == null)
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
      RigidBodyTransform previousSensorPoseToWorld;
      if (ignorePreviousOrientation)
      {
         previousSensorPoseToWorld = previousFrame.optimizedSensorPoseToWorld;
         previousSensorPoseToWorld.setRotation(optimizedSensorPoseToWorld.getRotation());
      }
      else
      {
         previousSensorPoseToWorld = previousFrame.optimizedSensorPoseToWorld;
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
         mergeable = false;
      else
         mergeable = true;
   }
}
