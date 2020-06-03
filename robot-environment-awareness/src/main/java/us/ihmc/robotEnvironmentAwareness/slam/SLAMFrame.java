package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;

public class SLAMFrame
{
   private final SLAMFrame previousFrame;

   /**
    * original sensor pose from message.
    */
   private final RigidBodyTransformReadOnly originalSensorPoseToWorld;

   /**
    * fixedDiff(parent.originalSensorPoseToWorld vs this.originalSensorPoseToWorld).
    */
   private final RigidBodyTransformReadOnly transformFromPreviousFrame;

   /**
    * parent.optimizedSensorPoseToWorld * transformFromPreviousFrame.
    */
   private final RigidBodyTransformReadOnly sensorPoseToWorld;

   /**
    * SLAM result.
    */
   private final RigidBodyTransform slamTransformer = new RigidBodyTransform();

   /**
    * this.sensorPoseToWorld * this.slamTransformer.
    */
   private final RigidBodyTransform optimizedSensorPoseToWorld = new RigidBodyTransform();

   private final Point3DReadOnly[] originalPointCloudToWorld; // For comparison after mapping.
   private final Point3DReadOnly[] pointCloudToSensorFrame;
   private final Point3D[] optimizedPointCloudToWorld;
   
   private double confidenceFactor;

   public SLAMFrame(StereoVisionPointCloudMessage message)
   {
      previousFrame = null;

      originalSensorPoseToWorld = MessageTools.unpackSensorPose(message);

      transformFromPreviousFrame = new RigidBodyTransform(originalSensorPoseToWorld);
      sensorPoseToWorld = new RigidBodyTransform(originalSensorPoseToWorld);
      optimizedSensorPoseToWorld.set(originalSensorPoseToWorld);

      originalPointCloudToWorld = PointCloudCompression.decompressPointCloudToArray(message);
      pointCloudToSensorFrame = SLAMTools.createConvertedPointsToSensorPose(originalSensorPoseToWorld, originalPointCloudToWorld);
      optimizedPointCloudToWorld = new Point3D[pointCloudToSensorFrame.length];
      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
         optimizedPointCloudToWorld[i] = new Point3D(pointCloudToSensorFrame[i]);

      updateOptimizedPointCloudAndSensorPose();
   }
   
//   public SLAMFrame(SLAMFrame frame, StereoVisionPointCloudMessage message)
//   {
//      previousFrame = frame;
//
//      originalSensorPoseToWorld = MessageTools.unpackSensorPose(message);
//
//      RigidBodyTransform transformDiff = new RigidBodyTransform(originalSensorPoseToWorld);
//      transformDiff.preMultiplyInvertOther(frame.originalSensorPoseToWorld);
//      transformFromPreviousFrame = new RigidBodyTransform(transformDiff);
//
//      RigidBodyTransform transformToWorld = new RigidBodyTransform(frame.optimizedSensorPoseToWorld);
//      transformToWorld.multiply(transformFromPreviousFrame);
//      sensorPoseToWorld = new RigidBodyTransform(transformToWorld);
//
//      originalPointCloudToWorld = PointCloudCompression.decompressPointCloudToArray(message);
//      pointCloudToSensorFrame = SLAMTools.createConvertedPointsToSensorPose(originalSensorPoseToWorld, originalPointCloudToWorld);
//      optimizedPointCloudToWorld = new Point3D[pointCloudToSensorFrame.length];
//      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
//         optimizedPointCloudToWorld[i] = new Point3D(pointCloudToSensorFrame[i]);
//
//      updateOptimizedPointCloudAndSensorPose();
//   }

   public SLAMFrame(SLAMFrame frame, StereoVisionPointCloudMessage message)
   {
      previousFrame = frame;

      originalSensorPoseToWorld = MessageTools.unpackSensorPose(message);

      transformFromPreviousFrame = new RigidBodyTransform();
      sensorPoseToWorld = new RigidBodyTransform(originalSensorPoseToWorld);

      originalPointCloudToWorld = PointCloudCompression.decompressPointCloudToArray(message);
      pointCloudToSensorFrame = SLAMTools.createConvertedPointsToSensorPose(originalSensorPoseToWorld, originalPointCloudToWorld);
      optimizedPointCloudToWorld = new Point3D[pointCloudToSensorFrame.length];
      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
         optimizedPointCloudToWorld[i] = new Point3D(pointCloudToSensorFrame[i]);

      updateOptimizedPointCloudAndSensorPose();
   }

   public void updateOptimizedCorrection(RigidBodyTransformReadOnly driftCorrectionTransform)
   {
      slamTransformer.set(driftCorrectionTransform);
      updateOptimizedPointCloudAndSensorPose();
   }

   private void updateOptimizedPointCloudAndSensorPose()
   {
      optimizedSensorPoseToWorld.set(sensorPoseToWorld);
      optimizedSensorPoseToWorld.getRotation().normalize();
      optimizedSensorPoseToWorld.multiply(slamTransformer);

      for (int i = 0; i < optimizedPointCloudToWorld.length; i++)
      {
         optimizedPointCloudToWorld[i].set(pointCloudToSensorFrame[i]);
         optimizedSensorPoseToWorld.transform(optimizedPointCloudToWorld[i]);
      }
      
      for (int i = 0; i < surfaceElements.size(); i++)
      {
         Plane3D surfel = surfaceElements.get(i);
         surfel.set(surfaceElementsToSensor.get(i));
         getSensorPose().transform(surfel.getPoint());
         getSensorPose().transform(surfel.getNormal());
      }
   }
   
   private final List<Plane3D> surfaceElements = new ArrayList<>();
   private final List<Plane3DReadOnly> surfaceElementsToSensor = new ArrayList<>();
   
   public void registerSurfaceElements(NormalOcTree map, double windowMargin, double surfaceElementResolution, int minimumNumberOfHits)
   {
      surfaceElements.clear();
      surfaceElementsToSensor.clear();
      NormalOcTree frameMap = new NormalOcTree(surfaceElementResolution);

      ScanCollection scanCollection = new ScanCollection();
      int numberOfPoints = getOriginalPointCloud().length;

      scanCollection.setSubSampleSize(numberOfPoints);
      scanCollection.addScan(SLAMTools.toScan(getOriginalPointCloud(), getOriginalPointCloudToSensorPose(), getOriginalSensorPose(), map, windowMargin));

      frameMap.insertScanCollection(scanCollection, false);
      frameMap.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(10);
      normalEstimationParameters.setMaxDistanceFromPlane(surfaceElementResolution);
      frameMap.setNormalEstimationParameters(normalEstimationParameters);
      frameMap.updateNormals();

      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createIterable(frameMap.getRoot());
      for (NormalOcTreeNode node : iterable)
      {
         if (node.getNumberOfHits() >= minimumNumberOfHits)
         {
            if (node.getNormalAverageDeviation() < 0.00005)
            {
               Plane3D surfaceElement = new Plane3D();
               node.getNormal(surfaceElement.getNormal());
               node.getHitLocation(surfaceElement.getPoint());

               surfaceElements.add(surfaceElement);
               surfaceElementsToSensor.add(SLAMTools.createConvertedSurfaceElementToSensorPose(getOriginalSensorPose(), surfaceElement));
            }
         }
      }
   }
   

   public List<Plane3D> getSurfaceElements()
   {
      return surfaceElements;
   }

   public List<Plane3DReadOnly> getSurfaceElementsToSensor()
   {
      return surfaceElementsToSensor;
   }
   
   public void setConfidenceFactor(double value)
   {
      confidenceFactor = value;
   }

   public Point3DReadOnly[] getOriginalPointCloud()
   {
      return originalPointCloudToWorld;
   }

   public Point3DReadOnly[] getOriginalPointCloudToSensorPose()
   {
      return pointCloudToSensorFrame;
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

   public boolean isFirstFrame()
   {
      if (previousFrame == null)
         return true;
      else
         return false;
   }

   public SLAMFrame getPreviousFrame()
   {
      return previousFrame;
   }
   
   public double getConfidenceFactor()
   {
      return confidenceFactor;
   }
}
