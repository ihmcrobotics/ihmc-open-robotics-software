package us.ihmc.humanoidBehaviors.ui.mapping;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;

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

   private final Point3DReadOnly[] originalPointCloudToWorld;
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

   public Point3DReadOnly[] getPointCloud()
   {
      return optimizedPointCloudToWorld;
   }

   public RigidBodyTransformReadOnly getSensorPose()
   {
      return optimizedSensorPoseToWorld;
   }

   public NormalOcTree computeOctreeInPreviousView(double octreeResolution)
   {
      // TODO: redefine this X-Y window with 2Dconcave hull.
      double maxX = 0.0;
      double minX = Double.MAX_VALUE;
      double maxY = 0.0;
      double minY = Double.MAX_VALUE;

      for (int i = 0; i < pointCloudToSensorFrame.length; i++)
      {
         maxX = Math.max(maxX, pointCloudToSensorFrame[i].getX());
         maxY = Math.max(maxY, pointCloudToSensorFrame[i].getY());

         minX = Math.min(minX, pointCloudToSensorFrame[i].getX());
         minY = Math.min(minY, pointCloudToSensorFrame[i].getY());
      }

      RigidBodyTransformReadOnly previousSensorPoseToWorld = previousFrame.optimizedSensorPoseToWorld;

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
         if (minX < point.getX() && point.getX() < maxX)
         {
            if (minY < point.getY() && point.getY() < maxY)
            {
               isInPreviousView[i] = true;
               numberOfPointsInPreviousView++;
            }
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

      NormalOcTree octree = IhmcSLAMTools.computeOctreeData(pointsInPreviousView, previousSensorPoseToWorld.getTranslation(), octreeResolution);
      return octree;
   }
}
