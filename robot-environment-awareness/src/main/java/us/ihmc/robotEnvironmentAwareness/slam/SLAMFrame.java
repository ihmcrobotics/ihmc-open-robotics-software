package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.robotEnvironmentAwareness.communication.converters.PointCloudCompression;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;

public class SLAMFrame
{
   private final Long timestamp;
   private final SLAMFrame previousFrame;

   /**
    * original pose from message.
    */
   private final RigidBodyTransformBasics uncorrectedLocalPoseInWorld;

   /**
    * SLAM result.
    */
   private final RigidBodyTransform driftCompensationTransform = new RigidBodyTransform();

   /**
    * this.sensorPoseToWorld * this.slamTransformer.
    */
   private final RigidBodyTransform correctedLocalPoseInWorld = new RigidBodyTransform();

   private final Point3DReadOnly[] uncorrectedPointCloudInWorld; // For comparison after mapping.
   private final Point3DReadOnly[] pointCloudInLocalFrame;
   private final Point3D[] correctedPointCloudInWorld;

   private double confidenceFactor = 1.0;

   private final List<Plane3D> surfaceElements = new ArrayList<>();
   private final List<Plane3DReadOnly> surfaceElementsInLocalFrame = new ArrayList<>();
   private NormalOcTree frameMap;

   public SLAMFrame(StereoVisionPointCloudMessage message)
   {
      this(null, new RigidBodyTransform(), message);
   }

   public SLAMFrame(RigidBodyTransformReadOnly sensorToLocalTransform, StereoVisionPointCloudMessage message)
   {
      this(null, sensorToLocalTransform, message);
   }

   public SLAMFrame(SLAMFrame frame, StereoVisionPointCloudMessage message)
   {
      this(frame, new RigidBodyTransform(), message);
   }

   public SLAMFrame(SLAMFrame frame, RigidBodyTransformReadOnly sensorToLocalTransform, StereoVisionPointCloudMessage message)
   {
      timestamp = message.getTimestamp();
      previousFrame = frame;

      RigidBodyTransformReadOnly uncorrectedSensorPoseInWorld = MessageTools.unpackSensorPose(message);
      uncorrectedLocalPoseInWorld = new RigidBodyTransform();
      sensorToLocalTransform.transform(uncorrectedSensorPoseInWorld, uncorrectedLocalPoseInWorld);

      correctedLocalPoseInWorld.set(uncorrectedSensorPoseInWorld);

      uncorrectedPointCloudInWorld = PointCloudCompression.decompressPointCloudToArray(message);
      pointCloudInLocalFrame = SLAMTools.createConvertedPointsToSensorPose(uncorrectedLocalPoseInWorld, uncorrectedPointCloudInWorld);
      correctedPointCloudInWorld = new Point3D[pointCloudInLocalFrame.length];
      for (int i = 0; i < correctedPointCloudInWorld.length; i++)
         correctedPointCloudInWorld[i] = new Point3D(pointCloudInLocalFrame[i]);

      updateOptimizedPointCloudAndSensorPose();
   }

   public void updateOptimizedCorrection(RigidBodyTransformReadOnly driftCorrectionTransform)
   {
      driftCompensationTransform.set(driftCorrectionTransform);
      updateOptimizedPointCloudAndSensorPose();
   }

   private void updateOptimizedPointCloudAndSensorPose()
   {
      correctedLocalPoseInWorld.set(uncorrectedLocalPoseInWorld);
      correctedLocalPoseInWorld.getRotation().normalize();
      correctedLocalPoseInWorld.multiply(driftCompensationTransform);

      for (int i = 0; i < correctedPointCloudInWorld.length; i++)
         correctedLocalPoseInWorld.transform(pointCloudInLocalFrame[i], correctedPointCloudInWorld[i]);

      for (int i = 0; i < surfaceElements.size(); i++)
      {
         Plane3D surfel = surfaceElements.get(i);
         surfel.set(surfaceElementsInLocalFrame.get(i));
         getCorrectedLocalPoseInWorld().transform(surfel.getPoint());
         getCorrectedLocalPoseInWorld().transform(surfel.getNormal());
      }
   }

   public void registerSurfaceElements(NormalOcTree map, double windowMargin, double surfaceElementResolution, int minimumNumberOfHits, boolean updateNormal)
   {
      surfaceElements.clear();
      surfaceElementsInLocalFrame.clear();
      frameMap = new NormalOcTree(surfaceElementResolution);

      ConvexPolygon2D mapHullInWorld = new ConvexPolygon2D();
      for (NormalOcTreeNode node : OcTreeIteratorFactory.createLeafBoundingBoxIteratable(map.getRoot(), map.getBoundingBox()))
         mapHullInWorld.addVertex(node.getHitLocationX(), node.getHitLocationY());
      mapHullInWorld.update();

      frameMap.insertScan(SLAMTools.toScan(getUncorrectedPointCloudInWorld(), getUncorrectedLocalPoseInWorld().getTranslation(), mapHullInWorld, windowMargin), false);
      frameMap.enableParallelComputationForNormals(true);

      NormalEstimationParameters normalEstimationParameters = new NormalEstimationParameters();
      normalEstimationParameters.setNumberOfIterations(10);
      frameMap.setNormalEstimationParameters(normalEstimationParameters);
      if (updateNormal)
         frameMap.updateNormals();

      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createIterable(frameMap.getRoot());
      for (NormalOcTreeNode node : iterable)
      {
         if (node.getNumberOfHits() >= minimumNumberOfHits)
         {
            if (!updateNormal || node.getNormalAverageDeviation() < 0.00005)
            {
               Plane3D surfaceElement = new Plane3D();
               node.getNormal(surfaceElement.getNormal());
               node.getHitLocation(surfaceElement.getPoint());

               surfaceElements.add(surfaceElement);
               surfaceElementsInLocalFrame.add(SLAMTools.createConvertedSurfaceElementToSensorPose(getUncorrectedLocalPoseInWorld(), surfaceElement));
            }
         }
      }
   }

   public NormalOcTree getFrameMap()
   {
      return frameMap;
   }

   public int getNumberOfSurfaceElements()
   {
      return surfaceElements.size();
   }

   public List<Plane3D> getSurfaceElements()
   {
      return surfaceElements;
   }

   public List<Plane3DReadOnly> getSurfaceElementsInLocalFrame()
   {
      return surfaceElementsInLocalFrame;
   }

   public void setConfidenceFactor(double value)
   {
      confidenceFactor = value;
   }

   public Point3DReadOnly[] getUncorrectedPointCloudInWorld()
   {
      return uncorrectedPointCloudInWorld;
   }

   public Point3DReadOnly[] getPointCloudInLocalFrame()
   {
      return pointCloudInLocalFrame;
   }

   public RigidBodyTransformReadOnly getUncorrectedLocalPoseInWorld()
   {
      return uncorrectedLocalPoseInWorld;
   }

   public Point3DReadOnly[] getCorrectedPointCloudInWorld()
   {
      return correctedPointCloudInWorld;
   }

   public RigidBodyTransformReadOnly getCorrectedLocalPoseInWorld()
   {
      return correctedLocalPoseInWorld;
   }

   public boolean isFirstFrame()
   {
      if (previousFrame == null)
         return true;
      else
         return false;
   }

   public Long getTimeStamp()
   {
      return timestamp;
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
