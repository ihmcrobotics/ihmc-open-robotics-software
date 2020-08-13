package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.RandomNumbers;
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
    * original sensor pose expressed in the world frame from the message.
    */
   private final RigidBodyTransformReadOnly uncorrectedSensorPoseInWorld;
   /**
    * original pose to optimize about expressed in the world frame. This is pose is calculated from the {@link #correctedSensorPoseInWorld} by transforming it
    * using the transform provided on construction.
    */
   private final RigidBodyTransformBasics uncorrectedLocalPoseInWorld;

   /**
    * The result of the SLAM algorithm. This is the transform that maps the {@link #uncorrectedLocalPoseInWorld} to the {@link #correctedLocalPoseInWorld},
    * which should account for any state estimation drift an error.
    */
   private final RigidBodyTransform driftCompensationTransform = new RigidBodyTransform();

   /**
    * The sensor pose expressed in world frame from {@link #uncorrectedSensorPoseInWorld} after correcting the data for any drift to make it match the map. This
    * is found by {@link #uncorrectedSensorPoseInWorld} * {@link #driftCompensationTransform}.
    */
   private final RigidBodyTransform correctedSensorPoseInWorld = new RigidBodyTransform();
   /**
    * The local frame pose expressed in world frame from {@link #uncorrectedLocalPoseInWorld} after correcting the data for any drift to make it match the map.
    * This is found by {@link #uncorrectedLocalPoseInWorld} * {@link #driftCompensationTransform}. This transform is ultimately the goal of the optimization.
    */
   private final RigidBodyTransform correctedLocalPoseInWorld = new RigidBodyTransform();

   /**
    * This is the raw point cloud expressed in world frame received on construction. These points are likely to have drifted.
    */
   private final Point3DReadOnly[] uncorrectedPointCloudInWorld; // For comparison after mapping.
   /**
    * Point cloud expressed in local frame. These points are absolute, and unaffected by the drift compensation transform. The goal of the algorithm is to
    * modify the {@link #correctedLocalPoseInWorld} such that, when {@link #pointCloudInLocalFrame} is transformed to world, they best match up with the world
    * map.
    */
   private final Point3DReadOnly[] pointCloudInLocalFrame;
   /**
    * Corrected point cloud expressed in the world frame. These are found by by applying {@link #driftCompensationTransform}, and the algorithm is said to have
    * converged when these correspond to the global map properly.
    */
   private final Point3D[] correctedPointCloudInWorld;

   private double confidenceFactor = 1.0;

   /**
    * Surface elements expressed in the world frame, which are the leaves of the {@link #frameMap} OcTree that correspond to points in the global map (they are
    * the features being used for SLAM). They are updated with {@link #driftCompensationTransform} each time
    * {@link SLAMFrame#updateOptimizedPointCloudAndSensorPose()} is called.
    */
   private final List<Plane3D> surfaceElements = new ArrayList<>();
   /**
    * Surface elements expressed in the local frame. They are absolute, and unaffected by drift correction, as they are local values.
    */
   private final List<Plane3DReadOnly> surfaceElementsInLocalFrame = new ArrayList<>();
   private final NormalEstimationParameters normalEstimationParameters;

   private NormalOcTree frameMap;

   public SLAMFrame(StereoVisionPointCloudMessage message, NormalEstimationParameters normalEstimationParameters)
   {
      this(null, new RigidBodyTransform(), message, normalEstimationParameters);
   }

   public SLAMFrame(RigidBodyTransformReadOnly localToSensorTransform,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters)
   {
      this(null, localToSensorTransform, message, normalEstimationParameters);
   }

   public SLAMFrame(SLAMFrame frame,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters)
   {
      this(frame, new RigidBodyTransform(), message, normalEstimationParameters);
   }

   public SLAMFrame(SLAMFrame frame,
                    RigidBodyTransformReadOnly localToSensorTransform,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters)
   {
      timestamp = message.getTimestamp();
      previousFrame = frame;

      this.normalEstimationParameters = normalEstimationParameters;

      uncorrectedSensorPoseInWorld = MessageTools.unpackSensorPose(message);
      uncorrectedLocalPoseInWorld = new RigidBodyTransform();
      uncorrectedLocalPoseInWorld.set(uncorrectedSensorPoseInWorld);
      uncorrectedLocalPoseInWorld.multiplyInvertOther(localToSensorTransform);

      correctedLocalPoseInWorld.set(uncorrectedLocalPoseInWorld);
      correctedSensorPoseInWorld.set(uncorrectedSensorPoseInWorld);

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

      correctedSensorPoseInWorld.set(uncorrectedSensorPoseInWorld);
      correctedSensorPoseInWorld.getRotation().normalize();
      correctedSensorPoseInWorld.multiply(driftCompensationTransform);

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

   public void registerSurfaceElements(NormalOcTree map,
                                       double windowMargin,
                                       double surfaceElementResolution,
                                       int minimumNumberOfHits,
                                       boolean updateNormal,
                                       int maxNumberOfSurfaceElements)
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
            }
         }
      }

      randomlySampleSurfaceElements(surfaceElements, maxNumberOfSurfaceElements);

      surfaceElements.forEach(surfaceElement -> surfaceElementsInLocalFrame.add(SLAMTools.createConvertedSurfaceElementToSensorPose(
            getUncorrectedLocalPoseInWorld(),
            surfaceElement)));
   }

   private static void randomlySampleSurfaceElements(List<Plane3D> surfaceElementsToSample, int maxNumberOfSurfaceElements)
   {
      Random random = new Random();
      while (surfaceElementsToSample.size() > maxNumberOfSurfaceElements)
         surfaceElementsToSample.remove(RandomNumbers.nextInt(random, 0, surfaceElementsToSample.size() - 1));
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

   public RigidBodyTransformReadOnly getUncorrectedSensorPoseInWorld()
   {
      return uncorrectedSensorPoseInWorld;
   }

   public Point3DReadOnly[] getCorrectedPointCloudInWorld()
   {
      return correctedPointCloudInWorld;
   }

   public RigidBodyTransformReadOnly getCorrectedLocalPoseInWorld()
   {
      return correctedLocalPoseInWorld;
   }

   public RigidBodyTransformReadOnly getCorrectedSensorPoseInWorld()
   {
      return correctedSensorPoseInWorld;
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
