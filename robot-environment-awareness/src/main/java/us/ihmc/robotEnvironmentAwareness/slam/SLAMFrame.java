package us.ihmc.robotEnvironmentAwareness.slam;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.Stream;
import java.util.stream.StreamSupport;

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
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
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
   /**
    * Whether or not to perform calculations using parallel streams. Set at construction.
    */
   private final boolean computeInParallel;

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
   private final List<Point3DReadOnly> pointCloudInLocalFrame;
   /**
    * Corrected point cloud expressed in the world frame. These are found by by applying {@link #driftCompensationTransform}, and the algorithm is said to have
    * converged when these correspond to the global map properly.
    */
   private List<Point3DBasics> correctedPointCloudInWorld;

   private double confidenceFactor = 1.0;

   /**
    * Surface elements expressed in the world frame, which are the leaves of the {@link #frameMap} OcTree that correspond to points in the global map (they are
    * the features being used for SLAM). They are updated with {@link #driftCompensationTransform} each time
    * {@link SLAMFrame#updateOptimizedPointCloudAndSensorPose()} is called.
    */
   private List<Plane3D> surfaceElements = new ArrayList<>();
   /**
    * Surface elements expressed in the local frame. They are absolute, and unaffected by drift correction, as they are local values.
    */
   private List<Plane3DReadOnly> surfaceElementsInLocalFrame = new ArrayList<>();
   private final NormalEstimationParameters normalEstimationParameters;

   private NormalOcTree frameMap;

   private boolean isCorrectedPointCloudUpToDate = false;
   private boolean areSurfaceElementsUpToDate = false;

   public SLAMFrame(StereoVisionPointCloudMessage message, NormalEstimationParameters normalEstimationParameters)
   {
      this(message, normalEstimationParameters, false);
   }

   public SLAMFrame(StereoVisionPointCloudMessage message, NormalEstimationParameters normalEstimationParameters, boolean computeInParallel)
   {
      this(null, new RigidBodyTransform(), message, normalEstimationParameters, computeInParallel);
   }

   public SLAMFrame(RigidBodyTransformReadOnly localToSensorTransform,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters)
   {
      this(null, localToSensorTransform, message, normalEstimationParameters, false);
   }

   public SLAMFrame(RigidBodyTransformReadOnly localToSensorTransform,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters,
                    boolean computeInParallel)
   {
      this(null, localToSensorTransform, message, normalEstimationParameters, computeInParallel);
   }

   public SLAMFrame(SLAMFrame previousFrame,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters)
   {
      this(previousFrame, new RigidBodyTransform(), message, normalEstimationParameters, false);
   }

   public SLAMFrame(SLAMFrame previousFrame,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters,
                    boolean computeInParallel)
   {
      this(previousFrame, new RigidBodyTransform(), message, normalEstimationParameters, computeInParallel);
   }

   public SLAMFrame(SLAMFrame previousFrame,
                    RigidBodyTransformReadOnly localToSensorTransform,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters)
   {
      this(previousFrame, localToSensorTransform, message, normalEstimationParameters, false);
   }

   public SLAMFrame(SLAMFrame previousFrame,
                    RigidBodyTransformReadOnly localToSensorTransform,
                    StereoVisionPointCloudMessage message,
                    NormalEstimationParameters normalEstimationParameters,
                    boolean computeInParallel)
   {
      timestamp = message.getTimestamp();
      this.previousFrame = previousFrame;
      this.computeInParallel = computeInParallel;

      this.normalEstimationParameters = normalEstimationParameters;

      uncorrectedSensorPoseInWorld = MessageTools.unpackSensorPose(message);
      uncorrectedLocalPoseInWorld = new RigidBodyTransform();
      uncorrectedLocalPoseInWorld.set(uncorrectedSensorPoseInWorld);
      uncorrectedLocalPoseInWorld.multiplyInvertOther(localToSensorTransform);

      correctedLocalPoseInWorld.set(uncorrectedLocalPoseInWorld);
      correctedSensorPoseInWorld.set(uncorrectedSensorPoseInWorld);

      uncorrectedPointCloudInWorld = PointCloudCompression.decompressPointCloudToArray(message);
      pointCloudInLocalFrame = SLAMTools.createConvertedPointsToSensorPose(uncorrectedLocalPoseInWorld, uncorrectedPointCloudInWorld,
                                                                           computeInParallel);

      updateOptimizedPointCloudAndSensorPose();
   }

   public void updateOptimizedCorrection(RigidBodyTransformReadOnly driftCorrectionTransform)
   {
      isCorrectedPointCloudUpToDate = false;
      areSurfaceElementsUpToDate = false;

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
   }

   private void updateCorrectedPointCloudInWorld()
   {
      Stream<Point3DReadOnly> pointCloudStream = computeInParallel ? pointCloudInLocalFrame.parallelStream() : pointCloudInLocalFrame.stream();
      correctedPointCloudInWorld = pointCloudStream.map(pointInLocal ->
                                                        {
                                                           Point3D pointInWorld = new Point3D();
                                                           correctedLocalPoseInWorld.transform(pointInLocal, pointInWorld);
                                                           return pointInWorld;
                                                        }).collect(Collectors.toList());
      isCorrectedPointCloudUpToDate = true;
   }

   private void updateSurfaceElements()
   {
      Stream<Plane3DReadOnly> surfaceElementInLocalStream = computeInParallel ? surfaceElementsInLocalFrame.parallelStream() : surfaceElementsInLocalFrame.stream();
      surfaceElements = surfaceElementInLocalStream.map(surfaceElementInLocal ->
                                                        {
                                                           Plane3D surfel = new Plane3D();
                                                           surfel.set(surfaceElementInLocal);
                                                           getCorrectedLocalPoseInWorld().transform(surfel.getPoint());
                                                           getCorrectedLocalPoseInWorld().transform(surfel.getNormal());
                                                           return surfel;
                                                        }).collect(Collectors.toList());
      areSurfaceElementsUpToDate = true;
   }

   public void registerSurfaceElements(NormalOcTree map,
                                       double windowMargin,
                                       double surfaceElementResolution,
                                       int minimumNumberOfHits,
                                       boolean updateNormal,
                                       int maxNumberOfSurfaceElements)
   {
      frameMap = new NormalOcTree(surfaceElementResolution);

      ConvexPolygon2D mapHullInWorld = new ConvexPolygon2D();
      for (NormalOcTreeNode node : OcTreeIteratorFactory.createLeafBoundingBoxIteratable(map.getRoot(), map.getBoundingBox()))
         mapHullInWorld.addVertex(node.getHitLocationX(), node.getHitLocationY());
      mapHullInWorld.update();

      frameMap.insertScan(SLAMTools.toScan(getUncorrectedPointCloudInWorld(),
                                           getUncorrectedLocalPoseInWorld().getTranslation(),
                                           mapHullInWorld,
                                           windowMargin, computeInParallel), false);
      frameMap.enableParallelComputationForNormals(true);

      frameMap.setNormalEstimationParameters(normalEstimationParameters);
      if (updateNormal)
         frameMap.updateNormals();

      OcTreeIterable<NormalOcTreeNode> iterable = OcTreeIteratorFactory.createLeafIterable(frameMap.getRoot());
      Stream<NormalOcTreeNode> nodeStream = StreamSupport.stream(iterable.spliterator(), computeInParallel);
      surfaceElements = nodeStream.filter(node -> isValidNode(node, minimumNumberOfHits, 0.00005, updateNormal)).map(node ->
                                                                                                                     {
                                                                                                                        Plane3D surfaceElement = new Plane3D();
                                                                                                                        node.getNormal(surfaceElement.getNormal());
                                                                                                                        node.getHitLocation(surfaceElement.getPoint());
                                                                                                                        return surfaceElement;
                                                                                                                     }).collect(Collectors.toList());

      randomlySampleSurfaceElements(surfaceElements, maxNumberOfSurfaceElements);

      Stream<Plane3D> surfaceElementStream = computeInParallel ? surfaceElements.parallelStream() : surfaceElements.stream();
      surfaceElementsInLocalFrame = surfaceElementStream.map(surfaceElement -> SLAMTools.createConvertedSurfaceElementToSensorPose(
            getUncorrectedLocalPoseInWorld(),
            surfaceElement)).collect(Collectors.toList());
   }

   private static boolean isValidNode(NormalOcTreeNode node, int minimumNumberOfHits, double maximumAverageDeviation, boolean updateNormal)
   {
      return node.getNumberOfHits() >= minimumNumberOfHits && (!updateNormal || node.getNormalAverageDeviation() < maximumAverageDeviation);
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
      return surfaceElementsInLocalFrame.size();
   }

   public List<Plane3D> getSurfaceElements()
   {
      if (!areSurfaceElementsUpToDate)
         updateSurfaceElements();

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

   public List<Point3DReadOnly> getPointCloudInLocalFrame()
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

   public List<? extends Point3DReadOnly> getCorrectedPointCloudInWorld()
   {
      if (!isCorrectedPointCloudUpToDate)
         updateCorrectedPointCloudInWorld();

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
