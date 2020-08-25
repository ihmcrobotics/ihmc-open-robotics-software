package us.ihmc.robotEnvironmentAwareness.slam;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.slam.tools.SLAMTools;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

public class SLAMBasics implements SLAMInterface
{
   private final AtomicReference<SLAMFrame> latestSlamFrame = new AtomicReference<>(null);
   protected Point3DBasics[] correctedCorrespondingPointLocation;
   protected final NormalOcTree mapOcTree;
   private final AtomicInteger mapSize = new AtomicInteger();

   private final AtomicDouble latestComputationTime = new AtomicDouble();

   protected final DriftCorrectionResult driftCorrectionResult = new DriftCorrectionResult();
   private final RigidBodyTransformReadOnly transformFromLocalToSensor;

   private final NormalEstimationParameters frameNormalEstimationParameters = new NormalEstimationParameters();

   private boolean computeInParallel = false;

   public SLAMBasics(double octreeResolution)
   {
      this(octreeResolution, new RigidBodyTransform());
   }

   public SLAMBasics(double octreeResolution, RigidBodyTransformReadOnly transformFromLocalFrameToSensor)
   {
      this.transformFromLocalToSensor = transformFromLocalFrameToSensor;
      mapOcTree = new NormalOcTree(octreeResolution);

      frameNormalEstimationParameters.setNumberOfIterations(10);
   }

   protected void insertNewPointCloud(SLAMFrame frame, boolean insertMiss)
   {
      List<? extends Point3DReadOnly> pointCloud = frame.getCorrectedPointCloudInWorld();
      RigidBodyTransformReadOnly sensorPose = frame.getCorrectedSensorPoseInWorld();

      Scan scan = SLAMTools.toScan(pointCloud, sensorPose.getTranslation());
      scan.getPointCloud().setTimestamp(frame.getTimeStamp());

      mapOcTree.insertScan(scan, insertMiss); // inserting the miss here is pretty dang expensive.
      mapOcTree.enableParallelComputationForNormals(true);
   }

   public void updateSurfaceNormals()
   {
      mapOcTree.updateNormals();
   }

   public void updateSurfaceNormalsInBoundingBox()
   {
      List<NormalOcTreeNode> leafNodesToUpdate = OcTreeIteratorFactory.createLeafBoundingBoxIteratable(mapOcTree.getRoot(), mapOcTree.getBoundingBox()).toList();
      mapOcTree.updateNodesNormals(leafNodesToUpdate);
   }

   public void setComputeInParallel(boolean computeInParallel)
   {
      this.computeInParallel = computeInParallel;
   }

   @Override
   public void addKeyFrame(StereoVisionPointCloudMessage pointCloudMessage, boolean insertMiss)
   {
      SLAMFrame frame = new SLAMFrame(transformFromLocalToSensor, pointCloudMessage, frameNormalEstimationParameters, computeInParallel);
      setLatestFrame(frame);
      insertNewPointCloud(frame, insertMiss);

      driftCorrectionResult.setDefault();
   }

   @Override
   public boolean addFrame(StereoVisionPointCloudMessage pointCloudMessage, boolean insertMiss)
   {
      SLAMFrame frame = new SLAMFrame(getLatestFrame(), transformFromLocalToSensor, pointCloudMessage, frameNormalEstimationParameters, computeInParallel);

      long startTime = System.nanoTime();
      RigidBodyTransformReadOnly driftCorrectionTransformer = computeFrameCorrectionTransformer(frame);
      latestComputationTime.set((double) Math.round(Conversions.nanosecondsToSeconds(System.nanoTime() - startTime) * 100) / 100);

      driftCorrectionResult.setComputationTime(latestComputationTime.get());
      if (driftCorrectionTransformer == null)
      {
         return false;
      }
      else
      {
         frame.updateOptimizedCorrection(driftCorrectionTransformer);
         setLatestFrame(frame);
         insertNewPointCloud(frame, insertMiss);
         return true;
      }
   }

   @Override
   public void clear()
   {
      latestSlamFrame.set(null);
      mapSize.set(0);
      mapOcTree.clear();
   }

   public Point3DReadOnly[] getSourcePoints()
   {
      return correctedCorrespondingPointLocation;
   }

   public boolean isEmpty()
   {
      if (mapSize.get() == 0)
         return true;
      else
         return false;
   }

   public int getMapSize()
   {
      return mapSize.get();
   }

   public void setLatestFrame(SLAMFrame frameToSet)
   {
      latestSlamFrame.set(frameToSet);
      mapSize.incrementAndGet();
   }

   public void handleBoundingBox(Pose3DReadOnly sensorPose, BoundingBoxParametersMessage boundingBoxParameters, boolean useBoundingBox)
   {
      handleBoundingBox(sensorPose.getPosition(), sensorPose.getOrientation(), boundingBoxParameters, useBoundingBox);
   }

   public void handleBoundingBox(Tuple3DReadOnly sensorPosition,
                                 Orientation3DReadOnly sensorOrientation,
                                 BoundingBoxParametersMessage boundingBoxParameters,
                                 boolean useBoundingBox)
   {
      if (!useBoundingBox)
      {
         mapOcTree.disableBoundingBox();
         return;
      }

      OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw();

      Point3D min = boundingBoxParameters.getMin();
      Point3D max = boundingBoxParameters.getMax();
      boundingBox.setLocalMinMaxCoordinates(min, max);

      if (sensorPosition != null && sensorOrientation != null)
      {
         boundingBox.setOffset(sensorPosition);
         boundingBox.setYawFromQuaternion(new Quaternion(sensorOrientation));
      }

      boundingBox.update(mapOcTree.getResolution(), mapOcTree.getTreeDepth());
      mapOcTree.setBoundingBox(boundingBox);
   }

   public SLAMFrame getLatestFrame()
   {
      return latestSlamFrame.get();
   }

   public double getOctreeResolution()
   {
      return mapOcTree.getResolution();
   }

   public NormalOcTree getMapOcTree()
   {
      return mapOcTree;
   }

   public void clearNormals()
   {
      mapOcTree.clearNormals();
   }

   public void setNormalEstimationParameters(NormalEstimationParameters normalEstimationParameters)
   {
      mapOcTree.setNormalEstimationParameters(normalEstimationParameters);
   }

   public void setFrameNormalEstimationParameters(NormalEstimationParameters normalEstimationParameters)
   {
      this.frameNormalEstimationParameters.set(normalEstimationParameters);
   }

   public double getComputationTimeForLatestFrame()
   {
      return latestComputationTime.get();
   }
   
   public DriftCorrectionResult getDriftCorrectionResult()
   {
      return driftCorrectionResult;
   }
}
