package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.HashSet;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.jOctoMap.boundingBox.OcTreeBoundingBoxWithCenterAndYaw;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.normalEstimation.NormalEstimationParameters;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.BoundingBoxMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.BoundingBoxParametersMessage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

public class REAOcTreeUpdater
{
   private final Messager reaMessager;
   private final NormalOcTree referenceOctree;
   private final REAOcTreeBuffer[] reaOcTreeBuffers;

   private final AtomicReference<Pose3D> latestLidarPoseReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> enableNormalEstimation;
   private final AtomicReference<Boolean> clearNormals;

   private final AtomicReference<Double> minRange;
   private final AtomicReference<Double> maxRange;

   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;

   private final AtomicReference<Boolean> useBoundingBox;
   private final AtomicReference<BoundingBoxParametersMessage> atomicBoundingBoxParameters;

   public REAOcTreeUpdater(NormalOcTree octree, REAOcTreeBuffer[] buffers, Messager reaMessager)
   {
      this.referenceOctree = octree;
      this.reaOcTreeBuffers = buffers;
      this.reaMessager = reaMessager;
      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, true);
      enableNormalEstimation = reaMessager.createInput(REAModuleAPI.NormalEstimationEnable, true);
      clearNormals = reaMessager.createInput(REAModuleAPI.NormalEstimationClear, false);
      minRange = reaMessager.createInput(REAModuleAPI.LidarMinRange, 0.2);
      maxRange = reaMessager.createInput(REAModuleAPI.LidarMaxRange, 5.0);
      useBoundingBox = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      atomicBoundingBoxParameters = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters,
                                                            BoundingBoxMessageConverter.createBoundingBoxParametersMessage(0.0f, -2.0f, -3.0f, 5.0f, 2.0f,
                                                                                                                           0.5f));
      normalEstimationParameters = reaMessager.createInput(REAModuleAPI.NormalEstimationParameters, new NormalEstimationParameters());

      reaMessager.registerTopicListener(REAModuleAPI.RequestEntireModuleState, messageContent -> sendCurrentState());

      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());
   }
   
   public void initializeReferenceOctree()
   {
      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, enable.get());
      reaMessager.submitMessage(REAModuleAPI.NormalEstimationEnable, enableNormalEstimation.get());
      reaMessager.submitMessage(REAModuleAPI.LidarMinRange, minRange.get());
      reaMessager.submitMessage(REAModuleAPI.LidarMaxRange, maxRange.get());
      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxEnable, useBoundingBox.get());

      reaMessager.submitMessage(REAModuleAPI.OcTreeBoundingBoxParameters, atomicBoundingBoxParameters.get());
      reaMessager.submitMessage(REAModuleAPI.NormalEstimationParameters, normalEstimationParameters.get());
   }

   public void loadConfiguration(FilePropertyHelper filePropertyHelper)
   {
      Boolean enableFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreeEnable.getName());
      if (enableFile != null)
         enable.set(enableFile);
      Boolean enableNormalEstimationFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.NormalEstimationEnable.getName());
      if (enableNormalEstimationFile != null)
         enableNormalEstimation.set(enableNormalEstimationFile);
      String normalEstimationParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.NormalEstimationParameters.getName());
      if (normalEstimationParametersFile != null)
         normalEstimationParameters.set(NormalEstimationParameters.parse(normalEstimationParametersFile));
      Boolean useBoundingBoxFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreeBoundingBoxEnable.getName());
      if (useBoundingBoxFile != null)
         useBoundingBox.set(useBoundingBoxFile);
      String boundingBoxParametersFile = filePropertyHelper.loadProperty(REAModuleAPI.OcTreeBoundingBoxParameters.getName());
      if (boundingBoxParametersFile != null)
         atomicBoundingBoxParameters.set(BoundingBoxMessageConverter.parse(boundingBoxParametersFile));
      Double minRangeFile = filePropertyHelper.loadDoubleProperty(REAModuleAPI.LidarMinRange.getName());
      if (minRangeFile != null)
         minRange.set(minRangeFile);
      Double maxRangeFile = filePropertyHelper.loadDoubleProperty(REAModuleAPI.LidarMaxRange.getName());
      if (maxRangeFile != null)
         maxRange.set(maxRangeFile);
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreeEnable.getName(), enable.get());
      filePropertyHelper.saveProperty(REAModuleAPI.NormalEstimationEnable.getName(), enableNormalEstimation.get());
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreeBoundingBoxEnable.getName(), useBoundingBox.get());

      filePropertyHelper.saveProperty(REAModuleAPI.NormalEstimationParameters.getName(), normalEstimationParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreeBoundingBoxParameters.getName(), atomicBoundingBoxParameters.get().toString());
      filePropertyHelper.saveProperty(REAModuleAPI.LidarMinRange.getName(), minRange.get());
      filePropertyHelper.saveProperty(REAModuleAPI.LidarMaxRange.getName(), maxRange.get());
   }

   public void update()
   {
      if (!enable.get())
         return;

      handleBoundingBox();

      if (minRange.get() != null && maxRange.get() != null)
      {
         referenceOctree.setBoundsInsertRange(minRange.get(), maxRange.get());
      }

      referenceOctree.setNormalEstimationParameters(normalEstimationParameters.get());

      if (latestLidarPoseReference.get() == null)
         return;

      Point3DReadOnly sensorOrigin = latestLidarPoseReference.get().getPosition();
      boolean hasOcTreeBeenUpdated = false;

      for (REAOcTreeBuffer buffer : reaOcTreeBuffers)
      {
         boolean isBufferFull = buffer.isBufferFull();
         if (isBufferFull)
            buffer.submitBufferRequest();

         NormalOcTree bufferOctree = buffer.pollNewBuffer();

         if (bufferOctree != null)
         {
            PointCloud pointCloud = new PointCloud();
            bufferOctree.forEach(node -> pointCloud.add(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ()));
            Scan scan = new Scan(sensorOrigin, pointCloud);
            Set<NormalOcTreeNode> updatedNodes = new HashSet<>();
            referenceOctree.insertScan(scan, updatedNodes, null);
            hasOcTreeBeenUpdated = true;
         }
      }

      if (clearNormals.getAndSet(false))
      {
         referenceOctree.clearNormals();
         return;
      }

      if (!hasOcTreeBeenUpdated || !enableNormalEstimation.get())
         return;

      referenceOctree.updateNormals();
   }

   public void clearOcTree()
   {
      referenceOctree.clear();
   }

   private void handleBoundingBox()
   {
      if (!useBoundingBox.get())
      {
         referenceOctree.disableBoundingBox();
         return;
      }

      OcTreeBoundingBoxWithCenterAndYaw boundingBox = new OcTreeBoundingBoxWithCenterAndYaw();

      Point3D min = atomicBoundingBoxParameters.get().getMin();
      Point3D max = atomicBoundingBoxParameters.get().getMax();
      boundingBox.setLocalMinMaxCoordinates(min, max);

      if (latestLidarPoseReference.get() != null)
      {
         Pose3D lidarPose = latestLidarPoseReference.get();
         boundingBox.setOffset(lidarPose.getPosition());
         boundingBox.setYawFromQuaternion(new Quaternion(lidarPose.getOrientation()));
      }

      boundingBox.update(referenceOctree.getResolution(), referenceOctree.getTreeDepth());
      referenceOctree.setBoundingBox(boundingBox);
   }

   public void handleLidarScanMessage(LidarScanMessage message)
   {
      latestLidarPoseReference.set(new Pose3D(message.getLidarPosition(), message.getLidarOrientation()));
   }

   public void handleStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestLidarPoseReference.set(new Pose3D(message.getSensorPosition(), message.getSensorOrientation()));
   }
}
