package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.*;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
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
   private NormalOcTree referenceOctree;
   private Pose3DBasics sensorPose = new Pose3D();
   private final REAOcTreeBuffer[] reaOcTreeBuffers;
   private final Map<REAOcTreeBuffer, AtomicReference<Pose3D>> sensorPoses;
   private final Map<REAOcTreeBuffer, AtomicBoolean> bufferClearRequest = new HashMap<>();

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> enableNormalEstimation;
   private final AtomicReference<Boolean> clearNormals;

   private final AtomicReference<Double> minRange;
   private final AtomicReference<Double> maxRange;

   private final AtomicReference<NormalEstimationParameters> normalEstimationParameters;

   private final AtomicReference<Boolean> useBoundingBox;
   private final AtomicReference<BoundingBoxParametersMessage> atomicBoundingBoxParameters;

   /** Lifetime of a node in milliseconds before it decays when not being hit. */
   private final AtomicReference<Long> nodeLifetimeMilliseconds;

   public REAOcTreeUpdater(double octreeResolution, REAOcTreeBuffer[] buffers, Map<REAOcTreeBuffer, AtomicReference<Pose3D>> sensorPoses, Messager reaMessager)
   {
      initializeReferenceOctree(octreeResolution);
      this.reaOcTreeBuffers = buffers;
      this.sensorPoses = sensorPoses;
      this.reaMessager = reaMessager;

      for (REAOcTreeBuffer buffer : buffers)
      {
         bufferClearRequest.put(buffer, new AtomicBoolean(false));
      }

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, true);
      enableNormalEstimation = reaMessager.createInput(REAModuleAPI.NormalEstimationEnable, true);
      clearNormals = reaMessager.createInput(REAModuleAPI.NormalEstimationClear, false);
      minRange = reaMessager.createInput(REAModuleAPI.LidarMinRange, 0.2);
      maxRange = reaMessager.createInput(REAModuleAPI.LidarMaxRange, 5.0);
      useBoundingBox = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxEnable, true);
      atomicBoundingBoxParameters = reaMessager.createInput(REAModuleAPI.OcTreeBoundingBoxParameters,
                                                            BoundingBoxMessageConverter.createBoundingBoxParametersMessage(-1.0f,
                                                                                                                           -2.0f,
                                                                                                                           -3.0f,
                                                                                                                           5.0f,
                                                                                                                           2.0f,
                                                                                                                           0.5f));
      normalEstimationParameters = reaMessager.createInput(REAModuleAPI.NormalEstimationParameters, new NormalEstimationParameters());
      nodeLifetimeMilliseconds = reaMessager.createInput(REAModuleAPI.OcTreeNodeLifetimeMillis, 60000L);

      reaMessager.addTopicListener(REAModuleAPI.RequestEntireModuleState, messageContent -> sendCurrentState());
   }

   public void initializeReferenceOctree(double octreeResolution)
   {
      referenceOctree = new NormalOcTree(octreeResolution);
      referenceOctree.enableParallelComputationForNormals(true);
      referenceOctree.enableParallelInsertionOfMisses(true);
      referenceOctree.setCustomRayMissProbabilityUpdater(new AdaptiveRayMissProbabilityUpdater());
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(REAModuleAPI.OcTreeEnable, enable.get());
      reaMessager.submitMessage(REAModuleAPI.NormalEstimationEnable, enableNormalEstimation.get());
      reaMessager.submitMessage(REAModuleAPI.OcTreeNodeLifetimeMillis, nodeLifetimeMilliseconds.get());
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
      Long nodeLifetimeMillisFile = filePropertyHelper.loadLongProperty(REAModuleAPI.OcTreeNodeLifetimeMillis.getName());
      if (nodeLifetimeMillisFile != null)
         nodeLifetimeMilliseconds.set(nodeLifetimeMillisFile);
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
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreeNodeLifetimeMillis.getName(), nodeLifetimeMilliseconds.get());
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

      boolean hasOcTreeBeenUpdated = false;

      for (REAOcTreeBuffer buffer : reaOcTreeBuffers)
      {
         AtomicReference<Pose3D> sensorPoseReference = sensorPoses.get(buffer);
         if (sensorPoseReference.get() == null)
            continue;

         Point3DReadOnly sensorOrigin = sensorPoseReference.get().getPosition();

         boolean isBufferFull = buffer.isBufferFull();
         if (isBufferFull)
            buffer.submitBufferRequest();

         NormalOcTree bufferOctree = buffer.pollNewBuffer();
         Pose3DReadOnly bufferSensorPose = buffer.pollNewSensorPoseBuffer();

         if (bufferOctree != null)
         {
            if (bufferClearRequest.get(buffer).getAndSet(false))
               referenceOctree.clear();

            PointCloud pointCloud = new PointCloud();
            bufferOctree.forEach(node -> pointCloud.add(node.getHitLocationX(), node.getHitLocationY(), node.getHitLocationZ()));
            pointCloud.setTimestamp(TimeUnit.NANOSECONDS.toMillis(System.nanoTime()));
            Scan scan = new Scan(sensorOrigin, pointCloud);
            Set<NormalOcTreeNode> updatedNodes = new HashSet<>();
            referenceOctree.insertScan(scan, updatedNodes, null);

            if (nodeLifetimeMilliseconds.get() > 0L)
               decayOcTreeNodes();

            hasOcTreeBeenUpdated = true;
         }

         if (bufferSensorPose != null)
         {
            sensorPose.set(bufferSensorPose);
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

   private void decayOcTreeNodes()
   {
      long currentTimestamp = TimeUnit.NANOSECONDS.toMillis(System.nanoTime());
      List<NormalOcTreeNode> decayedNodes = new ArrayList<>();

      for (NormalOcTreeNode node : referenceOctree)
      {
         if (currentTimestamp - node.getLastHitTimestamp() >= nodeLifetimeMilliseconds.get())
            decayedNodes.add(node);
      }
      decayedNodes.forEach(node -> referenceOctree.deleteNode(node.getKeyCopy()));
   }

   public void clearOcTreeOnNextUpdate(REAOcTreeBuffer bufferToClear)
   {
      bufferClearRequest.get(bufferToClear).set(true);
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

      Pose3D sensorPose = null;
      for (REAOcTreeBuffer buffer : reaOcTreeBuffers)
      {
         AtomicReference<Pose3D> sensorPoseReference = sensorPoses.get(buffer);
         if (sensorPoseReference.get() != null)
            sensorPose = sensorPoseReference.get();
      }

      if (sensorPose != null)
      {
         boundingBox.setOffset(sensorPose.getPosition());
         boundingBox.setYawFromQuaternion(new Quaternion(sensorPose.getOrientation()));
      }

      boundingBox.update(referenceOctree.getResolution(), referenceOctree.getTreeDepth());
      referenceOctree.setBoundingBox(boundingBox);
   }

   public NormalOcTree getMainOctree()
   {
      return referenceOctree;
   }

   public Pose3DReadOnly getSensorPose()
   {
      return sensorPose;
   }
}
