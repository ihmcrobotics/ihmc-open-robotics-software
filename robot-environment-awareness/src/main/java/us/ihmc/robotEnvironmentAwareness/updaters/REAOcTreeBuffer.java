package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import perception_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.StereoPointCloudCompression;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.idl.IDLSequence;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.Scan;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.communication.converters.OcTreeMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.packets.NormalOcTreeMessage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

public class REAOcTreeBuffer
{
   private static final int NUMBER_OF_SAMPLES = 100000;

   private final AtomicReference<LidarScanMessage> latestLidarScanMessage = new AtomicReference<>(null);
   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);
   private final AtomicReference<StereoVisionPointCloudMessage> latestDepthCloudPointCloudMessage = new AtomicReference<>(null);
   private final AtomicReference<ScanCollection> newFullScanReference = new AtomicReference<>(null);
   private final AtomicReference<Pose3DReadOnly> newSensorPoseReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> enableBuffer;
   private final AtomicReference<Integer> ocTreeCapacity;
   private final AtomicReference<Integer> messageCapacity;

   private final AtomicBoolean clearBuffer = new AtomicBoolean(false);
   private final AtomicBoolean isBufferFull = new AtomicBoolean(false);
   private final AtomicBoolean isBufferRequested = new AtomicBoolean(false);
   private final AtomicReference<NormalOcTree> newBuffer = new AtomicReference<>(null);
   private final AtomicReference<Pose3DReadOnly> newSensorPoseBuffer = new AtomicReference<>(null);

   private final AtomicReference<Boolean> isBufferStateRequested;

   private final AtomicReference<Double> octreeResolution;
   private int messageCounter = 0;

   private final Messager reaMessager;

   private final Topic<Boolean> enableBufferTopic;
   private final Topic<Integer> ocTreeCapacityTopic;
   private final Topic<Integer> messageCapacityTopic;
   private final Topic<NormalOcTreeMessage> stateTopic;

   private final AtomicReference<Integer> stereoVisionBufferSize;
   private final AtomicReference<Integer> depthCloudBufferSize;

   public REAOcTreeBuffer(double octreeResolution, Messager reaMessager, Topic<Boolean> enableBufferTopic, boolean enableBufferInitialValue,
                          Topic<Integer> ocTreeCapacityTopic, int ocTreeCapacityValue, Topic<Integer> messageCapacityTopic, int messageCapacityInitialValue,
                          Topic<Boolean> requestStateTopic, Topic<NormalOcTreeMessage> stateTopic)
   {
      this.octreeResolution = new AtomicReference<Double>(octreeResolution);
      this.reaMessager = reaMessager;
      this.enableBufferTopic = enableBufferTopic;
      this.ocTreeCapacityTopic = ocTreeCapacityTopic;
      this.messageCapacityTopic = messageCapacityTopic;
      this.stateTopic = stateTopic;

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, true);
      enableBuffer = reaMessager.createInput(enableBufferTopic, enableBufferInitialValue);
      ocTreeCapacity = reaMessager.createInput(ocTreeCapacityTopic, ocTreeCapacityValue);
      messageCapacity = reaMessager.createInput(messageCapacityTopic, messageCapacityInitialValue);

      isBufferStateRequested = reaMessager.createInput(requestStateTopic, false);

      reaMessager.addTopicListener(REAModuleAPI.RequestEntireModuleState, (messageContent) -> sendCurrentState());
      stereoVisionBufferSize = reaMessager.createInput(REAModuleAPI.StereoVisionBufferSize, NUMBER_OF_SAMPLES);
      depthCloudBufferSize = reaMessager.createInput(REAModuleAPI.DepthCloudBufferSize, NUMBER_OF_SAMPLES);
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(enableBufferTopic, enableBuffer.get());
      reaMessager.submitMessage(ocTreeCapacityTopic, ocTreeCapacity.get());
      reaMessager.submitMessage(messageCapacityTopic, messageCapacity.get());
   }

   public void loadConfiguration(FilePropertyHelper filePropertyHelper)
   {
      Boolean enableFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreeEnable.getName());
      if (enableFile != null)
         enable.set(enableFile);
      Boolean enableBufferFile = filePropertyHelper.loadBooleanProperty(enableBufferTopic.getName());
      if (enableBufferFile != null)
         enableBuffer.set(enableBufferFile);
      Integer ocTreeCapacityFile = filePropertyHelper.loadIntegerProperty(ocTreeCapacityTopic.getName());
      if (ocTreeCapacityFile != null)
         ocTreeCapacity.set(ocTreeCapacityFile);
      Integer messageCapacityFile = filePropertyHelper.loadIntegerProperty(messageCapacityTopic.getName());
      if (messageCapacityFile != null)
         messageCapacity.set(messageCapacityFile);
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(enableBufferTopic.getName(), enableBuffer.get());
      filePropertyHelper.saveProperty(ocTreeCapacityTopic.getName(), ocTreeCapacity.get());
      filePropertyHelper.saveProperty(messageCapacityTopic.getName(), messageCapacity.get());
   }

   public Runnable createBufferThread()
   {
      return new Runnable()
      {
         private NormalOcTree bufferOctree = new NormalOcTree(octreeResolution.get());

         @Override
         public void run()
         {
            updateScanCollection();
            ScanCollection newScan = newFullScanReference.getAndSet(null);
            Pose3DReadOnly newSensorPose = newSensorPoseReference.getAndSet(null);

            if (!enable.get() || !enableBuffer.get() || clearBuffer.getAndSet(false))
            {
               bufferOctree.clear();
               messageCounter = 0;
               isBufferFull.set(false);
               isBufferRequested.set(false);
               return;
            }

            if (newScan == null || newSensorPose == null) //TODO: check.
               return;

            bufferOctree.insertScanCollection(newScan, false);
            messageCounter++;

            if (messageCounter >= messageCapacity.get().intValue())
            {
               isBufferFull.set(true);
            }
            else
            {
               int numberOfLeafNodesInBuffer = bufferOctree.getNumberOfLeafNodes();
               if (numberOfLeafNodesInBuffer >= ocTreeCapacity.get().intValue())
                  isBufferFull.set(true);
               else
               {
                  isBufferFull.set(false);
                  return;
               }
            }

            if (isBufferRequested.get())
            {
               newBuffer.set(bufferOctree);
               newSensorPoseBuffer.set(newSensorPose);
               bufferOctree = new NormalOcTree(octreeResolution.get());
               isBufferRequested.set(false);
               messageCounter = 0;
            }

            if (isBufferStateRequested.getAndSet(false))
               reaMessager.submitMessage(stateTopic, OcTreeMessageConverter.convertToMessage(bufferOctree));
         }
      };
   }

   public void clearBuffer()
   {
      clearBuffer.set(true);
   }

   public void submitBufferRequest()
   {
      isBufferRequested.set(true);
   }

   public boolean isBufferFull()
   {
      return isBufferFull.get();
   }

   public NormalOcTree pollNewBuffer()
   {
      return newBuffer.getAndSet(null);
   }

   public Pose3DReadOnly pollNewSensorPoseBuffer()
   {
      return newSensorPoseBuffer.getAndSet(null);
   }

   public void setOctreeResolution(double resolution)
   {
      octreeResolution.set(resolution);
   }

   public void handleStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
   }

   public void handleDepthCloudPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestDepthCloudPointCloudMessage.set(message);
   }

   public void handleLidarScanMessage(LidarScanMessage message)
   {
      latestLidarScanMessage.set(message);
   }

   private void updateScanCollection()
   {
      LidarScanMessage lidarMessage = latestLidarScanMessage.getAndSet(null);
      StereoVisionPointCloudMessage stereoMessage = latestStereoVisionPointCloudMessage.getAndSet(null);
      StereoVisionPointCloudMessage depthCloudMessage = latestDepthCloudPointCloudMessage.getAndSet(null);

      if (!enable.get() || !enableBuffer.get())
         return;

      if (lidarMessage != null)
      {
         ScanCollection scanCollection = new ScanCollection();
         newFullScanReference.set(scanCollection);
         scanCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
         // FIXME Not downsizing the scan anymore, this needs to be reviewed to improve speed.
         scanCollection.addScan(toScan(lidarMessage.getScan(), lidarMessage.getNumberOfPoints(), lidarMessage.getLidarPosition()));

         Pose3D sensorPose = new Pose3D();
         sensorPose.getPosition().set(lidarMessage.getLidarPosition());
         sensorPose.getOrientation().set(lidarMessage.getLidarOrientation());
         newSensorPoseReference.set(sensorPose);
      }

      if (stereoMessage != null)
      {
         ScanCollection scanCollection = new ScanCollection();
         newFullScanReference.set(scanCollection);
         scanCollection.setSubSampleSize(stereoVisionBufferSize.get());
         // FIXME Not downsizing the scan anymore, this needs to be reviewed to improve speed.
         scanCollection.addScan(toScan(stereoMessage));
         // TODO: make NormalOctree constructor with octreeDepth.get().

         Pose3D sensorPose = new Pose3D();
         sensorPose.getPosition().set(stereoMessage.getSensorPosition());
         sensorPose.getOrientation().set(stereoMessage.getSensorOrientation());
         newSensorPoseReference.set(sensorPose);
      }

      if (depthCloudMessage != null)
      {
         ScanCollection scanCollection = new ScanCollection();
         newFullScanReference.set(scanCollection);
         scanCollection.setSubSampleSize(depthCloudBufferSize.get());
         // FIXME Not downsizing the scan anymore, this needs to be reviewed to improve speed.
         scanCollection.addScan(toScan(stereoMessage));
         // TODO: make NormalOctree constructor with octreeDepth.get().

         Pose3D sensorPose = new Pose3D();
         sensorPose.getPosition().set(depthCloudMessage.getSensorPosition());
         sensorPose.getOrientation().set(depthCloudMessage.getSensorOrientation());
         newSensorPoseReference.set(sensorPose);
      }
   }

   private static Scan toScan(StereoVisionPointCloudMessage stereoMessage)
   {
      PointCloud pointCloud = new PointCloud();
      StereoPointCloudCompression.decompressPointCloud(stereoMessage, pointCloud::add);
      return new Scan(stereoMessage.getSensorPosition(), pointCloud);
   }

   private static Scan toScan(IDLSequence.Byte data, int numberOfPoints, Point3DReadOnly sensorPosition)
   {
      PointCloud pointCloud = new PointCloud();
      LidarPointCloudCompression.decompressPointCloud(data, numberOfPoints, (i, x, y, z) -> pointCloud.add(x, y, z));
      return new Scan(sensorPosition, pointCloud);
   }
}
