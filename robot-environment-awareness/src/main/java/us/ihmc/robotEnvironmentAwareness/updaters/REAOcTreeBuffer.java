package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.LidarScanMessage;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.idl.IDLSequence.Float;
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
   private final AtomicReference<ScanCollection> newFullScanReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Boolean> enableBuffer;
   private final AtomicReference<Integer> ocTreeCapacity;
   private final AtomicReference<Integer> messageCapacity;

   private final AtomicBoolean clearBuffer = new AtomicBoolean(false);
   private final AtomicBoolean isBufferFull = new AtomicBoolean(false);
   private final AtomicBoolean isBufferRequested = new AtomicBoolean(false);
   private final AtomicReference<NormalOcTree> newBuffer = new AtomicReference<>(null);

   private final AtomicReference<Boolean> isBufferStateRequested;

   private final double octreeResolution;
   private int messageCounter = 0;
   private final AtomicReference<Integer> octreeDepth;

   private final Messager reaMessager;

   private final Topic<Boolean> enableBufferTopic;
   private final Topic<Integer> ocTreeCapacityTopic;
   private final Topic<Integer> messageCapacityTopic;
   private final Topic<NormalOcTreeMessage> stateTopic;
   
   private final AtomicReference<Integer> stereoVisionBufferSize;

   public REAOcTreeBuffer(double octreeResolution, Messager reaMessager, Topic<Boolean> enableBufferTopic, boolean enableBufferInitialValue,
                          Topic<Integer> ocTreeCapacityTopic, int ocTreeCapacityValue, Topic<Integer> messageCapacityTopic, int messageCapacityInitialValue,
                          Topic<Boolean> requestStateTopic, Topic<NormalOcTreeMessage> stateTopic)
   {
      this.octreeResolution = octreeResolution;
      this.reaMessager = reaMessager;
      this.enableBufferTopic = enableBufferTopic;
      this.ocTreeCapacityTopic = ocTreeCapacityTopic;
      this.messageCapacityTopic = messageCapacityTopic;
      this.stateTopic = stateTopic;

      octreeDepth = reaMessager.createInput(REAModuleAPI.OcTreeDepth, 15);

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, true);
      enableBuffer = reaMessager.createInput(enableBufferTopic, enableBufferInitialValue);
      ocTreeCapacity = reaMessager.createInput(ocTreeCapacityTopic, ocTreeCapacityValue);
      messageCapacity = reaMessager.createInput(messageCapacityTopic, messageCapacityInitialValue);

      isBufferStateRequested = reaMessager.createInput(requestStateTopic, false);

      reaMessager.registerTopicListener(REAModuleAPI.RequestEntireModuleState, (messageContent) -> sendCurrentState());
      stereoVisionBufferSize = reaMessager.createInput(REAModuleAPI.StereoVisionBufferSize, NUMBER_OF_SAMPLES);
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
         private NormalOcTree bufferOctree = new NormalOcTree(octreeResolution);

         @Override
         public void run()
         {
            updateScanCollection();
            ScanCollection newScan = newFullScanReference.getAndSet(null);

            if (!enable.get() || !enableBuffer.get() || clearBuffer.getAndSet(false))
            {
               bufferOctree.clear();
               messageCounter = 0;
               isBufferFull.set(false);
               isBufferRequested.set(false);
               return;
            }

            if (newScan == null)
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
               bufferOctree = new NormalOcTree(octreeResolution);
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

   public void handleStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
   }

   public void handleLidarScanMessage(LidarScanMessage message)
   {
      latestLidarScanMessage.set(message);
   }

   private void updateScanCollection()
   {
      LidarScanMessage lidarMessage = latestLidarScanMessage.getAndSet(null);
      StereoVisionPointCloudMessage stereoMessage = latestStereoVisionPointCloudMessage.getAndSet(null);

      if (!enable.get() || !enableBuffer.get())
         return;

      if (lidarMessage != null)
      {
         ScanCollection scanCollection = new ScanCollection();
         newFullScanReference.set(scanCollection);
         scanCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
         // FIXME Not downsizing the scan anymore, this needs to be reviewed to improve speed.
         scanCollection.addScan(toScan(lidarMessage.getScan(), lidarMessage.getLidarPosition()));
      }

      if (stereoMessage != null)
      {
         ScanCollection scanCollection = new ScanCollection();
         newFullScanReference.set(scanCollection);
         scanCollection.setSubSampleSize(stereoVisionBufferSize.get());
         // FIXME Not downsizing the scan anymore, this needs to be reviewed to improve speed.
         scanCollection.addScan(toScan(stereoMessage.getPointCloud(), stereoMessage.getSensorPosition()));
         // TODO: make NormalOctree constructor with octreeDepth.get().
      }
   }

   private static Scan toScan(Float data, Point3DReadOnly sensorPosition)
   {
      PointCloud pointCloud = new PointCloud();

      int bufferIndex = 0;

      while (bufferIndex < data.size())
      {
         float x = data.getQuick(bufferIndex++);
         float y = data.getQuick(bufferIndex++);
         float z = data.getQuick(bufferIndex++);
         pointCloud.add(x, y, z);
      }
      return new Scan(sensorPosition, pointCloud);
   }
}
