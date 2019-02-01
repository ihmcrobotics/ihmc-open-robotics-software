package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class REAStereoVisionBuffer
{
   private static final int NUMBER_OF_SAMPLES = 100000;
   private static final int NUMBER_OF_SNAPSHOT = 1;

   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);

   private final REAModuleStateReporter moduleStateReporter;

   private final Messager reaMessager;

   private final AtomicReference<ScanCollection> newFullScanReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Integer> bufferSize;

   private int stereoVisionBufferSize = 0;
   private final AtomicBoolean clearBuffer = new AtomicBoolean(false);
   private final AtomicBoolean isBufferFull = new AtomicBoolean(false);
   private final AtomicBoolean isBufferRequested = new AtomicBoolean(false);
   private final AtomicReference<NormalOcTree> newBuffer = new AtomicReference<>(null);

   private final double octreeResolution = 0.02;

   private final Point3D defaultPositionOfSensorOrigin = new Point3D();

   public REAStereoVisionBuffer(Messager reaMessager, REAModuleStateReporter moduleStateReporter)
   {
      this.reaMessager = reaMessager;
      this.moduleStateReporter = moduleStateReporter;

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, true);
      bufferSize = reaMessager.createInput(REAModuleAPI.OcTreeBufferSize, 10000);

      reaMessager.registerTopicListener(REAModuleAPI.RequestEntireModuleState, (messageContent) -> sendCurrentState());
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(REAModuleAPI.OcTreeBufferSize, bufferSize.get());
   }

   public Runnable createBufferThread()
   {
      return new Runnable()
      {
         private NormalOcTree bufferOctree = new NormalOcTree(octreeResolution);

         @Override
         public void run()
         {
            updateStereoVision();

            ScanCollection newScan = newFullScanReference.getAndSet(null);

            if (clearBuffer.getAndSet(false))
            {
               bufferOctree.clear();
               isBufferFull.set(false);
               isBufferRequested.set(false);
               return;
            }

            if (newScan == null)
               return;

            bufferOctree.insertScanCollection(newScan, false);

            int numberOfLeafNodesInBuffer = bufferOctree.getNumberOfLeafNodes();

            isBufferFull.set(numberOfLeafNodesInBuffer >= bufferSize.get().intValue() && stereoVisionBufferSize >= NUMBER_OF_SNAPSHOT);

            if (isBufferRequested.get())
            {
               newBuffer.set(bufferOctree);
               bufferOctree = new NormalOcTree(octreeResolution);
               isBufferRequested.set(false);
               stereoVisionBufferSize = 0;
            }

            moduleStateReporter.reportBufferOcTreeState(bufferOctree);
         }
      };
   }

   private void updateStereoVision()
   {
      StereoVisionPointCloudMessage stereoVisionPointCloudMessage = latestStereoVisionPointCloudMessage.getAndSet(null);

      if (!enable.get() || stereoVisionPointCloudMessage == null)
         return;

      ScanCollection scanCollection = new ScanCollection();
      newFullScanReference.set(scanCollection);
      scanCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
      scanCollection.addScan(stereoVisionPointCloudMessage.getPointCloud().toArray(), defaultPositionOfSensorOrigin);
      stereoVisionBufferSize++;
   }

   public void handleStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
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
}
