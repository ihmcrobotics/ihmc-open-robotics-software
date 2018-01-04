package us.ihmc.robotEnvironmentAwareness.updaters;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.LidarScanMessage;
import us.ihmc.communication.packets.RequestLidarScanMessage;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;

public class REAOcTreeBuffer
{
   private static final int NUMBER_OF_SAMPLES = 100000;

   private final AtomicReference<LidarScanMessage> latestLidarScanMessage = new AtomicReference<>(null);
   private final AtomicReference<ScanCollection> newFullScanReference = new AtomicReference<>(null);

   private final AtomicReference<Boolean> enable;
   private final AtomicReference<Integer> bufferSize;

   private final AtomicBoolean clearBuffer = new AtomicBoolean(false);
   private final AtomicBoolean isBufferFull = new AtomicBoolean(false);
   private final AtomicBoolean isBufferRequested = new AtomicBoolean(false);
   private final AtomicReference<NormalOcTree> newBuffer = new AtomicReference<>(null);
   
   private final double octreeResolution;

   private final PacketCommunicator publicPacketCommunicator;

   private final REAModuleStateReporter moduleStateReporter;

   private final REAMessager reaMessager;

   public REAOcTreeBuffer(double octreeResolution, REAMessager reaMessager, REAModuleStateReporter moduleStateReporter, PacketCommunicator publicPacketCommunicator)
   {
      this.octreeResolution = octreeResolution;
      this.reaMessager = reaMessager;
      this.moduleStateReporter = moduleStateReporter;
      this.publicPacketCommunicator = publicPacketCommunicator;

      enable = reaMessager.createInput(REAModuleAPI.OcTreeEnable, true);
      bufferSize = reaMessager.createInput(REAModuleAPI.OcTreeBufferSize, 10000);

      reaMessager.registerTopicListener(REAModuleAPI.RequestEntireModuleState, (messageContent) -> sendCurrentState());
      publicPacketCommunicator.attachListener(LidarScanMessage.class, this::handlePacket);
   }

   private void sendCurrentState()
   {
      reaMessager.submitMessage(REAModuleAPI.OcTreeBufferSize, bufferSize.get());
   }

   public void loadConfiguration(FilePropertyHelper filePropertyHelper)
   {
      Boolean enableFile = filePropertyHelper.loadBooleanProperty(REAModuleAPI.OcTreeEnable.getName());
      if (enableFile != null)
         enable.set(enableFile);
      Integer bufferSizeFile = filePropertyHelper.loadIntegerProperty(REAModuleAPI.OcTreeBufferSize.getName());
      if (bufferSizeFile != null)
         bufferSize.set(bufferSizeFile);
   }

   public void saveConfiguration(FilePropertyHelper filePropertyHelper)
   {
      filePropertyHelper.saveProperty(REAModuleAPI.OcTreeBufferSize.getName(), bufferSize.get());
   }

   public Runnable createBufferThread()
   {
      return new Runnable()
      {
         private NormalOcTree bufferOctree = new NormalOcTree(octreeResolution);

         @Override
         public void run()
         {
            publicPacketCommunicator.send(new RequestLidarScanMessage());

            updateScanCollection();
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
            isBufferFull.set(numberOfLeafNodesInBuffer >= bufferSize.get().intValue());

            if (isBufferRequested.get())
            {
               newBuffer.set(bufferOctree);
               bufferOctree = new NormalOcTree(octreeResolution);
               isBufferRequested.set(false);
            }

            moduleStateReporter.reportBufferOcTreeState(bufferOctree);
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

   private void updateScanCollection()
   {
      LidarScanMessage lidarScanMessage = latestLidarScanMessage.getAndSet(null);

      if (!enable.get() || lidarScanMessage == null)
         return;

      ScanCollection scanCollection = new ScanCollection();
      newFullScanReference.set(scanCollection);
      scanCollection.setSubSampleSize(NUMBER_OF_SAMPLES);
      scanCollection.addScan(lidarScanMessage.scan, lidarScanMessage.lidarPosition);
   }

   private void handlePacket(LidarScanMessage packet)
   {
      if (packet != null)
         latestLidarScanMessage.set(packet);
   }
}
