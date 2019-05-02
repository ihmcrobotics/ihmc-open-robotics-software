package us.ihmc.robotEnvironmentAwareness.fusion.objectDetection;

import java.awt.image.BufferedImage;
import java.io.BufferedReader;
import java.io.DataOutputStream;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.ConnectException;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicReference;

import controller_msgs.msg.dds.DoorParameterPacket;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import sensor_msgs.msg.dds.RegionOfInterest;
import us.ihmc.commons.Conversions;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.log.LogTools;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.tools.ImageVisualizationHelper;
import us.ihmc.ros2.Ros2Node;

public class FusionSensorObjectDetectionManager
{
   private final SharedMemoryJavaFXMessager messager;

   private static final int socketPort = 65535;
   private static final long milliSecondsForOneTick = 10;
   private static final double maximumTimeToWaitResult = 20.0;
   private Socket objectDetectionSocket;
   private boolean imageRequested;

   private final List<ObjectType> selectedObjectTypes = new ArrayList<>();
   private final Map<ObjectType, RegionOfInterest> objectTypeToROIMap = new HashMap<>();

   private final AbstractObjectParameterCalculator<DoorParameterPacket> doorParameterCalculator;

   private final AtomicReference<StereoVisionPointCloudMessage> latestStereoVisionPointCloudMessage = new AtomicReference<>(null);
   private final AtomicReference<BufferedImage> leatestBufferedImage = new AtomicReference<BufferedImage>();

   public FusionSensorObjectDetectionManager(Ros2Node ros2Node, SharedMemoryJavaFXMessager messager)
   {
      this.messager = messager;
      doorParameterCalculator = new DoorParameterCalculator(ros2Node, DoorParameterPacket.class);

      TimerTask socketTimerTask = new TimerTask()
      {
         private double waitingTime = 0;

         @Override
         public void run()
         {
            if (!imageRequested)
               return;

            waitingTime += (double) milliSecondsForOneTick / 1000;
            if (waitingTime > maximumTimeToWaitResult)
            {
               done();
               LogTools.info("# detecting time out!");
            }

            try
            {
               InputStreamReader inputStreamReader = new InputStreamReader(objectDetectionSocket.getInputStream());
               BufferedReader bufferedReader = new BufferedReader(inputStreamReader);

               if (!bufferedReader.ready())
                  return;

               int[] roiData = ObjectDetectionSocketHelper.convertStringToIntArray(bufferedReader.readLine());
               objectTypeToROIMap.clear();
               for (int i = 0; i < roiData.length; i += 5)
               {
                  if (roiData[i] == -1)
                     return;
                  ObjectType detectedObjectType = ObjectType.values()[roiData[i]];
                  int xmin = roiData[i + 1];
                  int xmax = roiData[i + 2];
                  int ymin = roiData[i + 3];
                  int ymax = roiData[i + 4];
                  RegionOfInterest roi = new RegionOfInterest();
                  roi.setXOffset(xmin);
                  roi.setYOffset(ymin);
                  roi.setHeight(ymax - ymin);
                  roi.setWidth(xmax - xmin);
                  objectTypeToROIMap.put(detectedObjectType, roi);
               }
               LogTools.info("# detecting time tooks " + waitingTime + " seconds.");
               done();
               publishResults();
            }
            catch (IOException e)
            {
               e.printStackTrace();
            }
         }

         private void done()
         {
            waitingTime = 0.0;
            imageRequested = false;
         }
      };
      Timer socketTimer = new Timer();
      socketTimer.schedule(socketTimerTask, 0, milliSecondsForOneTick);
   }

   public void updateLatestStereoVisionPointCloudMessage(StereoVisionPointCloudMessage message)
   {
      latestStereoVisionPointCloudMessage.set(message);
   }

   public void connectExternalModule(String moduleAddress)
   {
      LogTools.info("server address to connect is " + moduleAddress);

      try
      {
         objectDetectionSocket = new Socket(moduleAddress, socketPort);
      }
      catch (UnknownHostException | ConnectException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public void requestObjectDetection(BufferedImage image, List<ObjectType> selectedTypes)
   {
      leatestBufferedImage.set(image);
      byte[] imgBytes = ObjectDetectionSocketHelper.convertImgToBytes(image);
      byte[] imgDimBytes = ObjectDetectionSocketHelper.convertImgDimToBytes(imgBytes.length, image.getWidth(), image.getHeight());
      try
      {
         DataOutputStream dataOutputStream = new DataOutputStream(objectDetectionSocket.getOutputStream());
         dataOutputStream.write(imgDimBytes);
         dataOutputStream.write(imgBytes);
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
      selectedObjectTypes.clear();
      selectedObjectTypes.addAll(selectedTypes);
      imageRequested = true;
   }

   public void close() throws Exception
   {
      if (objectDetectionSocket != null)
         if (!objectDetectionSocket.isClosed())
            objectDetectionSocket.close();
   }

   private void publishResults()
   {
      for (ObjectType objectType : selectedObjectTypes)
      {
         RegionOfInterest detectedObjectROI = objectTypeToROIMap.get(objectType);
         if (detectedObjectROI != null)
         {
            ImageVisualizationHelper.drawROIOnImage(leatestBufferedImage.get(), detectedObjectROI, objectType.getROIColor(), objectType.toString());
            switch (objectType)
            {
            case Cup:
               break;
            case Door:
               calculateAndPublishDoorParameter(detectedObjectROI, objectTypeToROIMap.get(ObjectType.DoorHandle));
               break;
            case DoorHandle:
               break;
            case Human:
               break;
            default:
               break;
            }
            messager.submitMessage(LidarImageFusionAPI.ImageResultState, leatestBufferedImage.getAndSet(null));
         }
      }
   }

   /**
    * To calculate {@code DoorParameterPacket}, calculator needs handle ROI with door ROI.
    * @param doorROI
    * @param handleROI
    */
   private void calculateAndPublishDoorParameter(RegionOfInterest doorROI, RegionOfInterest handleROI)
   {
      long startTime = System.nanoTime();
      doorParameterCalculator.initialize();
      doorParameterCalculator.trimPointCloudInROI(latestStereoVisionPointCloudMessage.get(), doorROI);
      doorParameterCalculator.calculate(handleROI);
      doorParameterCalculator.publish();
      long computingTime = System.nanoTime() - startTime;
      LogTools.info("Door computing time is " + Conversions.nanosecondsToMicroseconds(computingTime));
   }
}