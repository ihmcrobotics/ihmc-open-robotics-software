package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.awt.image.BufferedImage;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.imageio.ImageIO;

import us.ihmc.communication.packets.ObjectDetectorResultPacket;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidBehaviors.behaviors.goalLocation.GoalDetectorBehaviorService;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.ihmcPerception.objectDetector.ObjectDetectorFromCameraImages;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;
import us.ihmc.tools.time.DateTools;

public class ObjectDetectorBehaviorService extends GoalDetectorBehaviorService
{
   private static final double DEFAULT_OBJECT_SIZE = 0.5;
   private static final double DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS = Math.toRadians(80.0);
   private static final double DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS = Math.toRadians(45.0);

   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<VideoPacket>(20);

   private final Object detectorFromCameraImagesConch = new Object();
   private final ObjectDetectorFromCameraImages objectDetectorFromCameraImages;
   private RigidBodyTransform transformFromReportedToFiducialFrame;

   private final BooleanYoVariable locationEnabled;

   private final Path videoFilesRecordingLocation = Paths.get(System.getProperty("user.home"), "diagnostic", "behaviors", "valveDetector");
   private final BooleanYoVariable shouldRecordVideoPackets;
   private final VideoPacketToImageFilesRecorder imageFilesRecorder = new VideoPacketToImageFilesRecorder(videoFilesRecordingLocation);

   public ObjectDetectorBehaviorService(CommunicationBridgeInterface communicationBridge,
                                        YoGraphicsListRegistry yoGraphicsListRegistry) throws Exception
   {
      super(ObjectDetectorBehaviorService.class.getSimpleName(), communicationBridge);

      getCommunicationBridge().attachNetworkListeningQueue(videoPacketQueue, VideoPacket.class);

      transformFromReportedToFiducialFrame = new RigidBodyTransform();
      objectDetectorFromCameraImages = new ObjectDetectorFromCameraImages(transformFromReportedToFiducialFrame, getYoVariableRegistry(), yoGraphicsListRegistry);

      objectDetectorFromCameraImages.setFieldOfView(DEFAULT_FIELD_OF_VIEW_X_IN_RADIANS, DEFAULT_FIELD_OF_VIEW_Y_IN_RADIANS);
      objectDetectorFromCameraImages.setExpectedObjectSize(DEFAULT_OBJECT_SIZE);

      getCommunicationBridge().attachListener(ObjectDetectorResultPacket.class, objectDetectorFromCameraImages);
      
      String prefix = "fiducial";
      locationEnabled = new BooleanYoVariable(prefix + "LocationEnabled", getYoVariableRegistry());

      shouldRecordVideoPackets = new BooleanYoVariable("ShouldRecordVideoPackets", getYoVariableRegistry());
      shouldRecordVideoPackets.set(false);
      
      locationEnabled.set(false);

      objectDetectorFromCameraImages.addDetectionResultListener(detectionVisualizationPackets -> {
         getCommunicationBridge().sendPacketToUI(detectionVisualizationPackets.getBoundingBoxesPacket());
         getCommunicationBridge().sendPacketToUI(detectionVisualizationPackets.getHeatMapPacket());
      });
   }

   @Override
   public void doThreadAction()
   {
      if (videoPacketQueue.isNewPacketAvailable())
      {
         VideoPacket videoPacket = videoPacketQueue.getLatestPacket();

         if(shouldRecordVideoPackets.getBooleanValue())
         {
            imageFilesRecorder.queueVideoPacket(videoPacket);
         }

         synchronized (detectorFromCameraImagesConch)
         {
            objectDetectorFromCameraImages.detectFromVideoPacket(videoPacket);
         }
      }
      else
      {
         ThreadTools.sleep(10);
      }
   }

   public void setExpectedObjectSize(double expectedFiducialSize)
   {
      synchronized (detectorFromCameraImagesConch)
      {
         objectDetectorFromCameraImages.setExpectedObjectSize(expectedFiducialSize);
      }
   }

   @Override
   public boolean getGoalHasBeenLocated()
   {
      synchronized (detectorFromCameraImagesConch)
      {
         return objectDetectorFromCameraImages.getTargetIDHasBeenLocated();
      }
   }

   @Override
   public void getReportedGoalPoseWorldFrame(FramePose framePoseToPack)
   {
      synchronized (detectorFromCameraImagesConch)
      {
         objectDetectorFromCameraImages.getReportedFiducialPoseWorldFrame(framePoseToPack);
      }
   }
   
   @Override
   public void run()
   {
      super.run();
      locationEnabled.set(true);
   }
   
   @Override
   public void pause()
   {
      super.pause();
      locationEnabled.set(false);
   }
   
   @Override
   public void destroy()
   {
      super.destroy();
      locationEnabled.set(false);
   }

   @Override
   public void initialize()
   {
      objectDetectorFromCameraImages.reset();
   }

   private class VideoPacketToImageFilesRecorder implements Runnable
   {
      private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
      private final ConcurrentLinkedQueue<BufferedImage> imagesQueue = new ConcurrentLinkedQueue<>();
      private final Path workingDirectory;
      private final Path sessionDirectory;

      private long index = 0;

      private final Object waitLock = new Object();
      private boolean waiting;

      public VideoPacketToImageFilesRecorder(Path workingDirectory) throws IOException
      {
         String dateString = DateTools.getDateString();
         String timeStringWithSeconds = DateTools.getTimeStringWithSeconds();

         this.workingDirectory = workingDirectory;
         Files.createDirectories(this.workingDirectory);

         this.sessionDirectory = this.workingDirectory.resolve(dateString + "_" + timeStringWithSeconds);
         Files.createDirectories(this.sessionDirectory);
      }

      public void queueVideoPacket(VideoPacket packet)
      {
         imagesQueue.add(jpegDecompressor.decompressJPEGDataToBufferedImage(packet.getData()));

         if(waiting)
         {
            synchronized (waitLock)
            {
               waitLock.notify();
            }
         }
      }

      @Override
      public void run()
      {
         while(true)
         {
            waiting = false;
            while(!imagesQueue.isEmpty())
            {
               index++;
               BufferedImage nextImage = imagesQueue.poll();
               String numberString = String.format("%030d", index);
               String imageFileNameString = "img" + numberString + ".png";
               Path imageFilePath = this.sessionDirectory.resolve(imageFileNameString);

               File imageFile = null;
               try
               {
                  imageFile = Files.createFile(imageFilePath).toFile();
               }
               catch (IOException e)
               {
                  PrintTools.error(this, "Could not create image file: " + imageFilePath);
                  continue;
               }

               try
               {
                  ImageIO.write(nextImage, "png", imageFile);
               }
               catch (IOException e)
               {
                  PrintTools.error(this, "Could not write image to file: " + imageFilePath);
               }
            }

            synchronized (waitLock)
            {
               waiting = true;
               try
               {
                  waitLock.wait();
               }
               catch (InterruptedException ignored)
               {

               }
            }
         }
      }
   }
}
