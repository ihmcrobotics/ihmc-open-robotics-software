package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.awt.image.BufferedImage;
import java.util.ArrayList;

import javax.vecmath.Point2d;

import us.ihmc.communication.producers.JPEGCompressor;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.humanoidBehaviors.behaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.ihmcPerception.vision.HSVValue;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.ihmcPerception.vision.shapes.HoughCircleResult;
import us.ihmc.ihmcPerception.vision.shapes.OpenCVColoredCircularBlobDetector;
import us.ihmc.ihmcPerception.vision.shapes.OpenCVColoredCircularBlobDetectorFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.thread.ThreadTools;

public class ColoredCircularBlobDetectorBehaviorService extends ThreadedBehaviorService
{
   private static final boolean DEBUG = true;
   
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<>();
   private final ConcurrentListeningQueue<RobotConfigurationData> robotConfigurationDataQueue = new ConcurrentListeningQueue<>();
   private long videoTimestamp = -1L;
   
   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private final JPEGCompressor jpegCompressor = new JPEGCompressor();
   
   private final OpenCVColoredCircularBlobDetector openCVColoredCircularBlobDetector;
   private static int counter = 0;
   private final Point2d latestBallPosition2d = new Point2d();
   
   public ColoredCircularBlobDetectorBehaviorService(BehaviorInterface behaviorInterface)
   {
      super(ColoredCircularBlobDetectorBehaviorService.class.getSimpleName(), behaviorInterface);
      
      getBehaviorInterface().attachNetworkProcessorListeningQueue(videoPacketQueue, VideoPacket.class);
      getBehaviorInterface().attachNetworkProcessorListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);
      
      OpenCVColoredCircularBlobDetectorFactory factory = new OpenCVColoredCircularBlobDetectorFactory();
      factory.setCaptureSource(OpenCVColoredCircularBlobDetector.CaptureSource.JAVA_BUFFERED_IMAGES);
      openCVColoredCircularBlobDetector = factory.buildBlobDetector();

      HSVRange greenRange = new HSVRange(new HSVValue(78, 100, 100), new HSVValue(83, 255, 255));
      openCVColoredCircularBlobDetector.addHSVRange(greenRange);
   }

   @Override
   public void initialize()
   {
      start();
   }

   @Override
   public void doThreadAction()
   {
      if (videoPacketQueue.isNewPacketAvailable())
      {
         VideoPacket packet = videoPacketQueue.getLatestPacket();
         RobotConfigurationData robotConfigurationData = robotConfigurationDataQueue.getLatestPacket();
         videoTimestamp = robotConfigurationData.getTimestamp();

         BufferedImage bufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(packet.getData());

         openCVColoredCircularBlobDetector.updateFromBufferedImage(bufferedImage);
         ArrayList<HoughCircleResult> circles = openCVColoredCircularBlobDetector.getCircles();

         BufferedImage thresholdBufferedImage = OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getThresholdMat());
         BufferedImage convertedBufferedImage = new BufferedImage(thresholdBufferedImage.getWidth(), thresholdBufferedImage.getHeight(),
                                                                  BufferedImage.TYPE_INT_RGB);
         convertedBufferedImage.getGraphics().drawImage(thresholdBufferedImage, 0, 0, null);

         byte[] jpegThresholdImage = jpegCompressor.convertBufferedImageToJPEGData(convertedBufferedImage);
         VideoPacket circleBlobThresholdImagePacket = new VideoPacket(RobotSide.LEFT, VideoSource.CV_THRESHOLD, videoTimestamp, jpegThresholdImage,
                                                                      packet.getPosition(), packet.getOrientation(), packet.getIntrinsicParameters());
         getBehaviorInterface().sendPacketToNetworkProcessor(circleBlobThresholdImagePacket);

         if (DEBUG)
         {
            if (counter++ % 30 == 0)
            {
               System.out.println("blob detection found " + circles.size() + " circles");

               for (int i = 0; i < circles.size(); i++)
               {
                  HoughCircleResult result = circles.get(i);
                  System.out.println("\t center: " + result.getCenter());
               }
            }
         }

         if (circles.size() > 0)
            latestBallPosition2d.set(circles.get(0).getCenter());
      }
      else
      {
         ThreadTools.sleep(10);
      }
   }

   public Point2d getLatestBallPosition2d()
   {
      return latestBallPosition2d;
   }
}
