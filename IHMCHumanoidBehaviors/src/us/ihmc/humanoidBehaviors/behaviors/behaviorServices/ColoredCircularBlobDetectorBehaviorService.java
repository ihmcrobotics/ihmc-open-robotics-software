package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Point2d;

import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import us.ihmc.communication.producers.JPEGCompressor;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.ihmcPerception.vision.shapes.HoughCircleResult;
import us.ihmc.ihmcPerception.vision.shapes.OpenCVColoredCircularBlobDetector;
import us.ihmc.ihmcPerception.vision.shapes.OpenCVColoredCircularBlobDetectorFactory;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.thread.ThreadTools;

public class ColoredCircularBlobDetectorBehaviorService extends ThreadedBehaviorService
{
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<>(2);
   private final ConcurrentListeningQueue<RobotConfigurationData> robotConfigurationDataQueue = new ConcurrentListeningQueue<>(2);
   private long videoTimestamp = -1L;

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private final JPEGCompressor jpegCompressor = new JPEGCompressor();

   private final OpenCVColoredCircularBlobDetector openCVColoredCircularBlobDetector;
   private final Point2d latestBallPosition2d = new Point2d();
   private final List<Point2d> latestBallPositionSet = new ArrayList<>();
   private BufferedImage latestUnmodifiedCameraImage;
   private final Object ballListConch = new Object();

   private static final Scalar circleColor = new Scalar(160, 0, 0);

   public ColoredCircularBlobDetectorBehaviorService(CommunicationBridgeInterface communicationBridge)
   {
      super(ColoredCircularBlobDetectorBehaviorService.class.getSimpleName(), communicationBridge);

      getCommunicationBridge().attachNetworkListeningQueue(videoPacketQueue, VideoPacket.class);
      getCommunicationBridge().attachNetworkListeningQueue(robotConfigurationDataQueue, RobotConfigurationData.class);

      OpenCVColoredCircularBlobDetectorFactory factory = new OpenCVColoredCircularBlobDetectorFactory();
      factory.setCaptureSource(OpenCVColoredCircularBlobDetector.CaptureSource.JAVA_BUFFERED_IMAGES);
      openCVColoredCircularBlobDetector = factory.buildBlobDetector();
   }

   @Override
   public void doThreadAction()
   {
      if (videoPacketQueue.isNewPacketAvailable())
      {
         VideoPacket videoPacket = videoPacketQueue.getLatestPacket();
         RobotConfigurationData robotConfigurationData = robotConfigurationDataQueue.getLatestPacket();
         videoTimestamp = robotConfigurationData.getTimestamp();

         latestUnmodifiedCameraImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData());

         openCVColoredCircularBlobDetector.updateFromBufferedImage(latestUnmodifiedCameraImage);
         ArrayList<HoughCircleResult> circles = openCVColoredCircularBlobDetector.getCircles();

         for(int i = 0; i < circles.size(); i++)
         {
            Point2d vecCenter = circles.get(i).getCenter();
            Point openCvCenter = new Point(vecCenter.getX(), vecCenter.getY());
            int circleRadius = (int) circles.get(i).getRadius();
            Imgproc.circle(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR(), openCvCenter, circleRadius, circleColor, 1);
            Imgproc.circle(openCVColoredCircularBlobDetector.getThresholdMat(), openCvCenter, circleRadius, circleColor, 1);
         }

         BufferedImage thresholdBufferedImageOpenCVEncoded = OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getThresholdMat());
         BufferedImage thresholdBufferedImage = OpenCVTools.convertToCompressableBufferedImage(thresholdBufferedImageOpenCVEncoded);

         byte[] jpegThresholdImage = jpegCompressor.convertBufferedImageToJPEGData(thresholdBufferedImage);
         VideoPacket circleBlobThresholdImagePacket = new VideoPacket(VideoSource.CV_THRESHOLD, videoTimestamp, jpegThresholdImage,
                                                                      videoPacket.getPosition(), videoPacket.getOrientation(), videoPacket.getIntrinsicParameters());
         getCommunicationBridge().sendPacket(circleBlobThresholdImagePacket);

         if (circles.size() > 0)
            latestBallPosition2d.set(circles.get(0).getCenter());

         synchronized (ballListConch)
         {
            latestBallPositionSet.clear();
            for (HoughCircleResult houghCircleResult : circles)
            {
               latestBallPositionSet.add(new Point2d(houghCircleResult.getCenter()));
            }
         }
      }
      else
      {
         ThreadTools.sleep(10);
      }
   }

   public void addHSVRange(HSVRange hsvRange)
   {
      openCVColoredCircularBlobDetector.addHSVRange(hsvRange);
   }

   public List<Point2d> getLatestBallPositionSet()
   {
      return latestBallPositionSet;
   }

   public Point2d getLatestBallPosition2d()
   {
      return latestBallPosition2d;
   }

   public BufferedImage getLatestUnmodifiedCameraImage()
   {
      return latestUnmodifiedCameraImage;
   }

   public void clearHSVRanges()
   {
      openCVColoredCircularBlobDetector.resetRanges();
   }

   public Object getBallListConch()
   {
      return ballListConch;
   }

   @Override
   public void initialize()
   {
      // TODO implement me
      
   }
}
