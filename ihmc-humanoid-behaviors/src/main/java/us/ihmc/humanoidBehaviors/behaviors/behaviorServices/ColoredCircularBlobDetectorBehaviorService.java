package us.ihmc.humanoidBehaviors.behaviors.behaviorServices;

import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

import org.opencv.core.Scalar;

import controller_msgs.msg.dds.RobotConfigurationData;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.JPEGCompressor;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.ros2.ROS2Node;

public class ColoredCircularBlobDetectorBehaviorService extends ThreadedBehaviorService
{
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<>(2);
   private final ConcurrentListeningQueue<RobotConfigurationData> robotConfigurationDataQueue = new ConcurrentListeningQueue<>(2);
   private long videoTimestamp = -1L;

   private final JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private final JPEGCompressor jpegCompressor = new JPEGCompressor();

//   private final OpenCVColoredCircularBlobDetector openCVColoredCircularBlobDetector;
   private final Point2D latestBallPosition2d = new Point2D();
   private final List<Point2D> latestBallPositionSet = new ArrayList<>();
   private BufferedImage latestUnmodifiedCameraImage;
   private final Object ballListConch = new Object();

   private static final Scalar circleColor = new Scalar(160, 0, 0);
   private final ROS2PublisherBasics<VideoPacket> videoPublisher;

   public ColoredCircularBlobDetectorBehaviorService(String robotName, ROS2Node ros2Node)
   {
      super(robotName, ColoredCircularBlobDetectorBehaviorService.class.getSimpleName(), ros2Node);

      createSubscriber(VideoPacket.class, ROS2Tools.IHMC_ROOT, videoPacketQueue::put); // FIXME Need to figure out the topic name for video streams
      createSubscriberFromController(RobotConfigurationData.class, robotConfigurationDataQueue::put);

      videoPublisher = createBehaviorOutputPublisher(VideoPacket.class, "/video");

//      OpenCVColoredCircularBlobDetectorFactory factory = new OpenCVColoredCircularBlobDetectorFactory();
//      factory.setCaptureSource(OpenCVColoredCircularBlobDetector.CaptureSource.JAVA_BUFFERED_IMAGES);
//      openCVColoredCircularBlobDetector = factory.buildBlobDetector();
   }

   @Override
   public void doThreadAction()
   {
      if (videoPacketQueue.isNewPacketAvailable())
      {
         VideoPacket videoPacket = videoPacketQueue.getLatestPacket();
         RobotConfigurationData robotConfigurationData = robotConfigurationDataQueue.getLatestPacket();
         videoTimestamp = robotConfigurationData.getMonotonicTime();

         latestUnmodifiedCameraImage = jpegDecompressor.decompressJPEGDataToBufferedImage(videoPacket.getData().toArray());

//         openCVColoredCircularBlobDetector.updateFromBufferedImage(latestUnmodifiedCameraImage);
//         ArrayList<HoughCircleResult> circles = openCVColoredCircularBlobDetector.getCircles();
//
//         for (int i = 0; i < circles.size(); i++)
//         {
//            Point2D vecCenter = circles.get(i).getCenter();
//            Point openCvCenter = new Point(vecCenter.getX(), vecCenter.getY());
//            int circleRadius = (int) circles.get(i).getRadius();
//            Imgproc.circle(openCVColoredCircularBlobDetector.getCurrentCameraFrameMatInBGR(), openCvCenter, circleRadius, circleColor, 1);
//            Imgproc.circle(openCVColoredCircularBlobDetector.getThresholdMat(), openCvCenter, circleRadius, circleColor, 1);
//         }
//
//         BufferedImage thresholdBufferedImageOpenCVEncoded = OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getThresholdMat());
//         BufferedImage thresholdBufferedImage = OpenCVTools.convertToCompressableBufferedImage(thresholdBufferedImageOpenCVEncoded);

//         byte[] jpegThresholdImage = jpegCompressor.convertBufferedImageToJPEGData(thresholdBufferedImage);
//         VideoPacket circleBlobThresholdImagePacket = HumanoidMessageTools.createVideoPacket(VideoSource.CV_THRESHOLD, videoTimestamp, jpegThresholdImage,
//                                                                                             videoPacket.getPosition(), videoPacket.getOrientation(),
//                                                                                             HumanoidMessageTools.toIntrinsicParameters(videoPacket.getIntrinsicParameters()));
//         videoPublisher.publish(circleBlobThresholdImagePacket);

//         if (circles.size() > 0)
//            latestBallPosition2d.set(circles.get(0).getCenter());
//
//         synchronized (ballListConch)
//         {
//            latestBallPositionSet.clear();
//            for (HoughCircleResult houghCircleResult : circles)
//            {
//               latestBallPositionSet.add(new Point2D(houghCircleResult.getCenter()));
//            }
//         }
      }
      else
      {
         ThreadTools.sleep(10);
      }
   }

//   public void addHSVRange(HSVRange hsvRange)
//   {
//      openCVColoredCircularBlobDetector.addHSVRange(hsvRange);
//   }

   public List<Point2D> getLatestBallPositionSet()
   {
      return latestBallPositionSet;
   }

   public Point2D getLatestBallPosition2d()
   {
      return latestBallPosition2d;
   }

   public BufferedImage getLatestUnmodifiedCameraImage()
   {
      return latestUnmodifiedCameraImage;
   }

   public void clearHSVRanges()
   {
//      openCVColoredCircularBlobDetector.resetRanges();
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
