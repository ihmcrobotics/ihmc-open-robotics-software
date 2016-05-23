package us.ihmc.humanoidBehaviors.behaviors;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.SdfLoader.SDFFullHumanoidRobotModel;
import us.ihmc.communication.producers.*;
import us.ihmc.humanoidBehaviors.communication.ConcurrentListeningQueue;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.sensing.VideoPacket;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.OpenCVTools;
import us.ihmc.ihmcPerception.vision.shapes.HSVRange;
import us.ihmc.ihmcPerception.vision.shapes.HoughCircleResult;
import us.ihmc.ihmcPerception.vision.shapes.OpenCVColoredCircularBlobDetector;
import us.ihmc.ihmcPerception.vision.shapes.OpenCVColoredCircularBlobDetectorFactory;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.communication.packets.dataobjects.RobotConfigurationData;
import us.ihmc.tools.thread.ThreadTools;

import javax.vecmath.Point2d;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import java.awt.image.BufferedImage;
import java.util.ArrayList;
import java.util.List;

public class BlobFilteredSphereDetectionBehavior extends SphereDetectionBehavior implements VideoStreamer
{
   private static final boolean DEBUG = true;

   private final ReferenceFrame headFrame;
   private static final double FILTERING_ANGLE = Math.toRadians(5.0);
   private static final double FILTERING_MIN_DISTANCE = 0.1;
   private static final double FILTERING_MAX_DISTANCE = 7.0;

   // http://www.bostondynamics.com/img/MultiSense_SL.pdf
   private static final double HORIZONTAL_FOV = Math.toRadians(80.0);
   private static final double VERTICAL_FOV = Math.toRadians(45.0);

   private final VideoClientThread blobDetectionThread = new VideoClientThread();
   private BufferedImage lastBufferedImage;
   private final JPEGCompressor jpegCompressor = new JPEGCompressor();
   private final Point2d latestBallPosition2d = new Point2d();
   private long videoTimestamp = -1L;
   private final BooleanYoVariable runBlobFilter = new BooleanYoVariable("runBlobFilter", registry);
   private final ConcurrentListeningQueue<VideoPacket> videoPacketQueue = new ConcurrentListeningQueue<>();
   private final ConcurrentListeningQueue<RobotConfigurationData> robotConfigurationDataQueue = new ConcurrentListeningQueue<>();
   private final CompressedVideoDataClient videoDataClient;
   private int numBallsDetected;

   private final OpenCVColoredCircularBlobDetector openCVColoredCircularBlobDetector;

   public BlobFilteredSphereDetectionBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, HumanoidReferenceFrames referenceFrames,
         SDFFullHumanoidRobotModel fullRobotModel)
   {
      super(outgoingCommunicationBridge, referenceFrames);

      OpenCVColoredCircularBlobDetectorFactory factory = new OpenCVColoredCircularBlobDetectorFactory();
      factory.setCaptureSource(OpenCVColoredCircularBlobDetector.CaptureSource.JAVA_BUFFERED_IMAGES);
      openCVColoredCircularBlobDetector = factory.buildBlobDetector();

      videoDataClient = CompressedVideoDataFactory.createCompressedVideoDataClient(this);
      runBlobFilter.set(false);
      this.headFrame = fullRobotModel.getHead().getBodyFixedFrame();
      blobDetectionThread.run();

   }

   public void addHSVRange(HSVRange hsvRange)
   {
      openCVColoredCircularBlobDetector.addHSVRange(hsvRange);
   }

   public void resetHSVRanges()
   {
      openCVColoredCircularBlobDetector.resetRanges();
   }

   @Override public void updateImage(BufferedImage bufferedImage, Point3d cameraPosition, Quat4d cameraOrientation, IntrinsicParameters intrinsicParamaters)
   {
      this.lastBufferedImage = bufferedImage;

      if(runBlobFilter.getBooleanValue())
      {
         openCVColoredCircularBlobDetector.updateFromBufferedImage(bufferedImage);
         ArrayList<HoughCircleResult> circles = openCVColoredCircularBlobDetector.getCircles();

         BufferedImage thresholdBufferedImage = OpenCVTools.convertMatToBufferedImage(openCVColoredCircularBlobDetector.getThresholdMat());
         byte[] jpegThresholdImage = jpegCompressor.convertBufferedImageToJPEGData(thresholdBufferedImage);
         VideoPacket circleBlobThresholdImagePacket = new VideoPacket(RobotSide.LEFT, VideoSource.MULTISENSE, videoTimestamp, jpegThresholdImage, cameraPosition, cameraOrientation,
               intrinsicParamaters);
         sendPacketToNetworkProcessor(circleBlobThresholdImagePacket);

         if (DEBUG)
         {
            System.out.println("blob detection found " + circles.size() + " circles");

            for (int i = 0; i < circles.size(); i++)
            {
               HoughCircleResult result = circles.get(i);
               System.out.println("\t center: " + result.getCenter());
            }
         }

         this.numBallsDetected = circles.size();

         if (numBallsDetected > 0)
            latestBallPosition2d.set(circles.get(0).getCenter());
      }
   }

   private class VideoClientThread extends Thread
   {
      @Override public void run()
      {
         while (true)
         {
            if (videoPacketQueue.isNewPacketAvailable())
            {
               VideoPacket packet = videoPacketQueue.getLatestPacket();
               RobotConfigurationData robotConfigurationData = robotConfigurationDataQueue.getLatestPacket();
               videoTimestamp = robotConfigurationData.getTimestamp();

               videoDataClient.consumeObject(packet.getData(), packet.getPosition(), packet.getOrientation(), packet.getIntrinsicParameters());
            }
            else
            {
               ThreadTools.sleep(10);
            }
         }
      }
   }

   @Override
   public void doControl()
   {
      while ((pointCloudPacket = pointCloudQueue.poll()) != null)
      {
         pointCloudPacketLatest = pointCloudPacket;
      }

      if (pointCloudPacketLatest != null)
      {
         Point3f[] fullPointCloud = pointCloudPacketLatest.getDecayingWorldScan();

         if(runBlobFilter.getBooleanValue())
         {
            Point3f[] filteredPointCloud = filterPointsNearBall(fullPointCloud);
            findBallsAndSaveResult(filteredPointCloud);
         }
         else
         {
            findBallsAndSaveResult(fullPointCloud);
         }
      }
   }

   private Point3f[] filterPointsNearBall(Point3f[] fullPointCloud)
   {
      List<Point3f> filteredPoints = new ArrayList<Point3f>();
      RigidBodyTransform worldToCameraTransform = headFrame.getTransformToWorldFrame();
      worldToCameraTransform.invert();

      int cameraPixelWidth = lastBufferedImage.getWidth();
      int cameraPixelHeight = lastBufferedImage.getHeight();

      double ballCenterX = latestBallPosition2d.x - lastBufferedImage.getMinX();
      double ballCenterY = latestBallPosition2d.y - lastBufferedImage.getMinY();

      double desiredRayAngleX = VERTICAL_FOV * (-ballCenterY / cameraPixelHeight + 0.5);
      double desiredRayAngleY = HORIZONTAL_FOV * (ballCenterX / cameraPixelWidth - 0.5);

      Point3f tempPoint = new Point3f();

      for (int i = 0; i < fullPointCloud.length; i++)
      {
         tempPoint.set(fullPointCloud[i]);
         worldToCameraTransform.transform(tempPoint);

         if (tempPoint.x > FILTERING_MIN_DISTANCE && tempPoint.x < FILTERING_MAX_DISTANCE)
         {
            // rayAngle axes are in terms of the buffered image (y-down), temp pnt axes are in terms of camera frame (z-up)
            double rayAngleX = Math.atan2(tempPoint.z, tempPoint.x);
            double rayAngleY = Math.atan2(tempPoint.y, tempPoint.x);
            if (Math.abs(rayAngleX - desiredRayAngleX) < FILTERING_ANGLE && Math.abs(rayAngleY - desiredRayAngleY) < FILTERING_ANGLE)
            {
               filteredPoints.add(fullPointCloud[i]);
            }
         }
      }

      Point3f[] filteredPointArray = new Point3f[filteredPoints.size()];
      filteredPoints.toArray(filteredPointArray);
      return filteredPointArray;
   }


   @Override public void initialize()
   {
      super.initialize();
      runBlobFilter.set(true);
   }

   @Override
   public boolean isDone()
   {
      return super.isDone();
   }

   @Override public void pause()
   {
      super.pause();
      runBlobFilter.set(false);
   }

   @Override public void stop()
   {
      super.stop();
      runBlobFilter.set(false);
   }

   @Override public void resume()
   {
      super.resume();
      runBlobFilter.set(true);
   }

   public Point2d getLatestBallPosition()
   {
      return latestBallPosition2d;
   }

   public int getNumBallsDetected()
   {
      return numBallsDetected;
   }
}
