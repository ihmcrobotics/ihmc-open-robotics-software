package us.ihmc.atlas;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.VideoPacket;
import org.bytedeco.javacv.Java2DFrameUtils;
import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ihmcPerception.lineSegmentDetector.LineDetectionTools;
import us.ihmc.ihmcPerception.lineSegmentDetector.LineSegmentMatch;
import us.ihmc.ihmcPerception.lineSegmentDetector.LineSegmentToPlanarRegionAssociator;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.referenceFrames.TransformReferenceFrame;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.ExecutorServiceTools;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.*;
import org.bytedeco.opencv.opencv_imgproc.*;

import static org.bytedeco.opencv.global.opencv_core.*;
import static org.bytedeco.opencv.global.opencv_highgui.*;
import static org.bytedeco.opencv.global.opencv_imgproc.*;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.net.URI;
import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static org.opencv.highgui.HighGui.createJFrame;
import static us.ihmc.ihmcPerception.lineSegmentDetector.LineSegmentMatchingTools.matchLineSegments;

public class AtlasLineSegmentEstimator
{
   private boolean USE_ROS1_PERCEPTION_TOPICS = false;

   private final AtlasRobotModel robotModel;

   private ConcurrentLinkedDeque<CompressedImage> compressedImagesRos1 = new ConcurrentLinkedDeque<>();
   private ConcurrentLinkedDeque<VideoPacket> videoPacketsRos2 = new ConcurrentLinkedDeque<>();

   private volatile CameraInfo cameraInfo = null;
   private volatile PlanarRegionsListMessage currentPlanarRegionsListMessage = null;
   private volatile CameraPinholeBrown cameraIntrinsics;

   private LineSegmentToPlanarRegionAssociator lineRegionAssociator;

   private RosMainNode ros1Node;
   private ROS2Node ros2Node;

   private JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private ScheduledExecutorService executorService
         = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private TransformReferenceFrame realsenseSensorFrame;
   private TransformReferenceFrame multisenseSensorFrame;

   private RemoteSyncedRobotModel syncedRobot;

   public AtlasLineSegmentEstimator()
   {
      // System.out.println(System.getProperty("java.library.path"));
//      System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
      // NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
      new ROS2Callback<>(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, this::planarRegionsListCallback);

      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);
      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);

      if (USE_ROS1_PERCEPTION_TOPICS)
      {
         URI masterURI = NetworkParameters.getROSURI();
         LogTools.info("Connecting to ROS 1 master URI: {}", masterURI);
         RosMainNode ros1Node = new RosMainNode(masterURI, "video_viewer", true);
         ros1Node.attachSubscriber(RosTools.D435_VIDEO, CompressedImage.class, message -> compressedImagesRos1.addLast(message));
         ros1Node.attachSubscriber(RosTools.D435_CAMERA_INFO, CameraInfo.class, info -> cameraInfo = info);
         ros1Node.execute();

         MovingReferenceFrame pelvisFrame = syncedRobot.getReferenceFrames().getPelvisFrame();
         // TODO: Check transform to color camera
         realsenseSensorFrame = new TransformReferenceFrame("Realsense", pelvisFrame, AtlasSensorInformation.transformPelvisToDepthCamera);
         executorService.scheduleAtFixedRate(this::mainUpdateUsingROS1Topics, 0, 32, TimeUnit.MILLISECONDS);
         lineRegionAssociator = new LineSegmentToPlanarRegionAssociator(realsenseSensorFrame);
      }
      else
      {
         new ROS2Callback<>(ros2Node, ROS2Tools.VIDEO, message ->
         {
            videoPacketsRos2.addLast(message);
            cameraIntrinsics = HumanoidMessageTools.toIntrinsicParameters(message.intrinsic_parameters_);
         });

         MovingReferenceFrame neckFrame = syncedRobot.getReferenceFrames().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
//         multisenseSensorFrame = new TransformReferenceFrame("Multisense", neckFrame, AtlasSensorInformation);

         executorService.scheduleAtFixedRate(this::mainUpdateUsingROS2Topics, 0, 32, TimeUnit.MILLISECONDS);
         lineRegionAssociator = new LineSegmentToPlanarRegionAssociator(neckFrame);
      }

   }

   public void planarRegionsListCallback(PlanarRegionsListMessage message)
   {
      currentPlanarRegionsListMessage = message;
   }

   public void mainUpdateUsingROS1Topics()
   {
      LogTools.info(StringTools.format("{} {} {} {}", currentPlanarRegionsListMessage != null,
                                       syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0),
                                       cameraInfo != null,
                                       compressedImagesRos1.size() > 1));
      if (currentPlanarRegionsListMessage != null && syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0) && cameraInfo != null
          && compressedImagesRos1.size() > 1)
      {
         while (compressedImagesRos1.size() > 2)
         {
            compressedImagesRos1.removeFirst();
         }

         CompressedImage latest = compressedImagesRos1.removeFirst();
         CompressedImage previous = compressedImagesRos1.getFirst();

         BufferedImage latestBufferedImage = RosTools.bufferedImageFromRosMessageJpeg(latest);
         BufferedImage previousBufferedImage = RosTools.bufferedImageFromRosMessageJpeg(previous);

         cameraIntrinsics = RosTools.cameraIntrisicsFromCameraInfo(cameraInfo);

         processBufferedImagesAndPlanarRegions(latestBufferedImage, previousBufferedImage);
      }
   }

   public void mainUpdateUsingROS2Topics()
   {
      LogTools.info(StringTools.format("{} {} {} {}", currentPlanarRegionsListMessage != null,
                                       syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0),
                                       cameraIntrinsics != null,
                                       videoPacketsRos2.size() > 1));
      if (currentPlanarRegionsListMessage != null && syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0) && cameraIntrinsics != null
          && videoPacketsRos2.size() > 1)
      {
         while (videoPacketsRos2.size() > 2)
         {
            videoPacketsRos2.removeFirst();
         }

         VideoPacket latest = videoPacketsRos2.removeFirst();
         VideoPacket previous = videoPacketsRos2.getFirst();

         BufferedImage latestBufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(latest.getData().toArray());
         BufferedImage previousBufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(previous.getData().toArray());

         processBufferedImagesAndPlanarRegions(latestBufferedImage, previousBufferedImage);
      }
   }

   private void processBufferedImagesAndPlanarRegions(BufferedImage latestBufferedImage, BufferedImage previousBufferedImage)
   {
      syncedRobot.update();

      float startTime = System.nanoTime();
      System.out.println("Processing Images");

      Mat currentImage = new Mat(latestBufferedImage.getHeight(), latestBufferedImage.getWidth(), CV_8UC3);
      currentImage.data().put(((DataBufferByte) latestBufferedImage.getRaster().getDataBuffer()).getData());
      Mat previousImage = new Mat(previousBufferedImage.getHeight(), previousBufferedImage.getWidth(), CV_8UC3);
      previousImage.data().put(((DataBufferByte) previousBufferedImage.getRaster().getDataBuffer()).getData());

      imshow("Image", currentImage);
      waitKey(1);
      Vec4iVector previousLineSegments = LineDetectionTools.getCannyHoughLinesFromImage(previousImage);
      Vec4iVector currentLineSegments = LineDetectionTools.getCannyHoughLinesFromImage(currentImage);

      lineRegionAssociator.setCurrentLineSegments(currentLineSegments);

      ArrayList<LineSegmentMatch> correspondingLineSegments = matchLineSegments(previousLineSegments, currentLineSegments);

      for (int i = 0; i < currentLineSegments.size(); i++)
      {
         Scalar4i currentLineSegment = currentLineSegments.get(i);
         line(currentImage,
              new Point(currentLineSegment.get(0), currentLineSegment.get(1)),
              new Point(currentLineSegment.get(2), currentLineSegment.get(3)),
              Scalar.YELLOW,
              2,
              LINE_8,
              0);
      }

      // displayLineMatches(prevImg, curImg, disparityImage, correspLines);
      Mat currentImageLeft = currentImage.clone();

      PlanarRegionsList planarRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(currentPlanarRegionsListMessage);
      lineRegionAssociator.loadParams(cameraIntrinsics);

      for (PlanarRegion region : planarRegions.getPlanarRegionsAsList())
      {
         Point2D regionMidPoint = new Point2D(0, 0);

         if (region.getConvexHull().getArea() * 1000 > 50)
         {
            ArrayList<Point> pointList = lineRegionAssociator.projectPlanarRegion(region, regionMidPoint);
            lineRegionAssociator.drawPolygonOnImage(currentImage, pointList, regionMidPoint, region.getRegionId());
            lineRegionAssociator.drawLineRegionAssociation(currentImageLeft, pointList, regionMidPoint, region.getRegionId());
         }
      }

      MatVector src = new MatVector(currentImageLeft, currentImage);
      Mat disparityImage = new Mat();
      hconcat(src, disparityImage);

      float endTime = System.nanoTime();
      System.out.println("Time:" + (int) ((endTime - startTime) * (1e-6)) + " ms\tCorresponding Lines:" + correspondingLineSegments.size());
      resize(disparityImage, disparityImage, new Size(2400, 1000));
      namedWindow("LineEstimator", WINDOW_AUTOSIZE);
      imshow("LineEstimator", disparityImage);
      waitKey(1);
   }

   public static void main(String[] args)
   {
//      namedWindow("LineEstimator", WINDOW_AUTOSIZE);
//      createJFrame("Hello", 0);
      new AtlasLineSegmentEstimator();
   }
}
