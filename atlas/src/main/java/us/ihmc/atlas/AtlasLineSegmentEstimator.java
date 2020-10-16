package us.ihmc.atlas;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.VideoPacket;
import javafx.scene.shape.Sphere;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import sensor_msgs.CameraInfo;
import sensor_msgs.CompressedImage;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ihmcPerception.lineSegmentDetector.LineDetectionTools;
import us.ihmc.ihmcPerception.lineSegmentDetector.LineMatch;
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

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.net.URI;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ConcurrentLinkedDeque;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static us.ihmc.ihmcPerception.lineSegmentDetector.LineMatchingTools.matchFLDLines;

public class AtlasLineSegmentEstimator
{

   private boolean USE_ROS1_PERCEPTION_TOPICS = false;

   private final AtlasRobotModel robotModel;
   private boolean prevImgFilled = false;
   private int code = 0;

   private ConcurrentLinkedDeque<CompressedImage> compressedImagesRos1 = new ConcurrentLinkedDeque<>();
   private ConcurrentLinkedDeque<VideoPacket> videoPacketsRos2 = new ConcurrentLinkedDeque<>();

   private volatile CameraInfo cameraInfo = null;
   private volatile PlanarRegionsListMessage currentPlanarRegionsListMessage = null;
   private volatile CameraPinholeBrown cameraIntrinsics;

   private Sphere sensorNode;

   private Mat currentImage;
   private Mat previousImage;
   private Mat currentLines;
   private Mat previousLines;
   private ArrayList<LineMatch> correspondingLines;


   private LineSegmentToPlanarRegionAssociator lineRegionAssociator;

   // private JavaFXPlanarRegionsViewer planarRegionsViewer;

   private RosMainNode ros1Node;
   private ROS2Node ros2Node;

   private JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private ScheduledExecutorService executorService
         = ExecutorServiceTools.newScheduledThreadPool(1, getClass(), ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private TransformReferenceFrame realsenseSensorFrame;
   private TransformReferenceFrame multisenseSensorFrame;
   private MovingReferenceFrame neckFrame;

   private RemoteSyncedRobotModel syncedRobot;

   public AtlasLineSegmentEstimator()
   {
      // System.out.println(System.getProperty("java.library.path"));
      System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
      // NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");




      new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REA.withOutput(), this::planarRegionsListCallback); // (AtlasObstacleCourseNoUI)
//      new ROS2Callback<>(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, this::planarRegionsListCallback); // (AtlasLookAndStepBehaviorDemo)

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
         // VideoPacket.class, ROS2Tools.IHMC_ROOT (For All)
         // ROS2Tools.VIDEO (AtlasLookAndStepBehaviorDemo)
         lineRegionAssociator = new LineSegmentToPlanarRegionAssociator();
         new ROS2Callback<>(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT, message ->
         {
            LogTools.info("Message Received: ", message);
            videoPacketsRos2.addLast(message);
            cameraIntrinsics = HumanoidMessageTools.toIntrinsicParameters(message.intrinsic_parameters_);
            lineRegionAssociator.setIntrinsics(cameraIntrinsics);
            lineRegionAssociator.setCameraOrientation(message.orientation_);
            lineRegionAssociator.setCameraPosition(message.position_);
         });

         neckFrame = syncedRobot.getReferenceFrames().getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
         //         multisenseSensorFrame = new TransformReferenceFrame("Multisense", neckFrame, AtlasSensorInformation);

         executorService.scheduleAtFixedRate(this::mainUpdateUsingROS2Topics, 0, 32, TimeUnit.MILLISECONDS);
      }

   }

   public void setSensorNode(Sphere sensorNode)
   {
      this.sensorNode = sensorNode;
   }

   public void planarRegionsListCallback(PlanarRegionsListMessage message)
   {
      currentPlanarRegionsListMessage = message;
   }

   public void mainUpdateUsingROS1Topics()
   {
//      LogTools.info(StringTools.format("{} {} {} {}", currentPlanarRegionsListMessage != null,
//                                       syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0),
//                                       cameraInfo != null,
//                                       compressedImagesRos1.size() > 1));

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

         neckFrame.update();
         LogTools.info("NeckFrame: {}", neckFrame.getTransformToWorldFrame());

         VideoPacket latest = videoPacketsRos2.removeFirst();
         VideoPacket previous = videoPacketsRos2.getFirst();

         BufferedImage latestBufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(latest.getData().toArray());
         BufferedImage previousBufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(previous.getData().toArray());

         processBufferedImagesAndPlanarRegions(latestBufferedImage, previousBufferedImage);
      }
   }

   private void processBufferedImagesAndPlanarRegions(BufferedImage latestBufferedImage, BufferedImage previousBufferedImage)
   {
      currentImage = new Mat(latestBufferedImage.getHeight(), latestBufferedImage.getWidth(), CvType.CV_8UC3);
      byte[] data = ((DataBufferByte) latestBufferedImage.getRaster().getDataBuffer()).getData();
      currentImage.put(0, 0, data);

      previousImage = new Mat(previousBufferedImage.getHeight(), previousBufferedImage.getWidth(), CvType.CV_8UC3);
      byte[] previousData = ((DataBufferByte) previousBufferedImage.getRaster().getDataBuffer()).getData();
      previousImage.put(0, 0, previousData);

      previousLines = LineDetectionTools.getFLDLinesFromImage(previousImage);


      syncedRobot.update();

      processImages();
   }

   public void processImages()
   {
      float startTime = System.nanoTime();


      Mat dispImage = new Mat();
      this.currentLines = LineDetectionTools.getFLDLinesFromImage(this.currentImage);
      lineRegionAssociator.setCurLines(this.currentLines);

      this.correspondingLines = matchFLDLines(this.previousLines, this.currentLines);

      for (int i = 0; i < currentLines.rows(); i++)
      {
         double[] val = currentLines.get(i, 0);
         Imgproc.line(this.currentImage, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(0, 255, 255), 2);
      }

      // displayLineMatches(prevImg, curImg, dispImage, correspLines);
      Mat curImgLeft = new Mat();
      currentImage.copyTo(curImgLeft);

      if (this.currentPlanarRegionsListMessage != null)
      {
         PlanarRegionsList newRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(this.currentPlanarRegionsListMessage);

         for (PlanarRegion region : newRegions.getPlanarRegionsAsList())
         {
            ArrayList<Point> pointList = new ArrayList<>();
            Point2D regionMidPoint = new Point2D(0, 0);

            if (region.getConvexHull().getArea() * 1000 > 50)
            {
               pointList = lineRegionAssociator.projectPlanarRegionUsingCameraPose(region, regionMidPoint);
               lineRegionAssociator.drawPolygonOnImage(currentImage, pointList, regionMidPoint, region.getRegionId());
               lineRegionAssociator.drawLineRegionAssociation(curImgLeft, pointList, regionMidPoint, region.getRegionId());
            }
         }
      }

      List<Mat> src = Arrays.asList(curImgLeft, currentImage);
      Core.hconcat(src, dispImage);

      float endTime = System.nanoTime();
      // System.out.println("Time:" + (int) ((endTime - startTime) * (1e-6)) + " ms\tCorresponding Lines:" + correspLines.size());
      Imgproc.resize(dispImage, dispImage, new Size(2400, 1000));
      HighGui.namedWindow("LineEstimator", HighGui.WINDOW_AUTOSIZE);
      HighGui.imshow("LineEstimator", dispImage);
      HighGui.waitKey(1);

      previousLines = currentLines;
   }

   public static void main(String[] args)
   {
//      new AtlasLineDetectionDemo().execFootstepPlan();
      new AtlasLineSegmentEstimator();
   }
}
