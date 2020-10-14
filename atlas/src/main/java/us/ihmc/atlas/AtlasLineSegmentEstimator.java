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
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.humanoidBehaviors.lookAndStep.BehaviorTaskSuppressor;
import us.ihmc.ihmcPerception.lineSegmentDetector.LineDetectionTools;
import us.ihmc.ihmcPerception.lineSegmentDetector.LineMatch;
import us.ihmc.ihmcPerception.lineSegmentDetector.LineSegmentToPlanarRegionAssociator;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
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
   private final AtlasRobotModel robotModel;
   private boolean prevImgFilled = false;
   private int code = 0;

   private ConcurrentLinkedDeque<CompressedImage> images = new ConcurrentLinkedDeque<>();
   private volatile CameraInfo cameraInfo = null;
   private volatile PlanarRegionsListMessage currentPlanarRegionsListMessage = null;

   private Sphere sensorNode;

   private Mat currentImage;
   private Mat previousImage;
   private Mat curLines;
   private Mat previousLines;
   private ArrayList<LineMatch> correspLines;

   private CameraPinholeBrown cameraIntrinsics;

   private LineSegmentToPlanarRegionAssociator lineRegionAssoc;
   // private JavaFXPlanarRegionsViewer planarRegionsViewer;
   private RosMainNode ros1Node;
   private ROS2Node ros2Node;
   private JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private ScheduledExecutorService executorService
         = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   private TransformReferenceFrame realsenseSensorFrame;
   private RemoteSyncedRobotModel syncedRobot;

   public AtlasLineSegmentEstimator()
   {
      // System.out.println(System.getProperty("java.library.path"));
//      System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
      // NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
      this.ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");

      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS);

      URI masterURI = NetworkParameters.getROSURI();
      LogTools.info("Connecting to ROS 1 master URI: {}", masterURI);
      RosMainNode ros1Node = new RosMainNode(masterURI, "video_viewer", true);

      new ROS2Callback<>(ros2Node, ROS2Tools.REALSENSE_SLAM_REGIONS, this::planarRegionsListCallback);
//      new ROS2Callback<>(ros2Node, ROS2Tools.VIDEO, this::videoPacketCallback);

      ros1Node.attachSubscriber(RosTools.D435_VIDEO, CompressedImage.class, message -> images.addLast(message));
      ros1Node.attachSubscriber(RosTools.D435_CAMERA_INFO, CameraInfo.class, info -> cameraInfo = info);

      syncedRobot = new RemoteSyncedRobotModel(robotModel, ros2Node);
      MovingReferenceFrame pelvisFrame = syncedRobot.getReferenceFrames().getPelvisFrame();
      // TODO: Check transform to color camera
      realsenseSensorFrame = new TransformReferenceFrame("Realsense", pelvisFrame, AtlasSensorInformation.transformPelvisToDepthCamera);

      ros1Node.execute();

      executorService.scheduleAtFixedRate(this::mainUpdate, 0, 32, TimeUnit.MILLISECONDS);
      lineRegionAssoc = new LineSegmentToPlanarRegionAssociator(realsenseSensorFrame);
   }

   public void setSensorNode(Sphere sensorNode)
   {
      this.sensorNode = sensorNode;
   }

//   public void videoPacketCallback(VideoPacket message)
//   {
//      currentVideoPacket = message;
//   }

   public void planarRegionsListCallback(PlanarRegionsListMessage message)
   {
//      currentPlanarRegionsListMessageFilled = true;
      currentPlanarRegionsListMessage = message;
   }

   public void mainUpdate()
   {
      LogTools.info(StringTools.format("{} {} {} {}", currentPlanarRegionsListMessage != null,
                                       syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0),
                                       cameraInfo != null,
                                       images.size() > 1));
      if (currentPlanarRegionsListMessage != null
          && syncedRobot.getDataReceptionTimerSnapshot().isRunning(1.0)
          && cameraInfo != null
          && images.size() > 1)
      {
         while (images.size() > 2)
         {
            images.removeFirst();
         }

         CompressedImage latest = images.removeFirst();
         CompressedImage previous = images.getFirst();

         BufferedImage latestBufferedImage = RosTools.bufferedImageFromRosMessageJpeg(latest);
         BufferedImage previousBufferedImage = RosTools.bufferedImageFromRosMessageJpeg(previous);

         currentImage = new Mat(latestBufferedImage.getHeight(), latestBufferedImage.getWidth(), CvType.CV_8UC3);
         byte[] data = ((DataBufferByte) latestBufferedImage.getRaster().getDataBuffer()).getData();
         currentImage.put(0, 0, data);

         previousImage = new Mat(previousBufferedImage.getHeight(), previousBufferedImage.getWidth(), CvType.CV_8UC3);
         byte[] previousData = ((DataBufferByte) previousBufferedImage.getRaster().getDataBuffer()).getData();
         previousImage.put(0, 0, previousData);

         previousLines = LineDetectionTools.getFLDLinesFromImage(previousImage);
         cameraIntrinsics = RosTools.cameraIntrisicsFromCameraInfo(cameraInfo);

         syncedRobot.update();

         processImages();
      }

      if (code == 81)
         System.exit(0);
   }

   public void processImages()
   {
      float startTime = System.nanoTime();

      Mat dispImage = new Mat();
      this.curLines = LineDetectionTools.getFLDLinesFromImage(this.currentImage);
      lineRegionAssoc.setCurLines(this.curLines);

      this.correspLines = matchFLDLines(this.previousLines, this.curLines);

      for (int i = 0; i < curLines.rows(); i++)
      {
         double[] val = curLines.get(i, 0);
         Imgproc.line(this.currentImage, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(0, 255, 255), 2);
      }

      // displayLineMatches(prevImg, curImg, dispImage, correspLines);
      Mat curImgLeft = new Mat();
      currentImage.copyTo(curImgLeft);

      if (this.currentPlanarRegionsListMessage != null)
      {
         PlanarRegionsList newRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(this.currentPlanarRegionsListMessage);
         lineRegionAssoc.loadParams(cameraIntrinsics);

         for (PlanarRegion region : newRegions.getPlanarRegionsAsList())
         {

            ArrayList<Point> pointList = new ArrayList<>();
            Point2D regionMidPoint = new Point2D(0, 0);

            if (region.getConvexHull().getArea() * 1000 > 50)
            {
               pointList = lineRegionAssoc.projectPlanarRegion(region, regionMidPoint);
               lineRegionAssoc.drawPolygonOnImage(currentImage, pointList, regionMidPoint, region.getRegionId());
               lineRegionAssoc.drawLineRegionAssociation(curImgLeft, pointList, regionMidPoint, region.getRegionId());
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
      this.code = HighGui.waitKey(1);

      if (this.code == 81)
         System.exit(0);
   }

   public static void main(String[] args)
   {
      HighGui.namedWindow("LineEstimator", HighGui.WINDOW_AUTOSIZE);
      HighGui.createJFrame("Hello", 0);
      new AtlasLineSegmentEstimator();
   }
}
