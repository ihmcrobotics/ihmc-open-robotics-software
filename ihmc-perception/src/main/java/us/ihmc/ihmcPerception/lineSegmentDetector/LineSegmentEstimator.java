package us.ihmc.ihmcPerception.lineSegmentDetector;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepPlanningToolboxOutputStatus;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.VideoPacket;
import javafx.scene.shape.Sphere;
import org.opencv.core.*;
import org.opencv.highgui.HighGui;
import org.opencv.imgproc.Imgproc;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.communication.producers.JPEGDecompressor;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.footstepPlanning.log.FootstepPlannerLog;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.ros2.ROS2Callback;
import us.ihmc.ros2.ROS2Node;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static us.ihmc.ihmcPerception.lineSegmentDetector.LineMatchingTools.matchFLDLines;

public class LineSegmentEstimator
{
   private boolean prevImgFilled = false;
   private boolean currentVideoPacketFilled = false;
   private boolean currentPlanarRegionsListMessageFilled = false;
   private int code = 0;

   private VideoPacket currentVideoPacket;
   private PlanarRegionsListMessage currentPlanarRegionsListMessage;

   private Sphere sensorNode;

   private Mat curImg;
   private Mat prevImg;
   private Mat curLines;
   private Mat prevLines;
   private ArrayList<LineMatch> correspLines;

   private CameraPinholeBrown camIntrinsics;

   private LineSegmentToPlanarRegionAssociator lineRegionAssoc;
   // private JavaFXPlanarRegionsViewer planarRegionsViewer;
   private ROS2Node ros2Node;
   private JPEGDecompressor jpegDecompressor = new JPEGDecompressor();
   private ScheduledExecutorService executorService
         = ExecutorServiceTools.newScheduledThreadPool(3, getClass(), ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   public LineSegmentEstimator()
   {
      // System.out.println(System.getProperty("java.library.path"));
      System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
      // NativeLibraryLoader.loadLibrary("org.opencv", OpenCVTools.OPEN_CV_LIBRARY_NAME);
      this.ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");
      this.registerCallbacks(ros2Node);

      executorService.scheduleAtFixedRate(this::mainUpdate, 0, 32, TimeUnit.MILLISECONDS);
      lineRegionAssoc = new LineSegmentToPlanarRegionAssociator();

      // execFootstepPlan();
   }

   // public void setPlanarRegionsViewer(JavaFXPlanarRegionsViewer planarRegionsViewer) {
   //     this.planarRegionsViewer = planarRegionsViewer;
   // }

   public void setSensorNode(Sphere sensorNode)
   {
      this.sensorNode = sensorNode;
   }

   public void execFootstepPlan()
   {
      FootstepPlannerLogLoader logLoader = new FootstepPlannerLogLoader();
      FootstepPlannerLogLoader.LoadResult loadresult = logLoader.load(new File("/home/quantum/.ihmc/logs/20200821_172044151_FootstepPlannerLog/"));
      if (!(loadresult == FootstepPlannerLogLoader.LoadResult.LOADED))
      {
         LogTools.error("Couldn't find file");
         return;
      }
      FootstepPlannerLog log = logLoader.getLog();
      FootstepPlanningToolboxOutputStatus statusPacket = log.getStatusPacket();
      FootstepDataListMessage footstepDataList = statusPacket.getFootstepDataList();

      String robotName = "Atlas";

      IHMCROS2Publisher<FootstepDataListMessage> footstepDataListPublisher = ROS2Tools.createPublisherTypeNamed(this.ros2Node,
                                                                                                                FootstepDataListMessage.class,
                                                                                                                ROS2Tools.getControllerInputTopic(robotName));
      footstepDataListPublisher.publish(footstepDataList);
   }

   public void videoPacketCallback(VideoPacket message)
   {
      currentVideoPacketFilled = true;
      currentVideoPacket = message;
   }

   public void planarRegionsListCallback(PlanarRegionsListMessage message)
   {
      currentPlanarRegionsListMessageFilled = true;
      currentPlanarRegionsListMessage = message;
   }

   public void registerCallbacks(ROS2Node ros2Node)
   {
      new ROS2Callback<>(ros2Node, PlanarRegionsListMessage.class, ROS2Tools.REA.withOutput(), this::planarRegionsListCallback);
      new ROS2Callback<>(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT, this::videoPacketCallback);
   }

   public void mainUpdate()
   {

      if (currentVideoPacketFilled)
      {
         BufferedImage bufferedImage = jpegDecompressor.decompressJPEGDataToBufferedImage(this.currentVideoPacket.getData().toArray());
         Mat mat = new Mat(bufferedImage.getHeight(), bufferedImage.getWidth(), CvType.CV_8UC3);
         byte[] data = ((DataBufferByte) bufferedImage.getRaster().getDataBuffer()).getData();
         mat.put(0, 0, data);
         this.curImg = mat;
         if (!(this.prevImgFilled))
         {
            prevImgFilled = true;
            this.prevImg = this.curImg;
            this.prevLines = LineDetectionTools.getFLDLinesFromImage(this.prevImg);
            camIntrinsics = HumanoidMessageTools.toIntrinsicParameters(this.currentVideoPacket.intrinsic_parameters_);

            currentVideoPacketFilled = false;
         }
         else
         {
            this.processImages();
            this.prevImgFilled = true;
            this.prevImg = this.curImg;
            this.prevLines = this.curLines;
         }
      }

      if (currentPlanarRegionsListMessageFilled)
      {
         currentPlanarRegionsListMessageFilled = false;
      }
      if (this.code == 81)
         System.exit(0);
   }

   public void processImages()
   {

      float startTime = System.nanoTime();

      Mat dispImage = new Mat();
      this.curLines = LineDetectionTools.getFLDLinesFromImage(this.curImg);
      lineRegionAssoc.setCurLines(this.curLines);

      this.correspLines = matchFLDLines(this.prevLines, this.curLines);

      for (int i = 0; i < curLines.rows(); i++)
      {
         double[] val = curLines.get(i, 0);
         Imgproc.line(this.curImg, new Point(val[0], val[1]), new Point(val[2], val[3]), new Scalar(0, 255, 255), 2);
      }

      // displayLineMatches(prevImg, curImg, dispImage, correspLines);
      Mat curImgLeft = new Mat();
      curImg.copyTo(curImgLeft);

      if (this.currentPlanarRegionsListMessage != null)
      {
         PlanarRegionsList newRegions = PlanarRegionMessageConverter.convertToPlanarRegionsList(this.currentPlanarRegionsListMessage);
         lineRegionAssoc.loadParams(camIntrinsics, currentVideoPacket.orientation_, currentVideoPacket.position_);

         for (PlanarRegion region : newRegions.getPlanarRegionsAsList())
         {

            ArrayList<Point> pointList = new ArrayList<>();
            Point2D regionMidPoint = new Point2D(0, 0);

            if (region.getConvexHull().getArea() * 1000 > 50)
            {

               pointList = lineRegionAssoc.projectPlanarRegion(region, regionMidPoint);
               lineRegionAssoc.drawPolygonOnImage(curImg, pointList, regionMidPoint, region.getRegionId());
               lineRegionAssoc.drawLineRegionAssociation(curImgLeft, pointList, regionMidPoint, region.getRegionId());
            }
         }
      }

      List<Mat> src = Arrays.asList(curImgLeft, curImg);
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
      new LineSegmentEstimator();
   }
}
