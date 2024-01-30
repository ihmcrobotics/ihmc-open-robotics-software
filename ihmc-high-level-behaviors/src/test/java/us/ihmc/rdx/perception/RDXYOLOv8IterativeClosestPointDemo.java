package us.ihmc.rdx.perception;

import boofcv.struct.flow.ImageFlow;
import com.badlogic.gdx.graphics.Color;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D32;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.*;
import us.ihmc.perception.YOLOv8.YOLOv8DetectableObject;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.iterativeClosestPoint.IterativeClosestPointTools;
import us.ihmc.perception.iterativeClosestPoint.IterativeClosestPointWorker;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.RDXReferenceFrameGraphic;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;
import us.ihmc.tools.thread.Throttler;

import java.util.Collections;
import java.util.Comparator;
import java.util.List;
import java.util.Random;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public class RDXYOLOv8IterativeClosestPointDemo
{
   private static final float CONFIDENCE_THRESHOLD = 0.5f;
   private static final float NMS_THRESHOLD = 0.1f;
   private static final float MASK_THRESHOLD = 0.0f;
   private static final YOLOv8DetectableObject OBJECT_TYPE = YOLOv8DetectableObject.CUP;
   private static final Vector3D32 OBJECT_LENGTHS = new Vector3D32(0.0f, 0.0f, 0.1f);
   private static final Vector3D32 OBJECT_RADII = new Vector3D32(0.05f, 0.0f, 0.0f);
   private static final Random random = new Random();

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor(openCLManager);
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter(openCLManager);

   private final ROS2Node node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "yolo_icp_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(node);
   private final ZEDColorDepthImageRetriever zedImageRetriever;
   private final ZEDColorDepthImagePublisher zedImagePublisher;

   private final YOLOv8ObjectDetector yoloObjectDetector = new YOLOv8ObjectDetector();

   private final IterativeClosestPointWorker icpWorker;

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();
   private final RDXPointCloudRenderer segmentedPointCloudRenderer = new RDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> segmentedPointCloud = new RecyclingArrayList<>(Point3D32::new);
   private final RDXPointCloudRenderer icpObjectPointCloudRenderer = new RDXPointCloudRenderer();
   private RDXReferenceFrameGraphic centroidGraphic;

   public RDXYOLOv8IterativeClosestPointDemo()
   {
      zedImageRetriever = new ZEDColorDepthImageRetriever(0,
                                                          ReferenceFrame::getWorldFrame,
                                                          new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                          new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      zedImagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH);

      icpWorker = new IterativeClosestPointWorker(OBJECT_TYPE.getCorrespondingShape(), OBJECT_LENGTHS, OBJECT_RADII, 500, 500, new Pose3D(), random);
      icpWorker.useProvidedTargetPoint(false);

      zedImageRetriever.start();

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      Throttler throttler = new Throttler();
      throttler.setFrequency(30.0);
      while (true)
      {
         throttler.waitAndRun();
         RawImage zedDepthImage = zedImageRetriever.getLatestRawDepthImage();
         RawImage zedColorImage = zedImageRetriever.getLatestRawColorImage(RobotSide.LEFT);

         YOLOv8DetectionResults results = yoloObjectDetector.runOnImage(zedColorImage, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);
         Mat objectMask = results.getSegmentationMatrixForObject(OBJECT_TYPE, MASK_THRESHOLD);
         if (objectMask != null)
         {
            RawImage segmentedDepth = segmenter.removeBackground(zedDepthImage, objectMask);

            List<Point3DReadOnly> ptcld = extractor.extractPointCloud(segmentedDepth);

            Collections.shuffle(ptcld, random);
            Point3DReadOnly centroid = IterativeClosestPointTools.computeCentroidOfPointCloud(ptcld, 500);
            centroidGraphic.setPositionInWorldFrame(centroid);
            Stream<Point3DReadOnly> pointCloudStream = ptcld.parallelStream();
            ptcld = pointCloudStream.sorted(Comparator.comparingDouble(point -> point.distanceSquared(centroid))).limit((int) (0.75 * ptcld.size())).collect(Collectors.toList());


            if (segmentedPointCloud != null)
               segmentedPointCloud.clear();
            for (int i = 0; i < Short.MAX_VALUE && i < ptcld.size(); ++i)
            {
               Point3D32 point = segmentedPointCloud.add();
               point.set(ptcld.get(i));
            }
            segmentedPointCloudRenderer.setPointsToRender(segmentedPointCloud, Color.GRAY);

            icpWorker.setEnvironmentPointCloud(segmentedPointCloud);
            if (icpWorker.runICP(1))
            {

                  icpObjectPointCloudRenderer.setPointsToRender(icpWorker.getObjectPointCloud(), Color.YELLOW);
            }

            segmentedDepth.release();
         }

         zedImagePublisher.setNextColorImage(zedColorImage.get(), RobotSide.LEFT);
         zedImagePublisher.setNextGpuDepthImage(zedDepthImage.get());

         zedDepthImage.release();
         zedColorImage.release();
         results.destroy();
      }
   }

   private void runUI()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            segmentedPointCloudRenderer.create(Short.MAX_VALUE);
            baseUI.getPrimaryScene().addRenderableProvider(segmentedPointCloudRenderer);

            icpObjectPointCloudRenderer.create(Short.MAX_VALUE);
            baseUI.getPrimaryScene().addRenderableProvider(icpObjectPointCloudRenderer);

            centroidGraphic = new RDXReferenceFrameGraphic(0.3);
            baseUI.getPrimaryScene().addRenderableProvider(centroidGraphic);

            RDXROS2ImageMessageVisualizer zedColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED2 Color Image",
                                                                                                      DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                                      PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(zedColorImageVisualizer, PerceptionAPI.REQUEST_ZED_COLOR);
            RDXROS2ColoredPointCloudVisualizer zedPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                                                DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                                                PerceptionAPI.ZED2_DEPTH,
                                                                                                                PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(zedPointCloudVisualizer, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);

            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            perceptionVisualizerPanel.create();
         }

         @Override
         public void render()
         {
            segmentedPointCloudRenderer.updateMesh();
            icpObjectPointCloudRenderer.updateMesh();
            perceptionVisualizerPanel.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            yoloObjectDetector.destroy();
            zedImageRetriever.destroy();
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXYOLOv8IterativeClosestPointDemo();
   }
}
