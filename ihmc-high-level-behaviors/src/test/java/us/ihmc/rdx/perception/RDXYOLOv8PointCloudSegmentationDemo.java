package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.YOLOv8.YOLOv8DetectionClass;
import us.ihmc.perception.detections.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.detections.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizersPanel;
import us.ihmc.rdx.ui.graphics.ros2.pointCloud.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

import java.util.Collections;
import java.util.List;
import java.util.Random;

public class RDXYOLOv8PointCloudSegmentationDemo
{
   private static final float CONFIDENCE_THRESHOLD = 0.5f;
   private static final float NMS_THRESHOLD = 0.1f;
   private static final float MASK_THRESHOLD = 0.0f;
   private static final YOLOv8DetectionClass OBJECT_TYPE = YOLOv8DetectionClass.DOOR_LEVER;
   private static final Random random = new Random();

   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor();
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter();

   private final ROS2Node node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "yolo_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(node);
   private final ZEDColorDepthImageRetriever zedImageRetriever;
   private final ZEDColorDepthImagePublisher zedImagePublisher;

   private final YOLOv8ObjectDetector yoloObjectDetector = new YOLOv8ObjectDetector();

   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXPerceptionVisualizersPanel perceptionVisualizerPanel = new RDXPerceptionVisualizersPanel();
   private final RDXPointCloudRenderer segmentedPointCloudRenderer = new RDXPointCloudRenderer();
   private final RecyclingArrayList<Point3D32> segmentedPointCloud = new RecyclingArrayList<>(Point3D32::new);

   public RDXYOLOv8PointCloudSegmentationDemo()
   {
      zedImageRetriever = new ZEDColorDepthImageRetriever(0,
                                                          ReferenceFrame::getWorldFrame,
                                                          new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                          new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      zedImagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH, PerceptionAPI.ZED2_CUT_OUT_DEPTH);

      zedImageRetriever.start();

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      while (true)
      {
         RawImage zedDepthImage = zedImageRetriever.getLatestRawDepthImage();
         RawImage zedColorImage = zedImageRetriever.getLatestRawColorImage(RobotSide.LEFT);

         YOLOv8DetectionResults results = yoloObjectDetector.runOnImage(zedColorImage, CONFIDENCE_THRESHOLD, NMS_THRESHOLD);
         RawImage objectMask = results.getSegmentationMatrixForObject(OBJECT_TYPE, MASK_THRESHOLD);
         if (objectMask != null)
         {
            RawImage segmentedDepth = segmenter.removeBackground(zedDepthImage, objectMask);

            List<Point3D32> ptcld = extractor.extractPointCloud(segmentedDepth);
            Collections.shuffle(ptcld, random);
            synchronized (segmentedPointCloudRenderer)
            {
               if (segmentedPointCloud != null)
                  segmentedPointCloud.clear();
               for (int i = 0; i < Short.MAX_VALUE && i < ptcld.size(); ++i)
               {
                  Point3D32 point = segmentedPointCloud.add();
                  point.set(ptcld.get(i));
               }
               segmentedPointCloudRenderer.setPointsToRender(segmentedPointCloud, Color.GREEN);
            }

            segmentedDepth.release();
         }

         zedImagePublisher.setNextColorImage(zedColorImage.get(), RobotSide.LEFT);
         zedImagePublisher.setNextGpuDepthImage(zedDepthImage.get());

         if (objectMask != null)
            objectMask.release();
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

            RDXROS2ImageMessageVisualizer zedColorImageVisualizer = new RDXROS2ImageMessageVisualizer("ZED2 Color Image",
                                                                                                      DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                                      PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            zedColorImageVisualizer.createRequestHeartbeat(node, PerceptionAPI.REQUEST_ZED_COLOR);
            perceptionVisualizerPanel.addVisualizer(zedColorImageVisualizer);
            RDXROS2ColoredPointCloudVisualizer zedPointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("ZED 2 Colored Point Cloud",
                                                                                                                DomainFactory.PubSubImplementation.FAST_RTPS,
                                                                                                                PerceptionAPI.ZED2_DEPTH,
                                                                                                                PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            zedPointCloudVisualizer.createRequestHeartbeat(node, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);
            perceptionVisualizerPanel.addVisualizer(zedPointCloudVisualizer);

            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.create();
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            perceptionVisualizerPanel.create();
         }

         @Override
         public void render()
         {
            synchronized (segmentedPointCloudRenderer)
            {
               segmentedPointCloudRenderer.updateMesh();
            }
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
      new RDXYOLOv8PointCloudSegmentationDemo();
   }
}