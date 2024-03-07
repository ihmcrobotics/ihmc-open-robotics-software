package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_cudaarithm;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.GpuMatVector;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.OpenCLDepthImageSegmenter;
import us.ihmc.perception.OpenCLPointCloudExtractor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8DetectableObject;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opticalFlow.OpenCVDenseOpticalFlowProcessor;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXPointCloudRenderer;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXPerceptionVisualizerPanel;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ColoredPointCloudVisualizer;
import us.ihmc.rdx.ui.graphics.ros2.RDXROS2ImageMessageVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensors.ZEDColorDepthImagePublisher;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

import java.util.List;
import java.util.stream.Collectors;

public class RDXYOLOv8SparseOpticalFlowDemo
{
   private static final YOLOv8DetectableObject OBJECT_TYPE = YOLOv8DetectableObject.CUP;

   private final ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "yolov8_optical_flow_demo");
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDColorDepthImageRetriever imageRetriever;
   private final ZEDColorDepthImagePublisher imagePublisher;

   private OpenCVDenseOpticalFlowProcessor opticalFlowProcessor;
   private final YOLOv8ObjectDetector objectDetector = new YOLOv8ObjectDetector();

   private final OpenCLManager openCLManager = new OpenCLManager();
   private final OpenCLPointCloudExtractor extractor = new OpenCLPointCloudExtractor(openCLManager);
   private final OpenCLDepthImageSegmenter segmenter = new OpenCLDepthImageSegmenter(openCLManager);

   private final RDXBaseUI baseUI = new RDXBaseUI("YOLOv8 Optical Flow Demo");
   private final RDXPerceptionVisualizerPanel perceptionVisualizerPanel = new RDXPerceptionVisualizerPanel();
   private final RDXPointCloudRenderer segmentedPointCloudRenderer = new RDXPointCloudRenderer();
   private final Notification runYOLONotification = new Notification();

   private final ImBoolean keepRunningYOLO = new ImBoolean(false);
   private final ImFloat confidenceThreshold = new ImFloat(0.5f);
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImFloat maskThreshold = new ImFloat(0.0f);
   private final ImFloat outlierRejectionThreshold = new ImFloat(10.0f);
   private final ImInt erosionValue = new ImInt(3);

   private RawImage objectMask = null;
   private RawImage opticalFlowMask = null;
   private boolean done = false;

   private GpuMat indexMat;
   private int erosionKernelRadius = 2;

   public RDXYOLOv8SparseOpticalFlowDemo()
   {
      imageRetriever = new ZEDColorDepthImageRetriever(0,
                                                       ReferenceFrame::getWorldFrame,
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_DEPTH),
                                                       new ROS2DemandGraphNode(ros2Helper, PerceptionAPI.REQUEST_ZED_COLOR));
      imagePublisher = new ZEDColorDepthImagePublisher(PerceptionAPI.ZED2_COLOR_IMAGES, PerceptionAPI.ZED2_DEPTH);

      imageRetriever.start();

      ThreadTools.startAThread(this::runUI, getClass().getSimpleName() + "UI");

      while (!done)
      {
         runStuff();
      }

      destroy();
   }

   public void runStuff()
   {
      RawImage depthImage = imageRetriever.getLatestRawDepthImage();
      RawImage colorImage = imageRetriever.getLatestRawColorImage(RobotSide.LEFT);

      GpuMat compressedColor = getResizedColor(colorImage);

      RawImage compressedColorImage = new RawImage(colorImage.getSequenceNumber(),
                                                   colorImage.getAcquisitionTime(),
                                                   compressedColor.cols(),
                                                   compressedColor.rows(),
                                                   colorImage.getDepthDiscretization(),
                                                   null,
                                                   compressedColor,
                                                   colorImage.getOpenCVType(),
                                                   colorImage.getFocalLengthX() / 4,
                                                   colorImage.getFocalLengthY() / 4,
                                                   colorImage.getPrincipalPointX() / 4,
                                                   colorImage.getPrincipalPointY() / 4,
                                                   colorImage.getPosition(),
                                                   colorImage.getOrientation());

      if (opticalFlowProcessor == null)
         opticalFlowProcessor = new OpenCVSparseOpticalFlowProcessor(compressedColorImage);

      if (runYOLONotification.poll() || (keepRunningYOLO.get() && colorImage.getSequenceNumber() % 5 == 0))
      {
         opticalFlowProcessor.setFirstImage(compressedColorImage);
         YOLOv8DetectionResults yoloResults = objectDetector.runOnImage(colorImage, confidenceThreshold.get(), nmsThreshold.get());
         if (objectMask != null)
            objectMask.release();
         objectMask = yoloResults.getSegmentationMatrixForObject(OBJECT_TYPE, maskThreshold.get());

      }
      else if (objectMask != null)
      {
         opencv_imgproc.erode(objectMask.getCpuImageMat(),
                              objectMask.getCpuImageMat(),
                              opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                   new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                   new Point(erosionKernelRadius, erosionKernelRadius)));
         opticalFlowProcessor.setSecondImage(compressedColorImage);
         try (GpuMat flow = opticalFlowProcessor.calculateFlow())
         {
            if (opticalFlowMask != null)
               opticalFlowMask.release();
            opticalFlowMask = new RawImage(objectMask.getSequenceNumber(),
                                           objectMask.getAcquisitionTime(),
                                           objectMask.getImageWidth(),
                                           objectMask.getImageHeight(),
                                           objectMask.getDepthDiscretization(),
                                           null,
                                           getShiftedMask(flow, objectMask.getGpuImageMat()),
                                           objectMask.getOpenCVType(),
                                           objectMask.getFocalLengthX(),
                                           objectMask.getFocalLengthY(),
                                           objectMask.getPrincipalPointX(),
                                           objectMask.getPrincipalPointY(),
                                           objectMask.getPosition(),
                                           objectMask.getOrientation());
         }
      }

      // Segment point cloud from object mask
      RawImage segmentedDepth = null;
      if (opticalFlowMask != null)
      {
         segmentedDepth = segmenter.removeBackground(depthImage, opticalFlowMask, erosionValue.get());
      }
      else if (objectMask != null)
      {
         segmentedDepth = segmenter.removeBackground(depthImage, objectMask, erosionValue.get());
      }

      if (segmentedDepth != null)
      {
         List<Point3D32> segmentedPointCloud = extractor.extractPointCloud(segmentedDepth)
                                                        .parallelStream()
                                                        .map(Point3D32::new)
                                                        .limit(Short.MAX_VALUE)
                                                        .collect(Collectors.toList());
         synchronized (segmentedPointCloudRenderer)
         {
            segmentedPointCloudRenderer.setPointsToRender(segmentedPointCloud, Color.GREEN);
         }
         segmentedDepth.release();
      }

      imagePublisher.setNextColorImage(colorImage.get(), RobotSide.LEFT);
      imagePublisher.setNextColorImage(compressedColorImage.get(), RobotSide.RIGHT);
      imagePublisher.setNextGpuDepthImage(depthImage.get());

      compressedColorImage.release();
      depthImage.release();
      colorImage.release();
   }

   private GpuMat getResizedColor(RawImage colorImage)
   {
      GpuMat compressedColor = new GpuMat();
      opencv_cudawarping.resize(colorImage.getGpuImageMat(), compressedColor, new Size(colorImage.getImageWidth() / 4, colorImage.getImageHeight() / 4));

      return compressedColor;
   }

   private GpuMat getShiftedMask(GpuMat flow, GpuMat originalMask)
   {
      if (indexMat == null)
      {
         Mat cpuIndexMat = new Mat(flow.size(), flow.type());

         for (int y = 0; y < cpuIndexMat.rows(); ++y)
         {
            for (int x = 0; x < cpuIndexMat.cols(); ++x)
            {
               BytePointer mapPointer = cpuIndexMat.ptr(y, x);
               mapPointer.putFloat((float) x);
               mapPointer.putFloat(Float.BYTES, (float) y);
            }
         }

         indexMat = new GpuMat(flow.size(), flow.type());
         indexMat.upload(cpuIndexMat);
      }

      GpuMat newMask = new GpuMat(originalMask.size(), originalMask.type());
      GpuMat map = new GpuMat(flow.size(), flow.type());

      opencv_cudaarithm.scaleAdd(flow, -1.0, indexMat, map);
      GpuMatVector splitMap = new GpuMatVector();
      opencv_cudaarithm.split(map, splitMap);

      opencv_cudawarping.remap(originalMask, newMask, splitMap.get(0), splitMap.get(1), opencv_imgproc.INTER_NEAREST);

      Mat cpuOriginalMask = new Mat();
      originalMask.download(cpuOriginalMask);
      PerceptionDebugTools.display("Original Mask", cpuOriginalMask, 1);

      Mat cpuNewMask = new Mat();
      newMask.download(cpuNewMask);
      PerceptionDebugTools.display("New Mask", cpuNewMask, 1);

      cpuNewMask.close();
      cpuOriginalMask.close();

      return newMask;
   }

   private void destroy()
   {
      imageRetriever.destroy();
      imagePublisher.destroy();
      opticalFlowProcessor.destroy();
      objectDetector.destroy();
      indexMat.release();
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

            RDXROS2ImageMessageVisualizer colorImageVisualizer = new RDXROS2ImageMessageVisualizer("Color Image",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(colorImageVisualizer, PerceptionAPI.REQUEST_ZED_COLOR);

            RDXROS2ImageMessageVisualizer depthImageVisualizer = new RDXROS2ImageMessageVisualizer("Depth Image",
                                                                                                   PubSubImplementation.FAST_RTPS,
                                                                                                   PerceptionAPI.ZED2_DEPTH);
            perceptionVisualizerPanel.addVisualizer(depthImageVisualizer, PerceptionAPI.REQUEST_ZED_DEPTH);

            RDXROS2ImageMessageVisualizer opticalFlowImageVisualizer = new RDXROS2ImageMessageVisualizer("Optical Flow",
                                                                                                         PubSubImplementation.FAST_RTPS,
                                                                                                         PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.RIGHT));
            perceptionVisualizerPanel.addVisualizer(opticalFlowImageVisualizer, PerceptionAPI.REQUEST_ZED_POINT_CLOUD);

            RDXROS2ColoredPointCloudVisualizer pointCloudVisualizer = new RDXROS2ColoredPointCloudVisualizer("Point Cloud",
                                                                                                             PubSubImplementation.FAST_RTPS,
                                                                                                             PerceptionAPI.ZED2_DEPTH,
                                                                                                             PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
            perceptionVisualizerPanel.addVisualizer(pointCloudVisualizer);

            perceptionVisualizerPanel.create();
            baseUI.getImGuiPanelManager().addPanel(perceptionVisualizerPanel);
            baseUI.getImGuiPanelManager().addPanel("Settings", this::renderSettings);
            baseUI.getPrimaryScene().addRenderableProvider(perceptionVisualizerPanel);
            baseUI.create();
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

         private void renderSettings()
         {
            ImGui.checkbox("Keep Running YOLO", keepRunningYOLO);
            if (ImGui.button("Run YOLO"))
            {
               runYOLONotification.set();
            }
            ImGui.sliderFloat("confidenceThreshold", confidenceThreshold.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("nmsThreshold", nmsThreshold.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("maskThreshold", maskThreshold.getData(), -1.0f, 1.0f);
            ImGui.sliderFloat("outlierRejectionThreshold", outlierRejectionThreshold.getData(), 0.0f, 50.0f);
            ImGui.sliderInt("erosionValue", erosionValue.getData(), 0, 20);
         }

         @Override
         public void dispose()
         {
            done = true;
            perceptionVisualizerPanel.destroy();
            baseUI.dispose();
            segmentedPointCloudRenderer.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXYOLOv8OpticalFlowDemo();
   }
}
