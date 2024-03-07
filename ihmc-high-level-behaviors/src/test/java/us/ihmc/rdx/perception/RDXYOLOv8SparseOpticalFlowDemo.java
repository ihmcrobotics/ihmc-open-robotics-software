package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_cudaarithm;
import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import org.bytedeco.opencv.opencv_core.Stream;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2DemandGraphNode;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.OpenCLDepthImageSegmenter;
import us.ihmc.perception.OpenCLPointCloudExtractor;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.YOLOv8.YOLOv8DetectableObject;
import us.ihmc.perception.YOLOv8.YOLOv8DetectionResults;
import us.ihmc.perception.YOLOv8.YOLOv8ObjectDetector;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.perception.opticalFlow.OpenCVSparseOpticalFlowProcessor;
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

   private OpenCVSparseOpticalFlowProcessor opticalFlowProcessor;
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
   private GpuMat lastMask = null;
   private boolean done = false;

   private final Vector2D totalTranslation = new Vector2D(0.0f, 0.0f);

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
         opticalFlowProcessor = new OpenCVSparseOpticalFlowProcessor();

      if (runYOLONotification.poll() || (keepRunningYOLO.get() && colorImage.getSequenceNumber() % 5 == 0))
      {
         YOLOv8DetectionResults yoloResults = objectDetector.runOnImage(colorImage, confidenceThreshold.get(), nmsThreshold.get());
         if (objectMask != null)
            objectMask.release();
         objectMask = yoloResults.getSegmentationMatrixForObject(OBJECT_TYPE, maskThreshold.get());
         if (objectMask != null)
         {
            opencv_imgproc.erode(objectMask.getCpuImageMat(),
                                 objectMask.getCpuImageMat(),
                                 opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                      new Size(2 * erosionKernelRadius + 1, 2 * erosionKernelRadius + 1),
                                                                      new Point(erosionKernelRadius, erosionKernelRadius)));
         }
         opticalFlowProcessor.setNewImage(compressedColorImage, objectMask);
         if (objectMask != null)
         {
            lastMask = objectMask.getGpuImageMat();
            totalTranslation.setToZero();
         }
      }
      else if (objectMask != null)
      {
         opticalFlowProcessor.setNewImage(compressedColorImage, null);

         Vector2D averageShift = opticalFlowProcessor.calculateFlow();

         if (averageShift != null)
         {
            opticalFlowMask = new RawImage(objectMask.getSequenceNumber(),
                                           objectMask.getAcquisitionTime(),
                                           objectMask.getImageWidth(),
                                           objectMask.getImageHeight(),
                                           objectMask.getDepthDiscretization(),
                                           null,
                                           getShiftedMask(averageShift, lastMask),
                                           objectMask.getOpenCVType(),
                                           objectMask.getFocalLengthX(),
                                           objectMask.getFocalLengthY(),
                                           objectMask.getPrincipalPointX(),
                                           objectMask.getPrincipalPointY(),
                                           objectMask.getPosition(),
                                           objectMask.getOrientation());
            lastMask = opticalFlowMask.getGpuImageMat();
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

   private GpuMat getShiftedMask(Vector2D averageShift, GpuMat originalMask)
   {
//      if (averageShift.norm() > 0.05)
      {
         GpuMat newMask = new GpuMat(originalMask.size(), originalMask.type());

         totalTranslation.add(averageShift);
         System.out.println("Translate = " + totalTranslation.getX() + "," + totalTranslation.getY());

         Mat cpuTranformMat = new Mat(new Size(3, 2), opencv_core.CV_32FC1);
         cpuTranformMat.data().putFloat(0L * Float.BYTES, 1.0f);
         cpuTranformMat.data().putFloat(1L * Float.BYTES, 0.0f);
         cpuTranformMat.data().putFloat(2L * Float.BYTES, averageShift.getX32()/*0.0f*/);
         cpuTranformMat.data().putFloat(3L * Float.BYTES, 0.0f);
         cpuTranformMat.data().putFloat(4L * Float.BYTES, 1.0f);
         cpuTranformMat.data().putFloat(5L * Float.BYTES, averageShift.getY32()/*0.0f*/);
         opencv_cudawarping.warpAffine(originalMask,
                                       newMask,
                                       cpuTranformMat,
                                       originalMask.size(),
                                       opencv_imgproc.INTER_LINEAR,
                                       opencv_core.BORDER_CONSTANT,
                                       new Scalar(),
                                       Stream.Null());
         opencv_cudaarithm.threshold(newMask, newMask, 0.5, 1.0, opencv_imgproc.THRESH_BINARY);

         Mat cpuOriginalMask = new Mat();
         originalMask.download(cpuOriginalMask);
         PerceptionDebugTools.display("Original Mask", cpuOriginalMask, 1);

         Mat cpuNewMask = new Mat();
         newMask.download(cpuNewMask);
         PerceptionDebugTools.display("New Mask", cpuNewMask, 1);

         cpuTranformMat.release();
         cpuNewMask.close();
         cpuOriginalMask.close();

         return newMask;
      }

      //return originalMask;
   }

   private void destroy()
   {
      imageRetriever.destroy();
      imagePublisher.destroy();
      opticalFlowProcessor.destroy();
      objectDetector.destroy();
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
      new RDXYOLOv8SparseOpticalFlowDemo();
   }
}
