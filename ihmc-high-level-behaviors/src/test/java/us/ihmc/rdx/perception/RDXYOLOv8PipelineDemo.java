package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.apache.logging.log4j.core.util.ExecutorServices;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDADepthImageSegmenter;
import us.ihmc.perception.cuda.CUDAPointCloudExtractor;
import us.ihmc.perception.detections.yolo.YOLOv8Detection;
import us.ihmc.perception.detections.yolo.YOLOv8DetectionList;
import us.ihmc.perception.detections.yolo.YOLOv8InstantDetection;
import us.ihmc.perception.detections.yolo.YOLOv8Model;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.zed.global.zed;

import java.io.File;
import java.net.URL;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

public class RDXYOLOv8PipelineDemo
{
   private static final String SVO_FILE = System.getProperty("user.home") + "/Downloads/20251217_151953_H1NewModelTest.svo2";

   private static final String SAVE_DIRECTORY = System.getProperty("user.home") + File.separator + "Documents" + File.separator;

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(RDXYOLOv8PipelineDemo.class.getSimpleName());
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ROS2ZEDSVOPlaybackSensor zedPlaybackSensor = new ROS2ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, zed.SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
   private RawImage colorImage;
   private final RDXImageVisualizer colorImageVisualizer = new RDXImageVisualizer("ZED Color", "ZED Color", false);
   private RawImage depthImage;
   private final RDXImageVisualizer depthImageVisualizer = new RDXImageVisualizer("ZED Depth", "ZED Depth", false);
   private final RDXRawImagePointCloudVisualizer zedPointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud", true);
   private final ImBoolean renderZEDPointCloud = new ImBoolean(true);

   private final List<YOLOv8Model> yoloModels = new ArrayList<>();
   private final List<String> availableModels = new ArrayList<>();
   private final ImInt selectedDetector = new ImInt(0);

   private RawImage detectionMask;
   private final RDXImageVisualizer detectionMaskVisualizer = new RDXImageVisualizer("Detection Mask", "Detection Mask", false);
   private RawImage erodedMask;
   private final RDXImageVisualizer erodedMaskVisualizer = new RDXImageVisualizer("Eroded Mask", "Eroded Mask", false);
   private RawImage annotatedImage;
   private final RDXImageVisualizer annotatedImageVisualizer = new RDXImageVisualizer("Annotated Image", "Annotated Image", false);

   private final ImFloat confidenceThreshold = new ImFloat(0.8f);
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImFloat maskThreshold = new ImFloat(0.0f);
   private final ImInt erosionKernelRadius = new ImInt(1);

   private final CUDADepthImageSegmenter depthImageSegmenter;
   private RawImage segmentedDepth;
   private final RDXImageVisualizer segmentedDepthVisualizer = new RDXImageVisualizer("Segmented Depth", "Segmented Depth", false);
   private final RDXRawImagePointCloudVisualizer segmentedPointCloudVisualizer = new RDXRawImagePointCloudVisualizer("Segmented Point Cloud", true);
   private final ImBoolean renderSegmentedPointCloud = new ImBoolean(false);

   private final CUDAPointCloudExtractor pointCloudExtractor = new CUDAPointCloudExtractor();
   private final Point3D32 centroid = new Point3D32();
   private ModelInstance centroidBall;
   private final ImBoolean renderCentroid = new ImBoolean(false);

   private final ExecutorService executor = Executors.newSingleThreadExecutor();
   private Future<?> task;
   private boolean wasDone = false;
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass().getSimpleName());

   private final ImInt frameToGrab = new ImInt(0);

   private RDXYOLOv8PipelineDemo() throws Exception
   {
      for (URL yoloModelDirectory : YOLOv8Tools.getYOLOModelDirectories())
      {
         YOLOv8Model model = new YOLOv8Model(yoloModelDirectory);

         LogTools.info("Loaded YOLOv8 model: " + model.getName());
         LogTools.info("\t\t\tClasses: " + model.getDetectableObjectCount());

         yoloModels.add(model);
         availableModels.add(model.getName());
      }

      depthImageSegmenter = new CUDADepthImageSegmenter();

      zedPlaybackSensor.useTrackedPose(false);
      zedPlaybackSensor.run(true);
      try
      {
         zedPlaybackSensor.waitForGrab();
      } catch (InterruptedException ignored) {}
      zedPlaybackSensor.run(false);

      task = executor.submit(() -> grabFrame(frameToGrab.get()));

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            centroidBall = RDXModelBuilder.createSphere(0.025f, Color.BLUE);
            baseUI.getPrimaryScene().addRenderableProvider((array, pool) ->
            {
               if (renderCentroid.get())
                  centroidBall.getRenderables(array, pool);
            });

            zedPointCloudVisualizer.create();
            segmentedPointCloudVisualizer.create();

            zedPointCloudVisualizer.setActive(true);
            segmentedPointCloudVisualizer.setActive(true);

            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool, sceneLevels) ->
            {
               if (renderZEDPointCloud.get())
                  zedPointCloudVisualizer.getRenderables(renderables, pool, sceneLevels);
            });
            baseUI.getPrimaryScene().addRenderableProvider((renderables, pool, sceneLevels) ->
            {
               if (renderSegmentedPointCloud.get())
                  segmentedPointCloudVisualizer.getRenderables(renderables, pool, sceneLevels);
            });

            colorImageVisualizer.setActive(true);
            depthImageVisualizer.setActive(true);
            detectionMaskVisualizer.setActive(true);
            erodedMaskVisualizer.setActive(true);
            annotatedImageVisualizer.setActive(true);
            segmentedDepthVisualizer.setActive(true);

            baseUI.getImGuiPanelManager().addPanel("Options", this::frameSettings);
            baseUI.getImGuiPanelManager().addPanel(colorImageVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel(depthImageVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel(detectionMaskVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel(erodedMaskVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel(annotatedImageVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel(segmentedDepthVisualizer.getPanel());

            baseUI.create();
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void frameSettings()
         {
            ImGui.combo("YOLO Model", selectedDetector, availableModels.toArray(String[]::new));
            if (ImGui.sliderInt("Frame", frameToGrab.getData(), 0, zedPlaybackSensor.getLength() - 1))
            {
               task = executor.submit(() -> grabFrame(frameToGrab.get()));
               wasDone = false;
            }
            if (ImGui.button("Run Pipeline"))
            {
               task = executor.submit(() -> runYOLO());
               wasDone = false;
            }

            if (task.isDone() && !wasDone)
            {
               wasDone = true;
               updateRenderables();
            }

            if (ImGui.button("Save Images"))
               saveImages();

            ImGui.separator();

            ImGui.sliderFloat("Confidence Threshold", confidenceThreshold.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("nmsThreshold", nmsThreshold.getData(), 0.0f, 1.0f);
            ImGui.sliderFloat("maskThreshold", maskThreshold.getData(), -1.0f, 1.0f);
            ImGui.sliderInt("Erosion Kernel Radius", erosionKernelRadius.getData(), 0, 5);

            ImGui.separator();

            zedPointCloudVisualizer.renderImGuiWidgets();

            ImGui.separator();

            segmentedPointCloudVisualizer.renderImGuiWidgets();

            ImGui.separator();

            ImGui.checkbox("Render ZED Point Cloud", renderZEDPointCloud);
            ImGui.checkbox("Render Segmented Point Cloud", renderSegmentedPointCloud);
            ImGui.checkbox("Render Centroid", renderCentroid);
         }

         @Override
         public void dispose()
         {
            destroy();
         }
      });
   }

   private void grabFrame(int frameToGrab)
   {
      zedPlaybackSensor.setCurrentPosition(frameToGrab);

      zedPlaybackSensor.run(true);
      try
      {
         zedPlaybackSensor.waitForGrab();
      } catch (InterruptedException ignored) {}
      zedPlaybackSensor.run(false);

      if (colorImage != null)
         colorImage.release();
      if (depthImage != null)
         depthImage.release();

      colorImage = zedPlaybackSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      depthImage = zedPlaybackSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
   }

   private void runYOLO()
   {
      // Get the model
      YOLOv8Model model = yoloModels.get(selectedDetector.get());

      // Set parameters
      model.setConfidenceThresholds(confidenceThreshold.get());
      model.setMaskThresholds(maskThreshold.get());
      model.setNMSThreshold(nmsThreshold.get());

      // Run YOLO on a BGR image
      Mat bgrMat = new Mat();
      opencv_imgproc.cvtColor(colorImage.getCpuImageMat(), bgrMat, opencv_imgproc.COLOR_BGRA2BGR);
      RawImage bgrImage = colorImage.replaceImage(bgrMat, PixelFormat.BGR8);
      YOLOv8DetectionList results = model.run(bgrImage);
      if (results.isEmpty())
         return;

      // Release previous result images
      if (detectionMask != null)
      {
         detectionMask.release();
         detectionMask = null;
      }
      if (erodedMask != null)
      {
         erodedMask.release();
         erodedMask = null;
      }
      if (segmentedDepth != null)
      {
         segmentedDepth.release();
         segmentedDepth = null;
      }
      if (annotatedImage != null)
      {
         annotatedImage.release();
         annotatedImage = null;
      }

      // Get a detection
      YOLOv8Detection detection = results.get(0);
      detectionMask = detection.mask();

      // Get the eroded mask
      Mat erodedMat = new Mat();
      opencv_imgproc.erode(detectionMask.getCpuImageMat(),
                           erodedMat,
                           opencv_imgproc.getStructuringElement(opencv_imgproc.CV_SHAPE_RECT,
                                                                new Size(2 * erosionKernelRadius.get() + 1, 2 * erosionKernelRadius.get() + 1),
                                                                new Point(erosionKernelRadius.get(), erosionKernelRadius.get())));
      erodedMask = detectionMask.replaceImage(erodedMat);

      // Segment depth using eroded mask
      segmentedDepth = depthImageSegmenter.removeBackground(depthImage, erodedMask);

      // Get an annotated image
      YOLOv8InstantDetection instantDetection = new YOLOv8InstantDetection(detection.objectClass(),
                                                                           detection.confidence(),
                                                                           new Pose3D(centroid, new RotationMatrix()),
                                                                           Instant.now(),
                                                                           bgrImage,
                                                                           erodedMask,
                                                                           depthImage,
                                                                           detection.boundingBox(),
                                                                           null);
      annotatedImage = bgrImage.replaceImage(new Mat(bgrImage.getCpuImageMat().size(), bgrImage.getOpenCVType()));
      YOLOv8Tools.annotateImage(bgrImage.getCpuImageMat(), annotatedImage.getCpuImageMat(), List.of(instantDetection));

      // Find the centroid of the segmented depth
      centroid.set(findCentroid(segmentedDepth));

      // Release stuff
      bgrImage.release();
      results.destroy();
   }

   private Point3D32 findCentroid(RawImage depthImage)
   {
      List<Point3D32> pointCloud = pointCloudExtractor.extractPointCloud(depthImage);

      Point3D32 centroid = new Point3D32();
      centroid.setToZero();
      for (Point3D32 point3D32 : pointCloud)
         centroid.add(point3D32);
      centroid.scale(1.0 / pointCloud.size());

      return centroid;
   }

   private void updateRenderables()
   {
      // Render color
      colorImageVisualizer.setImage(colorImage);
      colorImageVisualizer.update();

      // Render depth
      depthImageVisualizer.setImage(depthImage);
      depthImageVisualizer.update();

      // Render ZED point cloud
      zedPointCloudVisualizer.setColorImage(colorImage);
      zedPointCloudVisualizer.setDepthImage(depthImage);
      zedPointCloudVisualizer.update();

      // Render detection mask
      if (detectionMask != null)
      {
         Mat grayMask = new Mat();
         detectionMask.getCpuImageMat().convertTo(grayMask, opencv_core.CV_8UC1);
         opencv_imgproc.threshold(grayMask, grayMask, 0.5, 255.0, opencv_imgproc.THRESH_BINARY);
         detectionMaskVisualizer.setImage(grayMask, PixelFormat.GRAY8);
         detectionMaskVisualizer.update();
         grayMask.close();
      }

      // Render eroded mask
      if (erodedMask != null)
      {
         Mat grayErodedMask = new Mat();
         erodedMask.getCpuImageMat().convertTo(grayErodedMask, opencv_core.CV_8UC1);
         opencv_imgproc.threshold(grayErodedMask, grayErodedMask, 0.5, 255.0, opencv_imgproc.THRESH_BINARY);
         erodedMaskVisualizer.setImage(grayErodedMask, PixelFormat.GRAY8);
         erodedMaskVisualizer.update();
         grayErodedMask.close();
      }

      // Render segmented depth
      if (segmentedDepth != null)
      {
         segmentedDepthVisualizer.setImage(segmentedDepth);
         segmentedDepthVisualizer.update();
      }

      // Render segmented point cloud
      if (segmentedDepth != null)
      {
         segmentedPointCloudVisualizer.setColorImage(colorImage);
         segmentedPointCloudVisualizer.setDepthImage(segmentedDepth);
         segmentedPointCloudVisualizer.update();
      }

      // Set the centroid transform
      centroidBall.transform.setTranslation(centroid.getX32(), centroid.getY32(), centroid.getZ32());

      // Render annotated image
      if (annotatedImage != null)
      {
         annotatedImageVisualizer.setImage(annotatedImage);
         annotatedImageVisualizer.update();
      }
   }

   private void saveImages()
   {
      // Create a directory to save images in
      String resultDirectoryName = "YOLOPipelineDemo_Frame" + frameToGrab.get() + "_" + new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
      File resultDirectory = new File(SAVE_DIRECTORY + resultDirectoryName);
      if (!resultDirectory.exists())
         if (!resultDirectory.mkdir())
            throw new RuntimeException("Faild to save images");

      String resultDirectoryPath = resultDirectory.getAbsolutePath() + File.separator;

      // Save images as PNG to the directory
      opencv_imgcodecs.imwrite(resultDirectoryPath + "Color.png", colorImage.getCpuImageMat());
      opencv_imgcodecs.imwrite(resultDirectoryPath + "Depth.png", depthImage.getCpuImageMat());

      Mat detectionMat = new Mat();
      detectionMask.getCpuImageMat().convertTo(detectionMat, opencv_core.CV_8UC1);
      opencv_imgproc.threshold(detectionMat, detectionMat, 0.5, 255.0, opencv_imgproc.THRESH_BINARY);
      opencv_imgcodecs.imwrite(resultDirectoryPath + "DetectionMask.png", detectionMat);
      detectionMat.close();

      Mat erodedMat = new Mat();
      erodedMask.getCpuImageMat().convertTo(erodedMat, opencv_core.CV_8UC1);
      opencv_imgproc.threshold(erodedMat, erodedMat, 0.5, 255.0, opencv_imgproc.THRESH_BINARY);
      opencv_imgcodecs.imwrite(resultDirectoryPath + "ErodedMask.png", erodedMat);
      erodedMat.close();

      opencv_imgcodecs.imwrite(resultDirectoryPath + "SegmentedDepth.png", segmentedDepth.getCpuImageMat());
      opencv_imgcodecs.imwrite(resultDirectoryPath + "Annotated.png", annotatedImage.getCpuImageMat());
   }

   private void destroy()
   {
      baseUI.dispose();
      ExecutorServices.shutdown(executor, 2, TimeUnit.SECONDS, getClass().getSimpleName());

      if (colorImage != null)
         colorImage.release();
      if (depthImage != null)
         depthImage.release();
      if (detectionMask != null)
         detectionMask.release();
      if (erodedMask != null)
         erodedMask.release();
      if (segmentedDepth != null)
         segmentedDepth.release();
      if (annotatedImage != null)
         annotatedImage.release();

      for (YOLOv8Model model : yoloModels)
         model.destroy();

      zedPointCloudVisualizer.destroy();
      segmentedPointCloudVisualizer.destroy();
      colorImageVisualizer.destroy();
      depthImageVisualizer.destroy();
      detectionMaskVisualizer.destroy();
      erodedMaskVisualizer.destroy();
      segmentedDepthVisualizer.destroy();
      annotatedImageVisualizer.destroy();

      depthImageSegmenter.close();
      pointCloudExtractor.close();
      zedPlaybackSensor.close();
      ros2Node.destroy();
   }

   public static void main(String[] args) throws Exception
   {
      new RDXYOLOv8PipelineDemo();
   }
}
