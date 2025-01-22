package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.apache.logging.log4j.core.util.ExecutorServices;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Point;
import org.bytedeco.opencv.opencv_core.Rect;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.MathTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.detections.yolo.YOLOv8Detection;
import us.ihmc.perception.detections.yolo.YOLOv8Model;
import us.ihmc.perception.detections.yolo.YOLOv8Tools;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencl.OpenCLDepthImageSegmenter;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;

import java.io.File;
import java.nio.file.Path;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;

public class RDXYOLOv8PipelineDemo
{
   private static final int FONT = opencv_imgproc.FONT_HERSHEY_DUPLEX;
   private static final double FONT_SCALE = 1.5;
   private static final int FONT_THICKNESS = 2;
   private static final int LINE_TYPE = opencv_imgproc.LINE_4;
   private static final Scalar BOUNDING_BOX_COLOR = new Scalar(0.0, 196.0, 0.0, 255.0);
   private static final Mat GREEN_MAT = new Mat(1, 1, opencv_core.CV_8UC3, new Scalar(0.0, 255.0, 0.0, 255.0));

   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2").toAbsolutePath().toString();

   private static final String SAVE_DIRECTORY = System.getProperty("user.home") + File.separator + "Documents" + File.separator;

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(RDXYOLOv8PipelineDemo.class.getSimpleName());
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDSVOPlaybackSensor zed = new ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, SVO_FILE);
   private RawImage colorImage;
   private final RDXOpenCVVideoVisualizer colorImageVisualizer = new RDXOpenCVVideoVisualizer("ZED Color", "ZED Color", false);
   private RawImage depthImage;
   private final RDXOpenCVVideoVisualizer depthImageVisualizer = new RDXOpenCVVideoVisualizer("ZED Depth", "ZED Depth", false);
   private final RDXRawImagePointCloudVisualizer zedPointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud", true);
   private final ImBoolean renderZEDPointCloud = new ImBoolean(true);

   private final List<YOLOv8Model> yoloModels = new ArrayList<>();
   private final List<String> availableModels = new ArrayList<>();
   private final ImInt selectedDetector = new ImInt(0);

   private RawImage detectionMask;
   private final RDXOpenCVVideoVisualizer detectionMaskVisualizer = new RDXOpenCVVideoVisualizer("Detection Mask", "Detection Mask", false);
   private RawImage erodedMask;
   private final RDXOpenCVVideoVisualizer erodedMaskVisualizer = new RDXOpenCVVideoVisualizer("Eroded Mask", "Eroded Mask", false);
   private RawImage annotatedImage;
   private final RDXOpenCVVideoVisualizer annotatedImageVisualizer = new RDXOpenCVVideoVisualizer("Annotated Image", "Annotated Image", false);

   private final ImFloat confidenceThreshold = new ImFloat(0.8f);
   private final ImFloat nmsThreshold = new ImFloat(0.1f);
   private final ImFloat maskThreshold = new ImFloat(0.0f);
   private final ImInt erosionKernelRadius = new ImInt(1);

   private final OpenCLDepthImageSegmenter depthImageSegmenter = new OpenCLDepthImageSegmenter();
   private RawImage segmentedDepth;
   private final RDXOpenCVVideoVisualizer segmentedDepthVisualizer = new RDXOpenCVVideoVisualizer("Segmented Depth", "Segmented Depth", false);
   private final RDXRawImagePointCloudVisualizer segmentedPointCloudVisualizer = new RDXRawImagePointCloudVisualizer("Segmented Point Cloud", true);
   private final ImBoolean renderSegmentedPointCloud = new ImBoolean(false);

   private final OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor();
   private final Point3D32 centroid = new Point3D32();
   private ModelInstance centroidBall;
   private final ImBoolean renderCentroid = new ImBoolean(false);

   private final ExecutorService executor = Executors.newSingleThreadExecutor();
   private Future<?> task;
   private boolean wasDone = false;
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass().getSimpleName());

   private final ImInt frameToGrab = new ImInt(0);

   private RDXYOLOv8PipelineDemo()
   {
      for (Path yoloModelDirectory : YOLOv8Tools.getYOLOModelDirectories())
      {
         YOLOv8Model model = new YOLOv8Model(yoloModelDirectory);

         LogTools.info("Loaded YOLOv8 model: " + YOLOv8Tools.getONNXFile(yoloModelDirectory));
         LogTools.info("\t\t\tClasses: " + model.getDetectionClassNames().size());

         yoloModels.add(model);
         availableModels.add(model.getName());
      }

      zed.useTrackedPose(false);
      zed.run(true);
      try
      {
         zed.waitForGrab();
      } catch (InterruptedException ignored) {}
      zed.run(false);

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
            if (ImGui.sliderInt("Frame", frameToGrab.getData(), 0, zed.getLength() - 1))
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
      zed.setCurrentPosition(frameToGrab);

      zed.run(true);
      try
      {
         zed.waitForGrab();
      } catch (InterruptedException ignored) {}
      zed.run(false);

      if (colorImage != null)
         colorImage.release();
      if (depthImage != null)
         depthImage.release();

      colorImage = zed.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      depthImage = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
   }

   private void runYOLO()
   {
      // Get the model
      YOLOv8Model model = yoloModels.get(selectedDetector.get());

      // Run YOLO on a BGR image
      Mat bgrMat = new Mat();
      opencv_imgproc.cvtColor(colorImage.getCpuImageMat(), bgrMat, opencv_imgproc.COLOR_BGRA2BGR);
      RawImage bgrImage = colorImage.replaceImage(bgrMat, PixelFormat.BGR8);
      List<YOLOv8Detection> results = model.run(bgrImage, confidenceThreshold.get(), nmsThreshold.get(), maskThreshold.get());
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

      // Get an annotated image
      annotatedImage = bgrImage.replaceImage(bgrImage.getCpuImageMat().clone());
      annotateImage(detection, annotatedImage);

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

      // Find the centroid of the segmented depth
      centroid.set(findCentroid(segmentedDepth));

      // Release stuff
      bgrImage.release();
      for (YOLOv8Detection yoloDetection : results)
         yoloDetection.destroy();
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

   private void annotateImage(YOLOv8Detection detection, RawImage imageToAnnotate)
   {
      Mat annotatedMat = imageToAnnotate.getCpuImageMat();
      String text = String.format("%s: %.2f", detection.name(), detection.confidence());

      // Draw the bounding box
      Rect boundingBox = detection.boundingBox();
      opencv_imgproc.rectangle(annotatedMat, boundingBox, BOUNDING_BOX_COLOR, 5, LINE_TYPE, 0);

      // Draw text background
      Size textSize = opencv_imgproc.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS, new IntPointer());

      int textBoxClampedX = MathTools.clamp(boundingBox.x(), 0, imageToAnnotate.getWidth() - textSize.width());
      int textBoxClampedY = MathTools.clamp(boundingBox.y() - textSize.height(), 0, imageToAnnotate.getHeight() - textSize.height());

      Rect textBox = new Rect(textBoxClampedX, textBoxClampedY, textSize.width(), textSize.height());

      opencv_imgproc.rectangle(annotatedMat, textBox, BOUNDING_BOX_COLOR, opencv_imgproc.FILLED, LINE_TYPE, 0);

      opencv_imgproc.putText(annotatedMat,
                             text,
                             new Point(textBoxClampedX, textBoxClampedY + textSize.height()),
                             opencv_imgproc.CV_FONT_HERSHEY_DUPLEX,
                             FONT_SCALE,
                             new Scalar(255.0, 255.0, 255.0, 255.0),
                             FONT_THICKNESS,
                             LINE_TYPE,
                             false);

      // Add green tint to show mask
      RawImage mask = detection.mask();
      Mat maskMat = mask.getCpuImageMat().clone();

      // resize the mask to fit the result image
      opencv_imgproc.resize(maskMat, maskMat, annotatedMat.size(), 0.0, 0.0, opencv_imgproc.INTER_NEAREST);

      // ensure the green Mat is same size as image
      if (annotatedMat.cols() != GREEN_MAT.cols() || annotatedMat.rows() != GREEN_MAT.rows())
         opencv_imgproc.resize(GREEN_MAT, GREEN_MAT, annotatedMat.size());

      // add a green tint where mask = 255
      opencv_core.add(annotatedMat, GREEN_MAT, annotatedMat, maskMat, -1);
      maskMat.close();
      mask.release();
   }

   private void updateRenderables()
   {
      // Render color
      colorImageVisualizer.updateImageDimensions(colorImage.getWidth(), colorImage.getHeight());
      colorImage.getPixelFormat().convertToRGBA(colorImage.getCpuImageMat(), colorImageVisualizer.getRGBA8Mat());
      colorImageVisualizer.update();

      // Render depth
      depthImageVisualizer.updateImageDimensions(depthImage.getWidth(), depthImage.getHeight());
      Mat grayDepth = new Mat();
//      depthImage.getCpuImageMat().convertTo(grayDepth, opencv_core.CV_8UC1, 1.0 / 256.0, 0.0);
//      opencv_core.subtract(new Scalar(255.0), grayDepth).asMat().copyTo(grayDepth);
      opencv_core.normalize(depthImage.getCpuImageMat(), grayDepth, 0.0, 255.0, opencv_core.NORM_MINMAX, opencv_core.CV_8UC1, null);
      opencv_imgproc.cvtColor(grayDepth, depthImageVisualizer.getRGBA8Mat(), opencv_imgproc.COLOR_GRAY2BGRA);
      depthImageVisualizer.update();
      grayDepth.close();

      // Render ZED point cloud
      zedPointCloudVisualizer.setColorImage(colorImage);
      zedPointCloudVisualizer.setDepthImage(depthImage);
      zedPointCloudVisualizer.update();

      // Render detection mask
      if (detectionMask != null)
      {
         detectionMaskVisualizer.updateImageDimensions(detectionMask.getWidth(), detectionMask.getHeight());
         Mat grayMask = new Mat();
         detectionMask.getCpuImageMat().convertTo(grayMask, opencv_core.CV_8UC1);
         opencv_imgproc.threshold(grayMask, grayMask, 0.5, 255.0, opencv_imgproc.THRESH_BINARY);
         opencv_imgproc.cvtColor(grayMask, detectionMaskVisualizer.getRGBA8Mat(), opencv_imgproc.COLOR_GRAY2RGBA);
         detectionMaskVisualizer.update();
         grayMask.close();
      }

      // Render eroded mask
      if (erodedMask != null)
      {
         erodedMaskVisualizer.updateImageDimensions(erodedMask.getWidth(), erodedMask.getHeight());
         Mat grayErodedMask = new Mat();
         erodedMask.getCpuImageMat().convertTo(grayErodedMask, opencv_core.CV_8UC1);
         opencv_imgproc.threshold(grayErodedMask, grayErodedMask, 0.5, 255.0, opencv_imgproc.THRESH_BINARY);
         opencv_imgproc.cvtColor(grayErodedMask, erodedMaskVisualizer.getRGBA8Mat(), opencv_imgproc.COLOR_GRAY2RGBA);
         erodedMaskVisualizer.update();
         grayErodedMask.close();
      }

      // Render segmented depth
      if (segmentedDepth != null)
      {
         segmentedDepthVisualizer.updateImageDimensions(segmentedDepth.getWidth(), segmentedDepth.getHeight());
         Mat segmentedGrayDepth = new Mat();
         //      segmentedDepth.getCpuImageMat().convertTo(segmentedGrayDepth, opencv_core.CV_8UC1, 1.0 / 256.0, 0.0);
         //      opencv_core.subtract(new Scalar(255.0), segmentedGrayDepth).asMat().copyTo(segmentedGrayDepth);
         opencv_core.normalize(segmentedDepth.getCpuImageMat(), segmentedGrayDepth, 0.0, 255.0, opencv_core.NORM_MINMAX, opencv_core.CV_8UC1, null);
         opencv_imgproc.cvtColor(segmentedGrayDepth, segmentedDepthVisualizer.getRGBA8Mat(), opencv_imgproc.COLOR_GRAY2BGRA);
         segmentedDepthVisualizer.update();
         segmentedGrayDepth.close();
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
         annotatedImageVisualizer.updateImageDimensions(annotatedImage.getWidth(), annotatedImage.getHeight());
         annotatedImage.getPixelFormat().convertToRGBA(annotatedImage.getCpuImageMat(), annotatedImageVisualizer.getRGBA8Mat());
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
      depthImageSegmenter.destroy();
      colorImageVisualizer.destroy();
      depthImageVisualizer.destroy();
      detectionMaskVisualizer.destroy();
      erodedMaskVisualizer.destroy();
      segmentedDepthVisualizer.destroy();
      annotatedImageVisualizer.destroy();

      depthImageSegmenter.destroy();
      pointCloudExtractor.destroy();
      zed.close();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new RDXYOLOv8PipelineDemo();
   }
}
