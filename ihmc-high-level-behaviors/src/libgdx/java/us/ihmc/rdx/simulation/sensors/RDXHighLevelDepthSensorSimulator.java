package us.ihmc.rdx.simulation.sensors;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import com.badlogic.gdx.graphics.g3d.Renderable;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.utils.Array;
import com.badlogic.gdx.utils.Pool;
import imgui.ImGui;
import imgui.type.ImBoolean;
import imgui.type.ImDouble;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.BigVideoPacket;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.rdx.RDXPointCloudRendererOld;
import us.ihmc.rdx.imgui.ImGuiUniqueLabelMap;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDX3DScene;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.time.Instant;
import java.util.ArrayList;
import java.util.List;
import java.util.Set;
import java.util.function.LongSupplier;

import static us.ihmc.rdx.simulation.sensors.RDXLowLevelDepthSensorSimulator.FLOATS_PER_POINT;

public class RDXHighLevelDepthSensorSimulator extends RDXPanel
{
   private Mat depthImageMat;
   private final BytePointer compressedColorPointer = new BytePointer();
   private final BytePointer compressedDepthPointer = new BytePointer();;

   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final String sensorName;
   private final FramePose3D sensorPose = new FramePose3D();
   private final ReferenceFrame sensorFrame;
   private final Matrix4 gdxTransform = new Matrix4();
   private final RDXLowLevelDepthSensorSimulator depthSensorSimulator;
   private final LongSupplier timestampSupplier;
   private final CameraIntrinsics depthCameraIntrinsics;
   private final int imageWidth;
   private final int imageHeight;
   private final RDXPointCloudRendererOld pointCloudRenderer = new RDXPointCloudRendererOld();
   private final Mat rgba8Mat;
   private final Mat bgr8Mat;
   private final Mat depthDiscretizedMat;

   int depthSequenceNumber = 0;
   int colorSequenceNumber = 0;

   private ROS2Topic<ImageMessage> ros2DepthTopic;
   private ROS2Topic<ImageMessage> ros2ColorTopic;
   private final Mat rgb8Mat;

   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private RealtimeROS2Node realtimeROS2Node;
   private ROS2Publisher<BigVideoPacket> ros2VideoPublisher;
   private BigVideoPacket videoPacket;
   private BytePointer jpegImageBytePointer;
   private Mat yuv420Image;
   private IntPointer compressionParameters;

   private final Throttler throttler = new Throttler();
   private final ResettableExceptionHandlingExecutorService depthExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService colorExecutor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ResettableExceptionHandlingExecutorService colorROS2Executor = MissingThreadTools.newSingleThreadExecutor(getClass().getSimpleName(), true, 1);
   private final ImDouble publishRateHz = new ImDouble();
   private final ImBoolean debugCoordinateFrame = new ImBoolean(false);
   private ModelInstance coordinateFrame;
   private RigidBodyTransform sensorFrameToWorldTransform;

   private final ImGuiUniqueLabelMap labels = new ImGuiUniqueLabelMap(getClass());
   private final ImBoolean sensorEnabled = new ImBoolean(false);
   private final ImBoolean renderPointCloudDirectly = new ImBoolean(false);
   private final ImBoolean publishColorImageROS2 = new ImBoolean(false);
   private final ImBoolean publishDepthImageMessageROS2 = new ImBoolean(false);
   private final ImBoolean publishColorImageMessageROS2 = new ImBoolean(false);

   private final ImBoolean useSensorColor = new ImBoolean(false);
   private final ImBoolean colorBasedOnWorldZ = new ImBoolean(true);
   private final Color pointColorFromPicker = new Color();
   private final ImFloat pointSizeScale = new ImFloat(1.0f);
   private final float[] color = new float[] {1.0f, 1.0f, 1.0f, 1.0f};
   private final ImInt segmentationDivisor = new ImInt(8);
   private final List<Point3D> pointCloud = new ArrayList<>();

   private final Thread publishImagesThread;
   private volatile boolean publishImagesRunning;
   private ROS2Publisher<ImageMessage> depthImagePublisher;

   public RDXHighLevelDepthSensorSimulator(String sensorName,
                                           ReferenceFrame sensorFrame,
                                           LongSupplier timestampSupplier,
                                           double verticalFOV,
                                           int imageWidth,
                                           int imageHeight,
                                           double minRange,
                                           double maxRange,
                                           double noiseAmplitudeAtMinRange,
                                           double noiseAmplitudeAtMaxRange,
                                           boolean simulateL515Noise,
                                           double publishRateHz)
   {
      super(new ImGuiUniqueLabelMap(RDXHighLevelDepthSensorSimulator.class).get(sensorName + " Simulator"));
      this.sensorName = sensorName;
      setRenderMethod(this::renderImGuiWidgets);
      depthSensorSimulator = new RDXLowLevelDepthSensorSimulator(sensorName,
                                                                 verticalFOV,
                                                                 imageWidth,
                                                                 imageHeight,
                                                                 minRange,
                                                                 maxRange,
                                                                 noiseAmplitudeAtMinRange,
                                                                 noiseAmplitudeAtMaxRange,
                                                                 simulateL515Noise);

      this.sensorFrame = sensorFrame;
      this.timestampSupplier = timestampSupplier;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.publishRateHz.set(publishRateHz);

      LogTools.info("Capping depth sensor simulator to 8Hz.");
      this.publishRateHz.set(8.0);

      pointCloudRenderer.create(imageWidth * imageHeight);

      depthSensorSimulator.create(pointCloudRenderer.getVertexBuffer());
      addChild(depthSensorSimulator.getDepthPanel());
      addChild(depthSensorSimulator.getColorPanel());

      if (debugCoordinateFrame.get())
         coordinateFrame = RDXModelBuilder.createCoordinateFrameInstance(0.2);

      depthCameraIntrinsics = new CameraIntrinsics();
      updateCameraPinholeBrown();

      rgba8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC4, new BytePointer(depthSensorSimulator.getColorRGBA8Buffer()));
      rgb8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3, new BytePointer(depthSensorSimulator.getColorRGBA8Buffer()));
      bgr8Mat = new Mat(imageHeight, imageWidth, opencv_core.CV_8UC3);
      depthDiscretizedMat = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);

      publishImagesThread = new Thread(() ->
      {
         publishImagesRunning = true;

         while (publishImagesRunning)
         {
            publishImages();

            long sleepTime = (long) (Conversions.hertzToSeconds(publishRateHz) * 1000);
            ThreadTools.sleep(sleepTime);
         }
      }, getClass().getSimpleName() + "-ROSPublisher");
      publishImagesThread.start();
   }

   public void setupForROS2ImageMessages(ROS2Node ros2Node, ROS2Topic<ImageMessage> ros2DepthTopic, ROS2Topic<ImageMessage> ros2ColorTopic)
   {
      this.ros2Node = ros2Node;
      this.ros2Helper = new ROS2Helper(ros2Node);

      depthImagePublisher = ros2Node.createPublisher(ros2DepthTopic);

      this.ros2DepthTopic = ros2DepthTopic;
      this.ros2ColorTopic = ros2ColorTopic;

      this.depthImageMat = new Mat();
      this.yuv420Image = new Mat();
   }

   public void setupForROS2Color(ROS2Topic<BigVideoPacket> ros2VideoTopic)
   {
      // A Realtime ROS 2 node is required for video streaming in order to get stable performance.
      realtimeROS2Node = new ROS2NodeBuilder().buildRealtime(StringTools.titleToSnakeCase(sensorName) + "_video");
      ros2VideoPublisher = realtimeROS2Node.createPublisher(ros2VideoTopic);
      realtimeROS2Node.spin();
      videoPacket = new BigVideoPacket();
      yuv420Image = new Mat();
      jpegImageBytePointer = new BytePointer();
      compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);
   }

   private long lastUpdateTimeMs;
   public void render(RDX3DScene scene)
   {
      if (sensorEnabled.get())
      {
         if (sensorFrame != null)
            LibGDXTools.toLibGDX(sensorFrame.getTransformToWorldFrame(), gdxTransform);
         else
            LibGDXTools.toLibGDX(sensorFrameToWorldTransform, gdxTransform);

         depthSensorSimulator.setCameraWorldTransform(gdxTransform);

         if (coordinateFrame != null)
            coordinateFrame.transform.set(gdxTransform);

         long now = System.currentTimeMillis();
         long waitPeriodMs = (long) (Conversions.hertzToSeconds(publishRateHz.get()) * 1000);

         if (now - lastUpdateTimeMs > waitPeriodMs)
         {
            if (renderPointCloudDirectly.get())
            {
               LibGDXTools.toLibGDX(color, pointColorFromPicker);
               Color pointColor = useSensorColor.get() ? null : pointColorFromPicker;
               depthSensorSimulator.render(scene, colorBasedOnWorldZ.get(), pointColor, pointSizeScale.get());
               pointCloudRenderer.updateMeshFastest(imageWidth * imageHeight);
            }
            else
            {
               depthSensorSimulator.render(scene);
            }

            lastUpdateTimeMs = now;
         }
      }
   }

   public void publishImages()
   {
      double publishPeriod = Conversions.hertzToSeconds(publishRateHz.get());
      if (throttler.run(publishPeriod))
      {
         if (realtimeROS2Node != null)
         {
            if (publishColorImageROS2.get())
               publishColorImageROS2();
         }

         if (ros2Node != null)
         {
            if (publishDepthImageMessageROS2.get())
               publishROS2DepthImageMessage();
            if (publishColorImageMessageROS2.get())
               publishROS2ColorImageMessage();
         }
      }
   }

   private void publishColorImageROS2()
   {
      if (!colorROS2Executor.isExecuting())
      {
         colorROS2Executor.execute(() ->
         {
            long timestamp = timestampSupplier == null ? System.nanoTime() : timestampSupplier.getAsLong();
            Instant now = Instant.now();
            videoPacket.setAcquisitionTimeSecondsSinceEpoch(now.getEpochSecond());
            videoPacket.setAcquisitionTimeAdditionalNanos(now.getNano());

            opencv_imgproc.cvtColor(rgba8Mat, yuv420Image, opencv_imgproc.COLOR_RGBA2YUV_I420);
            opencv_imgcodecs.imencode(".jpg", yuv420Image, jpegImageBytePointer, compressionParameters);

            byte[] heapByteArrayData = new byte[jpegImageBytePointer.asBuffer().remaining()];
            jpegImageBytePointer.asBuffer().get(heapByteArrayData);
            videoPacket.getData().resetQuick();
            videoPacket.getData().add(heapByteArrayData);
            ros2VideoPublisher.publish(videoPacket);
         });
      }
   }

   private void updateCameraPinholeBrown()
   {
      depthCameraIntrinsics.setFx(depthSensorSimulator.getFocalLengthPixels().get());
      depthCameraIntrinsics.setFy(depthSensorSimulator.getFocalLengthPixels().get());
//      depthCameraIntrinsics.setSkew(0.0);
      depthCameraIntrinsics.setCx(depthSensorSimulator.getPrincipalOffsetXPixels().get());
      depthCameraIntrinsics.setCy(depthSensorSimulator.getPrincipalOffsetYPixels().get());
   }

   public void renderImGuiWidgets()
   {
      ImGui.text("Resolution: " + imageWidth + " x " + imageHeight);
      ImGui.checkbox(labels.get("Sensor Enabled"), sensorEnabled);
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Show frame graphic"), debugCoordinateFrame);
      ImGui.text("Render:");
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Point cloud"), renderPointCloudDirectly);
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Depth video"), getLowLevelSimulator().getDepthPanel().getIsShowing());
      ImGui.sameLine();
      ImGui.checkbox(labels.get("Color video"), getLowLevelSimulator().getColorPanel().getIsShowing());
      boolean publishing = realtimeROS2Node != null;
      if (publishing)
      {
         ImGui.text("Publish:");
         ImGui.sameLine();
         ImGui.text("(" + publishRateHz.get() + " Hz)");
         if (ros2DepthTopic != null)
         {
            ImGui.checkbox(labels.get("ROS 2 Depth image (" + ros2DepthTopic + ")"), publishDepthImageMessageROS2);
         }
         if (ros2ColorTopic != null)
         {
            ImGui.checkbox(labels.get("ROS 2 Color image (" + ros2ColorTopic + ")"), publishColorImageMessageROS2);
         }
         if (realtimeROS2Node != null)
         {
            ImGui.sameLine();
            ImGui.checkbox(labels.get("Color image (ROS 2)"), publishColorImageROS2);
         }
      }
      ImGui.checkbox("Use Sensor Color", useSensorColor);
      ImGui.sameLine();
      ImGui.checkbox("Color based on world Z", colorBasedOnWorldZ);
      ImGui.sliderFloat("Point scale", pointSizeScale.getData(), 0.0f, 2.0f);
      if (ImGui.collapsingHeader(labels.get("Color tuner")))
      {
         ImGui.colorPicker4("Color", color);
      }
      if (ImGui.collapsingHeader(labels.get("Low level settings")))
      {
         depthSensorSimulator.renderTuningSliders();
      }
   }

   public void publishROS2DepthImageMessage()
   {
      if (!depthExecutor.isExecuting() && publishDepthImageMessageROS2.get())
      {
         depthExecutor.execute(() -> {

            OpenCVTools.convertFloatToShort(depthSensorSimulator.getMetersDepthOpenCVMat(),
                                            depthImageMat, 1000.0, 0.0);

            depthImageMessage.setImageHeight(depthSensorSimulator.getImageHeight());
            depthImageMessage.setImageWidth(depthSensorSimulator.getImageWidth());
            depthImageMessage.setFocalLengthXPixels(depthSensorSimulator.getFocalLengthPixels().get());
            depthImageMessage.setFocalLengthYPixels(depthSensorSimulator.getFocalLengthPixels().get());
            depthImageMessage.setPrincipalPointXPixels(depthSensorSimulator.getPrincipalOffsetXPixels().get());
            depthImageMessage.setPrincipalPointYPixels(depthSensorSimulator.getPrincipalOffsetYPixels().get());

            Instant now = Instant.now();
            sensorPose.setToZero(sensorFrame);
            sensorPose.changeFrame(ReferenceFrame.getWorldFrame());
            OpenCVTools.compressImagePNG(depthImageMat, compressedDepthPointer);

            PerceptionMessageTools.publishCompressedDepthImage(compressedDepthPointer, depthImageMessage, depthImagePublisher, sensorPose, now, depthSequenceNumber++,
                                                                   depthSensorSimulator.getImageHeight(), depthSensorSimulator.getImageWidth(), 0.001f);

         });
      }
   }

   public void publishROS2ColorImageMessage()
   {
      if (!colorExecutor.isExecuting() && publishColorImageMessageROS2.get())
      {
         colorExecutor.execute(() -> {

            colorImageMessage.setImageHeight(depthSensorSimulator.getImageHeight());
            colorImageMessage.setImageWidth(depthSensorSimulator.getImageWidth());
            colorImageMessage.setFocalLengthXPixels(depthSensorSimulator.getFocalLengthPixels().get());
            colorImageMessage.setFocalLengthYPixels(depthSensorSimulator.getFocalLengthPixels().get());
            colorImageMessage.setPrincipalPointXPixels(depthSensorSimulator.getPrincipalOffsetXPixels().get());
            colorImageMessage.setPrincipalPointYPixels(depthSensorSimulator.getPrincipalOffsetYPixels().get());

            opencv_imgproc.cvtColor(rgba8Mat, rgb8Mat, opencv_imgproc.COLOR_RGBA2RGB);

            Instant now = Instant.now();
            OpenCVTools.compressRGBImageJPG(rgb8Mat, yuv420Image, compressedColorPointer);
            PerceptionMessageTools.publishJPGCompressedColorImage(compressedColorPointer, ros2ColorTopic, colorImageMessage, ros2Helper, sensorPose, now, colorSequenceNumber++,
                                                                  depthSensorSimulator.getImageHeight(), depthSensorSimulator.getImageWidth(), 0.001f);
         });
      }

   }

   public List<Point3D> getPointCloud()
   {
      pointCloud.clear();
      for (int i = 0; i < depthSensorSimulator.getNumberOfPoints()
                      && (FLOATS_PER_POINT * i + 2) < depthSensorSimulator.getPointCloudBuffer().limit(); i++)
      {
         float x = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i);
         float y = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i + 1);
         float z = depthSensorSimulator.getPointCloudBuffer().get(FLOATS_PER_POINT * i + 2);
         pointCloud.add(new Point3D(x, y, z));
      }

      return pointCloud;
   }

   public RawImage createRawColorImageBGR()
   {
      CameraIntrinsics intrinsics = depthSensorSimulator.getCameraIntrinsics();

      opencv_imgproc.cvtColor(rgba8Mat, bgr8Mat, opencv_imgproc.COLOR_RGBA2BGR);

      return RawImage.createWithBGRImage(bgr8Mat, intrinsics, sensorPose, Instant.now(), depthSensorSimulator.getSequenceNumber());
   }

   public RawImage createRawDepthImageDiscretized()
   {
      CameraIntrinsics intrinsics = depthSensorSimulator.getCameraIntrinsics();

      float discretization = 0.001f;
      OpenCVTools.convertFloatToShort(depthSensorSimulator.getMetersDepthOpenCVMat(), depthDiscretizedMat, 1.0f / discretization, 0.0);

      return RawImage.createWith16BitDepth(depthDiscretizedMat,
                                           intrinsics,
                                           sensorPose,
                                           Instant.now(),
                                           depthSensorSimulator.getSequenceNumber(),
                                           discretization);
   }

   public void dispose()
   {
      publishImagesRunning = false;
      try
      {
         publishImagesThread.join();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }

      if (realtimeROS2Node != null)
         realtimeROS2Node.destroy();
      depthExecutor.destroy();
      colorExecutor.destroy();
      colorROS2Executor.destroy();
      depthSensorSimulator.dispose();
   }

   public void getRenderables(Array<Renderable> renderables, Pool<Renderable> pool, Set<RDXSceneLevel> sceneLevelsToRender)
   {
      if (sceneLevelsToRender.contains(RDXSceneLevel.MODEL))
      {
         if (renderPointCloudDirectly.get())
            pointCloudRenderer.getRenderables(renderables, pool);
      }
      if (sceneLevelsToRender.contains(RDXSceneLevel.VIRTUAL))
      {
         if (debugCoordinateFrame.get())
         {
            if (coordinateFrame == null)
               coordinateFrame = RDXModelBuilder.createCoordinateFrameInstance(0.2);
            coordinateFrame.getRenderables(renderables, pool);
         }
         depthSensorSimulator.getVirtualRenderables(renderables, pool);
      }
   }

   public void setSensorEnabled(boolean sensorEnabled)
   {
      this.sensorEnabled.set(sensorEnabled);
   }

   public void setRenderPointCloudDirectly(boolean renderPointCloudDirectly)
   {
      this.renderPointCloudDirectly.set(renderPointCloudDirectly);
   }

   public void setPointSizeScale(double size)
   {
      pointSizeScale.set((float) size);
   }

   public void setRenderDepthVideoDirectly(boolean renderDepthVideoDirectly)
   {
      getLowLevelSimulator().getDepthPanel().getIsShowing().set(renderDepthVideoDirectly);
   }

   public void setRenderColorVideoDirectly(boolean renderColorVideoDirectly)
   {
      getLowLevelSimulator().getColorPanel().getIsShowing().set(renderColorVideoDirectly);
   }

   public void setPublishColorImageROS2(boolean publish)
   {
      publishColorImageROS2.set(publish);
   }

   public void setPublishDepthImageMessageROS2(boolean publish)
   {
      publishDepthImageMessageROS2.set(publish);
   }

   public void setPublishColorImageMessageROS2(boolean publish)
   {
      publishColorImageMessageROS2.set(publish);
   }

   public void setUseSensorColor(boolean useSensorColor)
   {
      this.useSensorColor.set(useSensorColor);
   }

   public void setDebugCoordinateFrame(boolean debugCoordinateFrame)
   {
      this.debugCoordinateFrame.set(debugCoordinateFrame);
   }

   public void setSensorFrameToWorldTransform(RigidBodyTransform sensorFrameToWorldTransform)
   {
      this.sensorFrameToWorldTransform = sensorFrameToWorldTransform;
   }

   public RDXLowLevelDepthSensorSimulator getLowLevelSimulator()
   {
      return depthSensorSimulator;
   }

   public CameraIntrinsics getDepthCameraIntrinsics()
   {
      return depthCameraIntrinsics;
   }

   public CameraIntrinsics getCopyOfCameraParameters()
   {
      return new CameraIntrinsics(imageHeight,
                                  imageWidth,
                                  getLowLevelSimulator().getFocalLengthPixels().get(),
                                  getLowLevelSimulator().getFocalLengthPixels().get(),
                                  getLowLevelSimulator().getPrincipalOffsetXPixels().get(),
                                  getLowLevelSimulator().getPrincipalOffsetYPixels().get());
   }

   public Color getPointColorFromPicker()
   {
      return pointColorFromPicker;
   }

   public ReferenceFrame getSensorFrame()
   {
      return sensorFrame;
   }
}
