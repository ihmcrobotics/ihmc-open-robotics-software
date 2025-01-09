package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDADepthColorizer;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.streaming.ROS2SRTSensorStreamer;
import us.ihmc.perception.streaming.ROS2SRTVideoSubscriber;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;

public class RDXDepthStreamingDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2").toAbsolutePath().toString();

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(RDXDepthStreamingDemo.class.getSimpleName());
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDSVOPlaybackSensor zed = new ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, SVO_FILE);

   private final CUDADepthColorizer depthColorizer = new CUDADepthColorizer();
   private final ROS2SRTSensorStreamer sensorStreamer = new ROS2SRTSensorStreamer(ros2Node);
   private final ROS2SRTVideoSubscriber depthSubscriber = new ROS2SRTVideoSubscriber(ros2Helper, PerceptionAPI.SRT_ZED_DEPTH_STREAM_STATUS, PixelFormat.YUV444P);
   private final ROS2SRTVideoSubscriber colorSubscriber = new ROS2SRTVideoSubscriber(ros2Helper, PerceptionAPI.SRT_ZED_LEFT_COLOR_STREAM_STATUS, PixelFormat.RGBA8);

   private final RepeatingTaskThread zedPublishThread = new RepeatingTaskThread("ZEDPublish", this::publishZED);

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass().getSimpleName());
   private final RDXOpenCVVideoVisualizer sentDepthVisualizer = new RDXOpenCVVideoVisualizer("Sent Colorized Depth", "Sent Colorized Depth", false);
   private final RDXOpenCVVideoVisualizer receivedDepthVisualizer = new RDXOpenCVVideoVisualizer("Received Colorized Depth", "Received Colorized Depth", false);
   private final RDXRawImagePointCloudVisualizer pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("De-Colorized Point Cloud");

   public RDXDepthStreamingDemo() throws Exception
   {
      zed.useTrackedPose(true);
      zed.run(true);
      zedPublishThread.startRepeating();

      depthSubscriber.addNewFrameConsumer(this::receiveColorizedDepth);
      depthSubscriber.subscribe();

      colorSubscriber.addNewFrameConsumer(this::receiveColorImage);
      colorSubscriber.subscribe();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            sentDepthVisualizer.setActive(true);
            receivedDepthVisualizer.setActive(true);
            pointCloudVisualizer.setActive(true);

            baseUI.getImGuiPanelManager().addPanel(sentDepthVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel(receivedDepthVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel("Visualizer Settings", pointCloudVisualizer::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            baseUI.create();
         }

         @Override
         public void render()
         {
            sentDepthVisualizer.update();
            receivedDepthVisualizer.update();
            pointCloudVisualizer.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            pointCloudVisualizer.destroy();
            baseUI.dispose();
            destroy();
         }
      });
   }

   public void publishZED()
   {
      try (Mat rgbMat = new Mat())
      {
         zed.waitForGrab();
         RawImage depthImage = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
         RawImage colorImage = zed.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);

         GpuMat colorizedDepth = depthColorizer.colorizeDepth(depthImage.getGpuImageMat());
         RawImage colorizedDepthImage = depthImage.replaceImage(colorizedDepth, PixelFormat.YUV444P);

         opencv_imgproc.cvtColor(colorizedDepthImage.getCpuImageMat(), rgbMat, opencv_imgproc.COLOR_YUV2RGB);
         sentDepthVisualizer.setImage(rgbMat);

         sensorStreamer.sendFrame(PerceptionAPI.SRT_ZED_DEPTH_STREAM_STATUS, colorizedDepthImage);
         sensorStreamer.sendFrame(PerceptionAPI.SRT_ZED_LEFT_COLOR_STREAM_STATUS, colorImage);

         colorizedDepthImage.release();
         depthImage.release();
         colorImage.release();
      } catch (InterruptedException ignored) {}
   }

   public void receiveColorizedDepth(RawImage colorizedDepth)
   {
      colorizedDepth.get();

      Mat rgbMat = new Mat();
      opencv_imgproc.cvtColor(colorizedDepth.getCpuImageMat(), rgbMat, opencv_imgproc.COLOR_YUV2RGB);
      receivedDepthVisualizer.setImage(rgbMat);

      GpuMat deColorizedDepth = depthColorizer.deColorizeDepth(colorizedDepth.getGpuImageMat());
      RawImage deColorizedImage = colorizedDepth.replaceImage(deColorizedDepth);

      pointCloudVisualizer.setDepthImage(deColorizedImage);

      rgbMat.close();
      deColorizedImage.release();
      colorizedDepth.release();
   }

   public void receiveColorImage(RawImage colorImage)
   {
      colorImage.get();

      pointCloudVisualizer.setColorImage(colorImage);

      colorImage.release();
   }

   public void destroy()
   {
      zedPublishThread.blockingKill();
      zed.close();
      depthColorizer.destroy();
      sensorStreamer.destroy();
      depthSubscriber.destroy();
      colorSubscriber.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new RDXDepthStreamingDemo();
   }
}
