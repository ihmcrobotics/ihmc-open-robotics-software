package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.cuda.CUDADepthColorizer;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.streaming.ROS2SRTVideoStreamer;
import us.ihmc.perception.streaming.ROS2SRTVideoSubscriber;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.ZEDImageSensor;
import us.ihmc.sensors.ZEDModelData;
import us.ihmc.sensors.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_BGR8;

public class RDXDepthStreamingDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2").toAbsolutePath().toString();

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(RDXDepthStreamingDemo.class.getSimpleName());
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDSVOPlaybackSensor zed = new ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, SVO_FILE);

   private final CUDADepthColorizer depthColorizer = new CUDADepthColorizer();
   private final ROS2SRTVideoStreamer videoStreamer = new ROS2SRTVideoStreamer(ros2Node, PerceptionAPI.SRT_ZED_DEPTH_STREAM_STATUS);
   private final ROS2SRTVideoSubscriber videoSubscriber = new ROS2SRTVideoSubscriber(ros2Helper, PerceptionAPI.SRT_ZED_DEPTH_STREAM_STATUS, PixelFormat.BGR8);

   private final RepeatingTaskThread zedPublishThread = new RepeatingTaskThread("ZEDPublish", this::publishZED);

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass().getSimpleName());
   private final RDXOpenCVVideoVisualizer imageVisualizer = new RDXOpenCVVideoVisualizer("Colorized Depth", "Colorized Depth", true);
   private final RDXRawImagePointCloudVisualizer pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("De-Colorized Point Cloud", false);

   public RDXDepthStreamingDemo()
   {
      zed.useTrackedPose(true);
      zed.run(true);
      zedPublishThread.startRepeating();

      videoSubscriber.addNewFrameConsumer(this::receiveColorizedDepth);
      videoSubscriber.subscribe();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            imageVisualizer.create();
            imageVisualizer.setActive(true);

            pointCloudVisualizer.create();
            pointCloudVisualizer.setActive(true);

            baseUI.getImGuiPanelManager().addPanel(imageVisualizer.getPanel());
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            baseUI.create();
         }

         @Override
         public void render()
         {
            imageVisualizer.update();
            pointCloudVisualizer.update();

            baseUI.renderBeforeOnScreenUI();

            pointCloudVisualizer.renderImGuiWidgets();

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

   public synchronized void publishZED()
   {
      try
      {
         zed.waitForGrab();
         RawImage depthImage = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
         GpuMat colorizedDepth = depthColorizer.colorizeDepth(depthImage.getGpuImageMat());
         RawImage colorizedImage = depthImage.replaceImage(colorizedDepth, PixelFormat.BGR8);

         if (!videoStreamer.isInitialized())
            videoStreamer.initializeForColor(colorizedImage, AV_PIX_FMT_BGR8);

         videoStreamer.sendFrame(colorizedImage);

         colorizedImage.release();
         depthImage.release();
      } catch (InterruptedException ignored) {}
   }

   public synchronized void receiveColorizedDepth(RawImage colorizedDepth)
   {
      colorizedDepth.get();

      imageVisualizer.updateImageDimensions(colorizedDepth.getWidth(), colorizedDepth.getHeight());
      opencv_imgproc.cvtColor(colorizedDepth.getCpuImageMat(), imageVisualizer.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA);

//      GpuMat deColorizedDepth = depthColorizer.deColorizeDepth(colorizedDepth.getGpuImageMat());
//      RawImage deColorizedImage = colorizedDepth.replaceImage(deColorizedDepth, PixelFormat.GRAY16);

//      pointCloudVisualizer.setDepthImage(deColorizedImage);

      colorizedDepth.release();
//      deColorizedImage.release();
   }

   public void destroy()
   {
      zedPublishThread.blockingKill();
      zed.close();
      depthColorizer.destroy();
      videoStreamer.destroy();
      videoSubscriber.destroy();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new RDXDepthStreamingDemo();
   }
}
