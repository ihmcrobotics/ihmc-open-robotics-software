package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_cudawarping;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
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

import static org.bytedeco.ffmpeg.global.avutil.AV_PIX_FMT_YUV444P;

public class RDXDepthStreamingDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2").toAbsolutePath().toString();

   private final ROS2Node ros2Node = new ROS2NodeBuilder().build(RDXDepthStreamingDemo.class.getSimpleName());
   private final ROS2Helper ros2Helper = new ROS2Helper(ros2Node);

   private final ZEDSVOPlaybackSensor zed = new ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_2, SVO_FILE);

   private final CUDADepthColorizer depthColorizer = new CUDADepthColorizer();
   private final ROS2SRTVideoStreamer videoStreamer = new ROS2SRTVideoStreamer(ros2Node, PerceptionAPI.SRT_ZED_DEPTH_STREAM_STATUS);
   private final ROS2SRTVideoSubscriber videoSubscriber = new ROS2SRTVideoSubscriber(ros2Helper, PerceptionAPI.SRT_ZED_DEPTH_STREAM_STATUS, PixelFormat.YUV444P);

   private final RepeatingTaskThread zedPublishThread = new RepeatingTaskThread("ZEDPublish", this::publishZED);

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass().getSimpleName());
   private final RDXOpenCVVideoVisualizer sentColorVisualizer = new RDXOpenCVVideoVisualizer("Sent Colorized Depth", "Sent Colorized Depth", false);
   private final RDXOpenCVVideoVisualizer receivedColorVisualizer = new RDXOpenCVVideoVisualizer("Received Colorized Depth", "Received Colorized Depth", false);
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
            sentColorVisualizer.create();
            sentColorVisualizer.setActive(true);

            receivedColorVisualizer.create();
            receivedColorVisualizer.setActive(true);

            pointCloudVisualizer.create();
            pointCloudVisualizer.setActive(true);

            baseUI.getImGuiPanelManager().addPanel(sentColorVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel(receivedColorVisualizer.getPanel());
            baseUI.getImGuiPanelManager().addPanel("Visualizer Settings", pointCloudVisualizer::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);
            baseUI.create();
         }

         @Override
         public void render()
         {
            sentColorVisualizer.update();
            receivedColorVisualizer.update();
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
      try
      {
         zed.waitForGrab();
         RawImage depthImage = zed.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

         GpuMat colorizedDepth = depthColorizer.colorizeDepth(depthImage.getGpuImageMat());
         RawImage colorizedImage = depthImage.replaceImage(colorizedDepth, PixelFormat.BGR8);

         sentColorVisualizer.updateImageDimensions(colorizedImage.getWidth(), colorizedImage.getHeight());
         opencv_imgproc.cvtColor(colorizedImage.getCpuImageMat(), sentColorVisualizer.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA);

         GpuMat smallerColorizedDepth = new GpuMat();
         opencv_cudawarping.resize(colorizedDepth, smallerColorizedDepth, new Size(colorizedImage.getWidth() / 2, colorizedImage.getHeight() / 2));
         RawImage smallerImage = new RawImage(null,
                                              smallerColorizedDepth,
                                              PixelFormat.BGR8,
                                              new CameraIntrinsics(smallerColorizedDepth.rows(),
                                                                   smallerColorizedDepth.cols(),
                                                                   colorizedImage.getFocalLengthX() / 2,
                                                                   colorizedImage.getFocalLengthY() / 2,
                                                                   colorizedImage.getPrincipalPointX() / 2,
                                                                   colorizedImage.getPrincipalPointY() / 2),
                                              colorizedImage.getCameraModel(),
                                              colorizedImage.getPose(),
                                              colorizedImage.getAcquisitionTime(),
                                              colorizedImage.getSequenceNumber(),
                                              colorizedImage.getDepthDiscretization());

         if (!videoStreamer.isInitialized())
            videoStreamer.initializeForColor(smallerImage, AV_PIX_FMT_YUV444P);

         videoStreamer.sendFrame(smallerImage);

         smallerImage.release();
         colorizedImage.release();
         depthImage.release();
      } catch (InterruptedException ignored) {}
   }

   public void receiveColorizedDepth(RawImage colorizedDepth)
   {
      colorizedDepth.get();

      receivedColorVisualizer.updateImageDimensions(colorizedDepth.getWidth(), colorizedDepth.getHeight());
      opencv_imgproc.cvtColor(colorizedDepth.getCpuImageMat(), receivedColorVisualizer.getRGBA8Mat(), opencv_imgproc.COLOR_BGR2RGBA);

      GpuMat deColorizedDepth = depthColorizer.deColorizeDepth(colorizedDepth.getGpuImageMat());
      RawImage deColorizedImage = colorizedDepth.replaceImage(deColorizedDepth);

      pointCloudVisualizer.setDepthImage(deColorizedImage);

      deColorizedImage.release();
      colorizedDepth.release();
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
