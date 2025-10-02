package us.ihmc.rdx;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXOpenCVVideoVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.zed.global.zed;

public class RDXOpenCVVideoVisualizerDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2").toAbsolutePath().toString();

   private final RDXOpenCVVideoVisualizer rgbVisualizer;
   private final RDXOpenCVVideoVisualizer bgrVisualizer;
   private final RDXOpenCVVideoVisualizer yuvVisualizer;
   private final RDXOpenCVVideoVisualizer grayVisualizer;
   private final RDXOpenCVVideoVisualizer depthVisualizer;

   private final ROS2Node ros2Node;

   private final ROS2ZEDSVOPlaybackSensor zedSensor;
   private final RepeatingTaskThread zedGrabThread;

   private RDXOpenCVVideoVisualizerDemo()
   {
      rgbVisualizer = new RDXOpenCVVideoVisualizer("ZED Color", "RGB Panel", false);
      bgrVisualizer = new RDXOpenCVVideoVisualizer("ZED Color", "BGR Panel", false);
      yuvVisualizer = new RDXOpenCVVideoVisualizer("ZED Color", "YUV Panel", false);
      grayVisualizer = new RDXOpenCVVideoVisualizer("ZED Color", "GRAY Panel", false);
      depthVisualizer = new RDXOpenCVVideoVisualizer("ZED Depth", "Depth Panel", false);

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());

      zedSensor = new ROS2ZEDSVOPlaybackSensor(new ROS2Helper(ros2Node), 0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
      zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);

      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            rgbVisualizer.create();
            rgbVisualizer.setActive(true);
            baseUI.getImGuiPanelManager().addPanel(rgbVisualizer.getPanel());

            bgrVisualizer.create();
            bgrVisualizer.setActive(true);
            baseUI.getImGuiPanelManager().addPanel(bgrVisualizer.getPanel());

            yuvVisualizer.create();
            yuvVisualizer.setActive(true);
            baseUI.getImGuiPanelManager().addPanel(yuvVisualizer.getPanel());

            grayVisualizer.create();
            grayVisualizer.setActive(true);
            baseUI.getImGuiPanelManager().addPanel(grayVisualizer.getPanel());

            depthVisualizer.create();
            depthVisualizer.setActive(true);
            baseUI.getImGuiPanelManager().addPanel(depthVisualizer.getPanel());

            baseUI.getImGuiPanelManager().addPanel(rgbVisualizer.getPanel());
            baseUI.create();

            zedSensor.run(true);
            zedGrabThread.startRepeating();
         }

         @Override
         public void render()
         {
            rgbVisualizer.update();
            bgrVisualizer.update();
            yuvVisualizer.update();
            grayVisualizer.update();
            depthVisualizer.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            destroy();
            baseUI.dispose();
            rgbVisualizer.destroy();
            bgrVisualizer.destroy();
            yuvVisualizer.destroy();
            grayVisualizer.destroy();
            depthVisualizer.destroy();
         }
      });
   }

   private void zedGrabThread() throws InterruptedException
   {
      zedSensor.waitForGrab();
      RawImage colorImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      RawImage depthImage = zedSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

      RawImage rgbImage = RawImageTools.convertColor(colorImage, PixelFormat.RGB8);
      RawImage bgrImage = RawImageTools.convertColor(colorImage, PixelFormat.BGR8);
      RawImage yuvImage = RawImageTools.convertColor(colorImage, PixelFormat.YUV444P);
      RawImage grayImage = RawImageTools.convertColor(colorImage, PixelFormat.GRAY8);

      synchronized (rgbVisualizer)
      {
         rgbVisualizer.setImage(rgbImage.getCpuImageMat(), opencv_imgproc.COLOR_RGB2RGBA);
      }
      synchronized (rgbVisualizer)
      {
         bgrVisualizer.setImage(bgrImage.getCpuImageMat(), opencv_imgproc.COLOR_BGR2RGBA);
      }

      // Cannot convert YUV444P directly to RGBA. Gotta do YUV to RGB first
      Mat yuvRGB = new Mat();
      opencv_imgproc.cvtColor(yuvImage.getCpuImageMat(), yuvRGB, opencv_imgproc.COLOR_YUV2RGB);
      synchronized (yuvVisualizer)
      {
         yuvVisualizer.setImage(yuvRGB, opencv_imgproc.COLOR_RGB2RGBA);
      }

      synchronized (grayVisualizer)
      {
         grayVisualizer.setImage(grayImage.getCpuImageMat(), opencv_imgproc.COLOR_GRAY2RGBA);
      }

      // Gotta do funky clamping to render depth
      Mat clampedDepth = new Mat();
      opencv_core.normalize(depthImage.getCpuImageMat(), clampedDepth, 0.0, 255.0, opencv_core.NORM_MINMAX, opencv_core.CV_8U, null);
      synchronized (depthVisualizer)
      {
         depthVisualizer.setImage(clampedDepth, opencv_imgproc.COLOR_GRAY2RGBA);
      }

      colorImage.release();
      depthImage.release();
      rgbImage.release();
      bgrImage.release();
      yuvImage.release();
      grayImage.release();

      yuvRGB.close();
      clampedDepth.close();
   }

   private void destroy()
   {
      zedGrabThread.blockingKill();
      zedSensor.close();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new RDXOpenCVVideoVisualizerDemo();
   }
}
