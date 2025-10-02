package us.ihmc.rdx;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.zed.global.zed;

public class RDXImageVisualizerDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2").toAbsolutePath().toString();

   private final RDXImageVisualizer rgbVisualizer;
   private final RDXImageVisualizer bgrVisualizer;
   private final RDXImageVisualizer yuvVisualizer;
   private final RDXImageVisualizer grayVisualizer;
   private final RDXImageVisualizer depthVisualizer;

   private final ROS2Node ros2Node;

   private final ROS2ZEDSVOPlaybackSensor zedSensor;
   private final RepeatingTaskThread zedGrabThread;

   private RDXImageVisualizerDemo()
   {
      rgbVisualizer = new RDXImageVisualizer("ZED Color", "RGB Panel", false);
      bgrVisualizer = new RDXImageVisualizer("ZED Color", "BGR Panel", false);
      yuvVisualizer = new RDXImageVisualizer("ZED Color", "YUV Panel", false);
      grayVisualizer = new RDXImageVisualizer("ZED Color", "GRAY Panel", false);
      depthVisualizer = new RDXImageVisualizer("ZED Depth", "Depth Panel", false);

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

      rgbVisualizer.setImage(rgbImage);
      bgrVisualizer.setImage(bgrImage);
      yuvVisualizer.setImage(yuvImage);
      grayVisualizer.setImage(grayImage);
      depthVisualizer.setImage(depthImage);

      colorImage.release();
      depthImage.release();
      rgbImage.release();
      bgrImage.release();
      yuvImage.release();
      grayImage.release();
   }

   private void destroy()
   {
      zedGrabThread.blockingKill();
      zedSensor.close();
      ros2Node.destroy();
   }

   public static void main(String[] args)
   {
      new RDXImageVisualizerDemo();
   }
}
