package us.ihmc.rdx;

import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_core.Size;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXImageVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.sensors.zed.ZEDSVOPlaybackSensor;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.zed.global.zed;

public class RDXImageVisualizerDemo
{
   private static final String SVO_FILE = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve("20240715_103234_ZEDRecording_NewONRCourseWalk.svo2").toAbsolutePath().toString();

   private final RDXImageVisualizer visualizer;

   private final ROS2Node ros2Node;

   private final ZEDSVOPlaybackSensor zedSensor;
   private final RepeatingTaskThread zedGrabThread;

   private RDXImageVisualizerDemo()
   {
      visualizer = new RDXImageVisualizer("ZED Color", "Visualizer Panel", false);

      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName().toLowerCase());

      zedSensor = new ZEDSVOPlaybackSensor(new ROS2Helper(ros2Node), 0, ZEDModelData.ZED_2I, zed.SL_DEPTH_MODE_PERFORMANCE, SVO_FILE);
      zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);

      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            visualizer.create();
            visualizer.setActive(true);

            baseUI.getImGuiPanelManager().addPanel(visualizer.getPanel());
            baseUI.create();

            zedSensor.run(true);
            zedGrabThread.startRepeating();
         }

         @Override
         public void render()
         {
            visualizer.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            destroy();
            baseUI.dispose();
            visualizer.destroy();
         }
      });
   }

   private void zedGrabThread() throws InterruptedException
   {
      zedSensor.waitForGrab();
      RawImage colorImage = zedSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
//      Mat resized = new Mat();
//      opencv_imgproc.resize(colorImage.getCpuImageMat(), resized, new Size(colorImage.getWidth() * 2, colorImage.getHeight() * 2));
//      visualizer.setImage(RDXBaseUI.getInstance().getRenderIndex() % 100 < 50 ? colorImage.getCpuImageMat() : resized, opencv_imgproc.COLOR_BGR2RGBA);
      visualizer.setImage(colorImage.getCpuImageMat(), opencv_imgproc.COLOR_BGR2RGBA);
      colorImage.release();
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
