package us.ihmc.rdx.perception;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.graphicsDescription.image.DepthImage;
import us.ihmc.log.LogTools;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.sensors.zed.ROS2ZEDSVOPlaybackSensor;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

public class RDXHeightMapLogPlayer
{
   private static final String SVO_FILE = System.getProperty("user.home") + "/Downloads/heightmap_test.svo2";

   private final ROS2ZEDSVOPlaybackSensor zedPlaybackSensor;
   private final RDXZEDSVORecorderPanel zedSVOPanel;

   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;

   private final RDXBaseUI baseUI;

   private final RDXRawImagePointCloudVisualizer zedPointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud", true);

   public RDXHeightMapLogPlayer()
   {
      ros2Node = new ROS2NodeBuilder().build(getClass().getSimpleName());
      ros2Helper = new ROS2Helper(ros2Node);

      baseUI = new RDXBaseUI();

      zedPlaybackSensor = new ROS2ZEDSVOPlaybackSensor(ros2Helper, 0, ZEDModelData.ZED_X_MINI, zed.SL_DEPTH_MODE_NEURAL_LIGHT, SVO_FILE);
      zedSVOPanel = new RDXZEDSVORecorderPanel(ros2Helper);

      RepeatingTaskThread imageThread = new RepeatingTaskThread("Image Thread", this::runMethod);

      imageThread.startRepeating();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getImGuiPanelManager().addPanel("ZED SVO", zedSVOPanel::render);
            zedPointCloudVisualizer.create();

            zedPointCloudVisualizer.setActive(true);
            zedPlaybackSensor.run(true);

            baseUI.getImGuiPanelManager().addPanel("ZED Point Cloud", zedPointCloudVisualizer::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider(zedPointCloudVisualizer);
         }

         @Override
         public void render()
         {
            zedPointCloudVisualizer.update();
            zedSVOPanel.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();

            zedPlaybackSensor.close();
            ros2Node.destroy();
         }
      });
   }

   private void runMethod()
   {
      try
      {
         zedPlaybackSensor.waitForGrab();
         LogTools.info("fashdfasd");
         RawImage depthImage = zedPlaybackSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);
         zedPlaybackSensor.registerImageQueue();
         RawImage colorImageLeft = zedPlaybackSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
         RawImage colorImageRight = zedPlaybackSensor.getImage(ZEDImageSensor.RIGHT_COLOR_IMAGE_KEY);

         zedPointCloudVisualizer.setColorImage(colorImageLeft);
         zedPointCloudVisualizer.setDepthImage(depthImage);

         depthImage.release();
         colorImageLeft.release();
         colorImageRight.release();
      }
      catch (InterruptedException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      RDXHeightMapLogPlayer heightMapLogPlayer = new RDXHeightMapLogPlayer();
   }
}
