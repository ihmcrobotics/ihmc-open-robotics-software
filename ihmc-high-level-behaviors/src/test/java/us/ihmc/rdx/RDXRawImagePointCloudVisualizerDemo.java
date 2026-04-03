package us.ihmc.rdx;

import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensors.zed.ZEDImageSensor;
import us.ihmc.sensors.zed.ZEDModelData;
import us.ihmc.zed.global.zed;

import java.time.Instant;
import java.util.LinkedList;
import java.util.Queue;

public class RDXRawImagePointCloudVisualizerDemo
{
   private RDXRawImagePointCloudVisualizer pointCloudVisualizer;

   private ZEDImageSensor zedImageSensor;
   private final RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);

   private final Queue<RawImage> depthImageQueue = new LinkedList<>();
   private final ImFloat depthImageDelay = new ImFloat(0.0f);
   private final Queue<RawImage> colorImageQueue = new LinkedList<>();
   private final ImFloat colorImageDelay = new ImFloat(0.0f);

   private RDXPose3DGizmo sensorPoseGizmo;

   private RDXRawImagePointCloudVisualizerDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            sensorPoseGizmo = new RDXPose3DGizmo();
            sensorPoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);

            zedImageSensor = new ZEDImageSensor(0, 0, ZEDModelData.ZED_2I, zed.SL_INPUT_TYPE_USB, zed.SL_DEPTH_MODE_NEURAL_LIGHT, zed.SL_RESOLUTION_VGA, 100);
            zedImageSensor.run(true);

            pointCloudVisualizer = new RDXRawImagePointCloudVisualizer("ZED Point Cloud");
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudVisualizer);

            baseUI.getImGuiPanelManager().addPanel("Options", this::renderOptions);
            zedGrabThread.startRepeating();
         }

         @Override
         public void render()
         {
            pointCloudVisualizer.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderOptions()
         {
            pointCloudVisualizer.renderImGuiWidgets();

            ImGui.separator();

            ImGui.sliderFloat("Depth Delay", depthImageDelay.getData(), 0.0f, 10.0f);
            ImGui.sliderFloat("Color Delay", colorImageDelay.getData(), 0.0f, 10.0f);
         }

         @Override
         public void dispose()
         {
            zedGrabThread.blockingKill();
            zedImageSensor.close();
            pointCloudVisualizer.destroy();
            colorImageQueue.forEach(RawImage::release);
            depthImageQueue.forEach(RawImage::release);
            baseUI.dispose();
         }
      });
   }

   private void zedGrabThread() throws InterruptedException
   {
      zedImageSensor.waitForGrab();

      RawImage colorImage = zedImageSensor.getImage(ZEDImageSensor.LEFT_COLOR_IMAGE_KEY);
      RawImage depthImage = zedImageSensor.getImage(ZEDImageSensor.DEPTH_IMAGE_KEY);

      colorImageQueue.add(colorImage);
      depthImageQueue.add(depthImage);

      Instant now = Instant.now();

      RawImage oldestColorImage = colorImageQueue.peek();
      while (oldestColorImage != null && TimeTools.secondsBetween(oldestColorImage.getAcquisitionTime(), now) > colorImageDelay.get())
      {
         pointCloudVisualizer.setColorImage(oldestColorImage);
         colorImageQueue.remove().release();
         oldestColorImage = colorImageQueue.peek();
      }

      RawImage oldestDepthImage = depthImageQueue.peek();
      while (oldestDepthImage != null && TimeTools.secondsBetween(oldestDepthImage.getAcquisitionTime(), now) > depthImageDelay.get())
      {
         pointCloudVisualizer.setDepthImage(oldestDepthImage);
         depthImageQueue.remove().release();
         oldestDepthImage = depthImageQueue.peek();
      }
   }

   public static void main(String[] args)
   {
      new RDXRawImagePointCloudVisualizerDemo();
   }
}