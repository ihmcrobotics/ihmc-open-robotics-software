package us.ihmc.rdx.ui;

import imgui.ImGui;
import imgui.type.ImFloat;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeTools;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

import java.time.Instant;
import java.util.LinkedList;
import java.util.Queue;

public class RDXRawImagePointCloudVisualizerDemo
{
   private RDXRawImagePointCloudVisualizer pointCloudVisualizer;

   private ZEDColorDepthImageRetriever zed;
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

            zed = new ZEDColorDepthImageRetriever(0, sensorPoseGizmo::getGizmoFrame, () -> true, () -> true);
            zed.start();

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
            zed.destroy();
            pointCloudVisualizer.destroy();
            colorImageQueue.forEach(RawImage::release);
            depthImageQueue.forEach(RawImage::release);
            baseUI.dispose();
         }
      });
   }

   private void zedGrabThread()
   {
      RawImage colorImage = zed.getLatestRawColorImage(RobotSide.LEFT);
      RawImage depthImage = zed.getLatestRawDepthImage();

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
