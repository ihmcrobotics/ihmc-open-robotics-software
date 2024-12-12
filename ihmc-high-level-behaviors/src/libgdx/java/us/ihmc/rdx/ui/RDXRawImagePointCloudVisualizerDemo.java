package us.ihmc.rdx.ui;

import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudVisualizer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

public class RDXRawImagePointCloudVisualizerDemo
{
   private RDXRawImagePointCloudVisualizer pointCloudVisualizer;

   private ZEDColorDepthImageRetriever zed;
   RepeatingTaskThread zedGrabThread = new RepeatingTaskThread("ZEDGrabThread", this::zedGrabThread);
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
         }

         @Override
         public void dispose()
         {
            zed.destroy();
            pointCloudVisualizer.destroy();
            baseUI.dispose();
         }
      });
   }

   private void zedGrabThread()
   {
      RawImage colorImage = zed.getLatestRawColorImage(RobotSide.LEFT);
      RawImage depthImage = zed.getLatestRawDepthImage();

      // TODO: Add artificial delay to image setting
      pointCloudVisualizer.setColorImage(colorImage);
      pointCloudVisualizer.setDepthImage(depthImage);

      colorImage.release();
      depthImage.release();
   }

   public static void main(String[] args)
   {
      new RDXRawImagePointCloudVisualizerDemo();
   }
}
