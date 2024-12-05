package us.ihmc.rdx.simulation;

import us.ihmc.commons.thread.Throttler;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.DepthSensorDemoObjectsModel;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXMatImagePanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.sensors.RDXSensorSimulator;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer.ColoringMethod;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer.InputMethod;

public class RDXSensorSimulatorDemo
{
   private static final int WIDTH = 1280;
   private static final int HEIGHT = 720;
   private static final float FOV = 70.0f;
   private static final float MIN_RANGE = 0.2f;
   private static final float MAX_RANGE = 20.0f;

   private final Throttler throttler = new Throttler().setFrequency(30.0);
   private final RDXSensorSimulator sensorSimulator;
   private RDXPose3DGizmo sensorPoseGizmo;

   private final RDXRawImagePointCloudRenderer pointCloudRenderer = new RDXRawImagePointCloudRenderer(InputMethod.DEPTH_IMAGE);

   private RDXMatImagePanel imagePanel;

   public RDXSensorSimulatorDemo()
   {
      sensorSimulator = new RDXSensorSimulator(WIDTH, HEIGHT, FOV, MIN_RANGE, MAX_RANGE);

      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(new DepthSensorDemoObjectsModel().newInstance(), RDXSceneLevel.GROUND_TRUTH);

            imagePanel = new RDXMatImagePanel("Simulated color", WIDTH, HEIGHT, false);
            baseUI.getImGuiPanelManager().addPanel(imagePanel.getImagePanel());

            sensorSimulator.create(baseUI.getPrimaryScene());
            sensorSimulator.enableColor(true);
            sensorSimulator.enableDepth(true);

            sensorPoseGizmo = new RDXPose3DGizmo();
            sensorPoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);

            pointCloudRenderer.create(WIDTH * HEIGHT);
            pointCloudRenderer.setColoringMethod(ColoringMethod.GRADIENT_SENSOR_X);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
         }

         @Override
         public void render()
         {
            if (throttler.run())
            {
               // "Grab" the image
               sensorSimulator.render(sensorPoseGizmo.getTransformToParent());

               // Get the grabbed images
               RawImage colorImage = sensorSimulator.getColorImage();
               RawImage depthImage = sensorSimulator.getDepthImage();

               // Set point cloud to render
               pointCloudRenderer.updateMesh(depthImage);

               // Set color image to render
               imagePanel.ensureDimensionsMatch(colorImage.getWidth(), colorImage.getHeight());
               colorImage.getCpuImageMat().copyTo(imagePanel.getImage());
               imagePanel.display();

               // Release the grabbed images
               colorImage.release();
               depthImage.release();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            pointCloudRenderer.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSensorSimulatorDemo();
   }
}
