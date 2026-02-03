package us.ihmc.rdx.simulation;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import us.ihmc.commons.thread.Throttler;
import us.ihmc.perception.RawImage;
import us.ihmc.rdx.AbstractRDXPointCloudRenderer.ColoringMethod;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXMatImagePanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXSensorSimulator;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer;
import us.ihmc.rdx.visualizers.RDXFrustumGraphic;

public class RDXSceneInteractionDemo
{
   private static final int WIDTH = 640;
   private static final int HEIGHT = 480;
   private static final float FOV = 55.0f;
   private static final float MIN_RANGE = 0.105f;
   private static final float MAX_RANGE = 5.0f;
   private static final int NOISE = 20;
   private static final double FPS = 30.0;

   private final RDXBaseUI baseUI = new RDXBaseUI();

   private final Throttler renderThrottler = new Throttler().setFrequency(FPS);
   private final RDXSensorSimulator sensorSimulator;
   private RDXPose3DGizmo sensorPoseGizmo;
   private RDXEnvironmentBuilder environmentBuilder;

   private final RDXRawImagePointCloudRenderer pointCloudRenderer = new RDXRawImagePointCloudRenderer(true);
   private RDXMatImagePanel colorImagePanel;
   private RDXMatImagePanel depthImagePanel;

   private ModelInstance mousePickSphere;
   private int mousePosX;
   private int mousePosY;

   private RDXFrustumGraphic frustumVisualizer;

   public RDXSceneInteractionDemo()
   {
      sensorSimulator = new RDXSensorSimulator(WIDTH, HEIGHT, FOV, MIN_RANGE, MAX_RANGE, NOISE);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.setModelSceneMouseCollisionEnabled(true);

            // Environment setup
            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DepthSensorZeroTest.json");

            // Sensor pose gizmo
            sensorPoseGizmo = new RDXPose3DGizmo();
            sensorPoseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            // Sensor simulator setup
            sensorSimulator.create(baseUI.getPrimaryScene());
            sensorSimulator.enableColor(true);
            sensorSimulator.enableDepth(true);

            // Point cloud renderer
            pointCloudRenderer.create(WIDTH * HEIGHT);
            pointCloudRenderer.setColoringMethod(ColoringMethod.COLOR_IMAGE);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);

            // Image panels
            colorImagePanel = new RDXMatImagePanel("Color Image", WIDTH, HEIGHT, false);
            baseUI.getImGuiPanelManager().addPanel(colorImagePanel.getImagePanel());

            depthImagePanel = new RDXMatImagePanel("Depth Image", WIDTH, HEIGHT, true);
            baseUI.getImGuiPanelManager().addPanel(depthImagePanel.getImagePanel());

            // Mouse picking sphere
            mousePickSphere = RDXModelBuilder.createSphere(0.03f, Color.RED);
            baseUI.getPrimaryScene().addRenderableProvider(mousePickSphere, RDXSceneLevel.VIRTUAL);

            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(input ->
            {
               mousePosX = (int) input.getMousePosX();
               mousePosY = (int) input.getMousePosY();

               LibGDXTools.toLibGDX(input.getPickPointInWorld(), mousePickSphere.transform);
            });

            // Mouse picking panel
            baseUI.getImGuiPanelManager().addPanel("Mouse Picking", () ->
            {
               ImGui.text("Mouse x: " + mousePosX + " y: " + mousePosY);

               if (ImGui.isMouseClicked(ImGuiMouseButton.Right))
               {
                  ImGui.text("Right mouse button clicked at x: " + mousePosX + " y: " + mousePosY);
               }
            });

            // Additional 3D panel
            RDX3DPanel panel3D = new RDX3DPanel("3D View 2", true);
            baseUI.add3DPanel(panel3D);

            // Frustum visualizer
            frustumVisualizer = new RDXFrustumGraphic();
            baseUI.getPrimaryScene().addRenderableProvider(frustumVisualizer, RDXSceneLevel.VIRTUAL);

            // Add a test box
            ModelInstance box = RDXModelBuilder.createBox(0.2f, 0.2f, 0.2f, Color.YELLOW);
            box.transform.translate(0.4f, 0.3f, 0.25f);
            baseUI.getPrimaryScene().addRenderableProvider(box, RDXSceneLevel.MODEL);
            baseUI.getPrimaryScene().addRenderableProvider(box, RDXSceneLevel.GROUND_TRUTH);
         }

         @Override
         public void render()
         {
            if (renderThrottler.run())
            {
               // Render sensor images
               sensorSimulator.render(sensorPoseGizmo.getTransformToParent());

               // Get the rendered images
               RawImage colorImage = sensorSimulator.getColorImage();
               RawImage depthImage = sensorSimulator.getDepthImage();

               // Update point cloud
               pointCloudRenderer.updateMesh(depthImage, colorImage);

               // Display color image
               colorImagePanel.ensureDimensionsMatch(colorImage.getWidth(), colorImage.getHeight());
               colorImage.getCpuImageMat().copyTo(colorImagePanel.getImage());
               colorImagePanel.display();

               // Display depth image
               depthImagePanel.ensureDimensionsMatch(depthImage.getWidth(), depthImage.getHeight());
               depthImage.getCpuImageMat().copyTo(depthImagePanel.getImage());
               depthImagePanel.display();

               // Release images
               colorImage.release();
               depthImage.release();
            }

            // Update frustum visualizer
            frustumVisualizer.generateMeshAsync(baseUI.getPrimary3DPanel().getCamera3D().frustum);
            frustumVisualizer.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            sensorSimulator.destroy();
            pointCloudRenderer.dispose();
            frustumVisualizer.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXSceneInteractionDemo();
   }
}
