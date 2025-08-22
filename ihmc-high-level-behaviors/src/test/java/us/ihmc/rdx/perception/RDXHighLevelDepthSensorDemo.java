package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import imgui.flag.ImGuiMouseButton;
import org.bytedeco.opencv.global.opencv_core;
import perception_msgs.msg.dds.HeightMapMessage;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.tools.RDXModelBuilder;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDX3DPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXGridMapGraphic;
import us.ihmc.rdx.visualizers.RDXFrustumGraphic;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.heightMap.HeightMapMessageTools;

import java.nio.ByteBuffer;

public class RDXHighLevelDepthSensorDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   private RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private ModelInstance mousePickSphere;
   private int mousePosX;
   private int mousePosY;
   private RDXFrustumGraphic frustumVisualizer;
   private RDXBytedecoImagePanel mainViewDepthPanel;
   private BytedecoImage image;
   private RDXGridMapGraphic heightMap;

   public RDXHighLevelDepthSensorDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.setModelSceneMouseCollisionEnabled(true);

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("DepthSensorZeroTest.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);
//            sensorPoseGizmo.getTransformToParent().appendTranslation(2.2, 0.0, 1.0);
//            sensorPoseGizmo.getTransformToParent().appendPitchRotation(Math.PI / 4.0);

            // https://www.scratchapixel.com/lessons/3d-basic-rendering/perspective-and-orthographic-projection-matrix/opengl-perspective-projection-matrix
            double publishRateHz = 5.0;
            double verticalFOV = 55.0;
            int imageWidth = 640;
            int imageHeight = 480;
            // range should be as small as possible because depth precision is nonlinear
            // it gets drastically less precise father away
            double minRange = 0.105;
            double maxRange = 5.0;
            highLevelDepthSensorSimulator = new RDXHighLevelDepthSensorSimulator("Stepping L515",
                                                                                 sensorPoseGizmo.getGizmoFrame(),
                                                                                 () -> 0L,
                                                                                 verticalFOV,
                                                                                 imageWidth,
                                                                                 imageHeight,
                                                                                 minRange,
                                                                                 maxRange,
                                                                                 0.03,
                                                                                 0.05,
                                                                                 true,
                                                                                 publishRateHz);
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.setPublishPointCloudROS2(false);
            highLevelDepthSensorSimulator.setRenderPointCloudDirectly(true);
            highLevelDepthSensorSimulator.setDebugCoordinateFrame(false);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            highLevelDepthSensorSimulator.setRenderDepthVideoDirectly(true);
            highLevelDepthSensorSimulator.setPublishColorImageROS2(false);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator::getRenderables);

            mousePickSphere = RDXModelBuilder.createSphere(0.03f, Color.RED);
            baseUI.getPrimaryScene().addRenderableProvider(mousePickSphere, RDXSceneLevel.VIRTUAL);

            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(input ->
            {
               mousePosX = (int) input.getMousePosX();
               mousePosY = (int) input.getMousePosY();

               LibGDXTools.toLibGDX(input.getPickPointInWorld(), mousePickSphere.transform);
            });

            baseUI.getImGuiPanelManager().addPanel("Mouse Picking", () ->
            {
               ImGui.text("Mouse x: " + mousePosX + " y: " + mousePosY);

               if (ImGui.isMouseClicked(ImGuiMouseButton.Right))
               {  // Place breakpoint here for debugging
                  ImGui.text("Right mouse button clicked at x: " + mousePosX + " y: " + mousePosY);
               }
            });

            RDX3DPanel panel3D = new RDX3DPanel("3D View 2", true);
            baseUI.add3DPanel(panel3D);

            frustumVisualizer = new RDXFrustumGraphic();
            baseUI.getPrimaryScene().addRenderableProvider(frustumVisualizer, RDXSceneLevel.VIRTUAL);

            ModelInstance box = RDXModelBuilder.createBox(0.2f, 0.2f, 0.2f, Color.YELLOW);
            box.transform.translate(0.4f, 0.3f, 0.25f);
            baseUI.getPrimaryScene().addRenderableProvider(box, RDXSceneLevel.MODEL);
            baseUI.getPrimaryScene().addRenderableProvider(box, RDXSceneLevel.GROUND_TRUTH);

            heightMap = new RDXGridMapGraphic();
            HeightMapMessage heightMapMessage = new HeightMapMessage();
            heightMapMessage.setWidthInMeters(2.0);
            heightMapMessage.setGridCenterX(1.0);
            heightMapMessage.setGridCenterY(1.0);
            HeightMapMessageTools.setToFlatGround(heightMapMessage);
            heightMap.generateMeshesAsync(heightMapMessage);
            baseUI.getPrimaryScene().addRenderableProvider(heightMap, RDXSceneLevel.MODEL);
         }

         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());

            int aliasedRenderedAreaWidth = (int) baseUI.getPrimary3DPanel().getRenderSizeX();
            int aliasedRenderedAreaHeight = (int) baseUI.getPrimary3DPanel().getRenderSizeY();

            ByteBuffer depthBuffer = baseUI.getPrimary3DPanel().getNormalizedDeviceCoordinateDepthDirectByteBuffer();
            if (depthBuffer != null)
            {
               if (image == null)
               {
                  image = new BytedecoImage((int) baseUI.getPrimary3DPanel().getRenderSizeX(),
                                            (int) baseUI.getPrimary3DPanel().getRenderSizeY(),
                                            opencv_core.CV_32FC1,
                                            depthBuffer);
                  mainViewDepthPanel = new RDXBytedecoImagePanel("Main view depth", (int) baseUI.getPrimary3DPanel().getRenderSizeX(),
                                                                 (int) baseUI.getPrimary3DPanel().getRenderSizeY(),
                                                                 true);
                  baseUI.getImGuiPanelManager().addPanel(mainViewDepthPanel.getImagePanel());

                  baseUI.getLayoutManager().reloadLayout();
               }

               image.resize(aliasedRenderedAreaWidth, aliasedRenderedAreaHeight, null, depthBuffer);
               mainViewDepthPanel.resize(aliasedRenderedAreaWidth, aliasedRenderedAreaHeight, null);
               mainViewDepthPanel.drawDepthImage(image.getBytedecoOpenCVMat());
            }

            frustumVisualizer.generateMeshAsync(baseUI.getPrimary3DPanel().getCamera3D().frustum);
            frustumVisualizer.update();

            heightMap.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            environmentBuilder.destroy();
            highLevelDepthSensorSimulator.dispose();
            heightMap.destroy();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXHighLevelDepthSensorDemo();
   }
}
