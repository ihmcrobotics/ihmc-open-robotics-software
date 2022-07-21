package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDX3DPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.gdx.visualizers.GDXFrustumVisualizer;
import us.ihmc.perception.BytedecoImage;

import java.nio.ByteBuffer;

public class GDXHighLevelDepthSensorDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private ModelInstance mousePickSphere;
   private int mousePosX;
   private int mousePosY;
   private FramePoint3D pickPoint = new FramePoint3D();
   private GDXFrustumVisualizer frustumVisualizer;
   private GDXCVImagePanel mainViewDepthPanel;

   public GDXHighLevelDepthSensorDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.getPrimaryScene().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.getPrimaryScene().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("DepthSensorZeroTest.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel().getCamera3D());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);
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
            highLevelDepthSensorSimulator = new GDXHighLevelDepthSensorSimulator("Stepping L515",
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
            highLevelDepthSensorSimulator.setPublishDepthImageROS1(false);
            highLevelDepthSensorSimulator.setDebugCoordinateFrame(false);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            highLevelDepthSensorSimulator.setRenderDepthVideoDirectly(true);
            highLevelDepthSensorSimulator.setPublishColorImageROS1(false);
            highLevelDepthSensorSimulator.setPublishColorImageROS2(false);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator, GDXSceneLevel.VIRTUAL);

            mousePickSphere = GDXModelBuilder.createSphere(0.03f, Color.RED);
            baseUI.getPrimaryScene().addRenderableProvider(mousePickSphere, GDXSceneLevel.VIRTUAL);

            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(input ->
            {
               mousePosX = (int) input.getMousePosX();
               mousePosY = (int) input.getMousePosY();
            });

            baseUI.getImGuiPanelManager().addPanel("Mouse Picking", () ->
            {
               ImGui.text("Mouse x: " + mousePosX + " y: " + mousePosY);
               ImGui.text("Pick point: " + pickPoint);
            });

            GDX3DPanel panel3D = new GDX3DPanel("3D View 2", 2, true);
            baseUI.add3DPanel(panel3D);

            frustumVisualizer = new GDXFrustumVisualizer();
            baseUI.getPrimaryScene().addRenderableProvider(frustumVisualizer::getRenderables, GDXSceneLevel.VIRTUAL);
         }

         FramePoint3D cameraPose = new FramePoint3D();
         BytedecoImage image;

         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());

            float viewportWidth = baseUI.getPrimary3DPanel().getCamera3D().viewportWidth;
            float viewportHeight = baseUI.getPrimary3DPanel().getCamera3D().viewportHeight;
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
                  mainViewDepthPanel = new GDXCVImagePanel("Main view depth", (int) baseUI.getPrimary3DPanel().getRenderSizeX(),
                                                                              (int) baseUI.getPrimary3DPanel().getRenderSizeY(),
                                                           true);
                  baseUI.getImGuiPanelManager().addPanel(mainViewDepthPanel.getVideoPanel());

                  baseUI.getPerspectiveManager().reloadPerspective();
               }

               image.resize(aliasedRenderedAreaWidth, aliasedRenderedAreaHeight, null, depthBuffer);
               mainViewDepthPanel.resize(aliasedRenderedAreaWidth, aliasedRenderedAreaHeight, null);
               mainViewDepthPanel.drawFloatImage(image.getBytedecoOpenCVMat());

               int aliasedMouseY = mousePosY * baseUI.getPrimary3DPanel().getAntiAliasing();
               int aliasedMouseX = mousePosX * baseUI.getPrimary3DPanel().getAntiAliasing();
               int aliasedFlippedMouseY = aliasedRenderedAreaHeight - aliasedMouseY;

               if (aliasedMouseX >= 0.0f && aliasedMouseX < viewportWidth && aliasedMouseY >= 0.0f && aliasedMouseY < viewportHeight)
               {
                  float normalizedDeviceCoordinateZ = depthBuffer.getFloat(aliasedFlippedMouseY * (int) baseUI.getPrimary3DPanel().getRenderSizeX() * Float.BYTES + aliasedMouseX * Float.BYTES);

                  if (normalizedDeviceCoordinateZ != 0.0f)
                  {
                     float cameraNear = baseUI.getPrimary3DPanel().getCamera3D().near;
                     float cameraFar = baseUI.getPrimary3DPanel().getCamera3D().far;
                     float twoXCameraFarNear = 2.0f * cameraNear * cameraFar;
                     float farPlusNear = cameraFar + cameraNear;
                     float farMinusNear = cameraFar - cameraNear;
                     float eyeDepth = (twoXCameraFarNear / (farPlusNear - normalizedDeviceCoordinateZ * farMinusNear));

                     float principalOffsetXPixels = viewportWidth / 2.0f;
                     float principalOffsetYPixels = viewportHeight / 2.0f;
                     float fieldOfViewY = baseUI.getPrimary3DPanel().getCamera3D().getVerticalFieldOfView();
                     float focalLengthPixels = (float) ((viewportHeight / 2.0) / Math.tan(Math.toRadians((fieldOfViewY / 2.0))));
                     float zUp3DX = eyeDepth;
                     float zUp3DY = -(aliasedMouseX - principalOffsetXPixels) / focalLengthPixels * eyeDepth;
                     float zUp3DZ = -(aliasedMouseY - principalOffsetYPixels) / focalLengthPixels * eyeDepth;

                     cameraPose.setToZero(baseUI.getPrimary3DPanel().getCamera3D().getCameraFrame());
                     cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

                     pickPoint.setIncludingFrame(baseUI.getPrimary3DPanel().getCamera3D().getCameraFrame(), zUp3DX, zUp3DY, zUp3DZ);
                     pickPoint.changeFrame(ReferenceFrame.getWorldFrame());
                     GDXTools.toGDX(pickPoint, mousePickSphere.transform);
                  }
                  else
                  {
                     // TODO: Snap to XY plane.
                  }
               }
            }

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
            highLevelDepthSensorSimulator.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXHighLevelDepthSensorDemo();
   }
}
