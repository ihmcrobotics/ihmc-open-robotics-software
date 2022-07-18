package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Color;
import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.ImGui;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

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

            mousePickSphere = GDXModelBuilder.createSphere(0.1f, Color.RED);
//            baseUI.getPrimaryScene().addRenderableProvider(mousePickSphere, GDXSceneLevel.VIRTUAL);

            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(input ->
            {
               mousePosX = (int) input.getMousePosX() * baseUI.getPrimary3DPanel().getAntiAliasing();
               mousePosY = (int) input.getMousePosY() * baseUI.getPrimary3DPanel().getAntiAliasing();
            });

            baseUI.getImGuiPanelManager().addPanel("Mouse Picking", () ->
            {
               ImGui.text("Mouse x: " + mousePosX + " y: " + mousePosY);
               ImGui.text("Pick point: " + pickPoint);
            });
         }

         @Override
         public void render()
         {
            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());

            float viewportWidth = baseUI.getPrimary3DPanel().getCamera3D().viewportWidth;
            float viewportHeight = baseUI.getPrimary3DPanel().getCamera3D().viewportHeight;

            if (mousePosX >= 0.0f && mousePosX < viewportWidth && mousePosY >= 0.0f && mousePosY < viewportHeight)
            {
               ByteBuffer depthBuffer = baseUI.getPrimary3DPanel().getNormalizedDeviceCoordinateDepthDirectByteBuffer();
               if (depthBuffer != null)
               {
                  float normalizedDeviceCoordinateZ = depthBuffer.getFloat(mousePosY * (int) baseUI.getPrimary3DPanel().getRenderSizeX() * Float.BYTES + mousePosX * Float.BYTES);

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
                     float zUp3DY = -(mousePosX - principalOffsetXPixels) / focalLengthPixels * eyeDepth;
                     float zUp3DZ = -(mousePosY - principalOffsetYPixels) / focalLengthPixels * eyeDepth;
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
