package us.ihmc.rdx.ui;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import imgui.type.ImFloat;
import imgui.type.ImInt;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.opencl.OpenCLPointCloudExtractor;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer.ColoringMethod;
import us.ihmc.rdx.ui.graphics.RDXRawImagePointCloudRenderer.InputMethod;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensors.ZEDColorDepthImageRetriever;

import java.util.Arrays;
import java.util.List;

public class RDXPointCloudRendererDemo
{
   private final String[] inputMethods = Arrays.stream(InputMethod.values()).map(Enum::name).toArray(String[]::new);
   private final ImInt currentInputMethod = new ImInt(InputMethod.DEPTH_IMAGE.ordinal());

   private final String[] coloringMethods = Arrays.stream(ColoringMethod.values()).map(Enum::name).toArray(String[]::new);
   private final ImInt currentColoringMethod = new ImInt(ColoringMethod.DEFAULT.ordinal());

   private final ImFloat pointScale = new ImFloat(0.01f);
   private final float[] defaultColor = new float[] {1.0f, 1.0f, 1.0f, 1.0f};

   private RDXRawImagePointCloudRenderer pointCloudRenderer;
   private final OpenCLPointCloudExtractor pointCloudExtractor = new OpenCLPointCloudExtractor();

   private long lastGrabSequenceNumber = -1L;
   private ZEDColorDepthImageRetriever zed;
   private RDXPose3DGizmo sensorPoseGizmo;

   private RDXPointCloudRendererDemo()
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

            pointCloudRenderer = new RDXRawImagePointCloudRenderer(InputMethod.values()[currentInputMethod.get()]);
            pointCloudRenderer.create(1280 * 720);
            pointCloudRenderer.setColoringMethod(ColoringMethod.values()[currentColoringMethod.get()]);
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);

            baseUI.getImGuiPanelManager().addPanel("Options", this::renderOptions);
         }

         @Override
         public void render()
         {
            if (lastGrabSequenceNumber < zed.getGrabSequenceNumber())
            {
               lastGrabSequenceNumber = zed.getGrabSequenceNumber();

               RawImage colorImage = zed.getLatestRawColorImage(RobotSide.LEFT);
               RawImage depthImage = zed.getLatestRawDepthImage();

               InputMethod inputMethod = InputMethod.values()[currentInputMethod.get()];
               if (inputMethod == InputMethod.DEPTH_IMAGE)
               {
                  pointCloudRenderer.updateMesh(depthImage);
               }
               else if (inputMethod == InputMethod.DEPTH_AND_COLOR_IMAGE)
               {
                  // TODO:
               }
               else // inputMethod == InputMethod.POINT_CLOUD
               {
                  List<Point3D32> pointCloud = pointCloudExtractor.extractPointCloud(depthImage);
                  pointCloudRenderer.updateMesh(pointCloud);
               }

               colorImage.release();
               depthImage.release();
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         private void renderOptions()
         {
            if (ImGui.combo("Input Method", currentInputMethod, inputMethods))
               reinitializePointCloudRenderer();
            if (ImGui.combo("Coloring Method", currentColoringMethod, coloringMethods))
               reinitializePointCloudRenderer();

            if (ImGui.sliderFloat("Point Scale", pointScale.getData(), 0.0f, 0.1f))
               pointCloudRenderer.setPointScale(pointScale.get());

            if (ImGui.colorEdit4("Default Color", defaultColor))
               pointCloudRenderer.setDefaultPointColor(new Color(defaultColor[0], defaultColor[1], defaultColor[2], defaultColor[3]));
         }

         private void reinitializePointCloudRenderer()
         {
            if (pointCloudRenderer != null)
               pointCloudRenderer.dispose();

            pointCloudRenderer = new RDXRawImagePointCloudRenderer(InputMethod.values()[currentInputMethod.get()]);
            pointCloudRenderer.create(1280 * 720);
            pointCloudRenderer.setColoringMethod(ColoringMethod.values()[currentColoringMethod.get()]);
            pointCloudRenderer.setPointScale(pointScale.get());
            pointCloudRenderer.setDefaultPointColor(new Color(defaultColor[0], defaultColor[1], defaultColor[2], defaultColor[3]));
            baseUI.getPrimaryScene().addRenderableProvider(pointCloudRenderer);
         }

         @Override
         public void dispose()
         {
            zed.destroy();
            pointCloudRenderer.dispose();
            pointCloudExtractor.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPointCloudRendererDemo();
   }
}
