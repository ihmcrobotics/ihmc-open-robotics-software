package us.ihmc.gdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.imgui.ImGuiPanel;
import us.ihmc.gdx.sceneManager.GDXSceneLevel;
import us.ihmc.gdx.simulation.environment.GDXEnvironmentBuilder;
import us.ihmc.gdx.simulation.environment.object.GDXEnvironmentObject;
import us.ihmc.gdx.simulation.sensors.GDXHighLevelDepthSensorSimulator;
import us.ihmc.gdx.simulation.sensors.GDXSimulatedSensorFactory;
import us.ihmc.gdx.tools.GDXTools;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;
import us.ihmc.perception.BytedecoImage;

import java.nio.ByteOrder;

public class GDXOpenCVColorByteOrderDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");

   private GDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final GDXPose3DGizmo sensorPoseGizmo = new GDXPose3DGizmo();
   private GDXEnvironmentBuilder environmentBuilder;
   private GDXCVImagePanel openCVImagePanel;
   private final float[] imColor = new float[] {0.8f, 0.0f, 0.0f, 1.0f};
   private final Color gdxColor = new Color();

   public GDXOpenCVColorByteOrderDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new GDXEnvironmentBuilder(baseUI.get3DSceneManager());
            environmentBuilder.create(baseUI);
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getRealRenderables, GDXSceneLevel.REAL_ENVIRONMENT);
            baseUI.get3DSceneManager().addRenderableProvider(environmentBuilder::getVirtualRenderables, GDXSceneLevel.VIRTUAL);
            environmentBuilder.loadEnvironment("DepthSensorZeroTest.json");

            sensorPoseGizmo.create(baseUI.get3DSceneManager().getCamera3D());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.get3DSceneManager().addRenderableProvider(sensorPoseGizmo, GDXSceneLevel.VIRTUAL);

            highLevelDepthSensorSimulator = GDXSimulatedSensorFactory.createL515ImageOnlyNoComms(sensorPoseGizmo.getGizmoFrame());
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.create();
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.getLowLevelSimulator().setDepthEnabled(false);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            baseUI.get3DSceneManager().addRenderableProvider(highLevelDepthSensorSimulator, GDXSceneLevel.VIRTUAL);

            ImGuiPanel panel = new ImGuiPanel("Color Byte Order Debugging", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            openCVImagePanel = new GDXCVImagePanel("OpenCV Image Panel", highLevelDepthSensorSimulator.getLowLevelSimulator().getRGBA8888ColorImage());
            baseUI.getImGuiPanelManager().addPanel(openCVImagePanel.getVideoPanel());
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("System native byte order: " + ByteOrder.nativeOrder().toString());

            ImGui.text("Simulated sensor buffer bytes:");
            ImGui.text("Should read R G B A");

            BytedecoImage sensorImage = highLevelDepthSensorSimulator.getLowLevelSimulator().getRGBA8888ColorImage();
            sensorImage.getBackingDirectByteBuffer().rewind();
            for (int i = 0; i < 4; i++)
            {
               printBytes(sensorImage.getBackingDirectByteBuffer().get(),
                          sensorImage.getBackingDirectByteBuffer().get(),
                          sensorImage.getBackingDirectByteBuffer().get(),
                          sensorImage.getBackingDirectByteBuffer().get());
            }

            ImGui.text("Simulated sensor Mat bytes:");
            ImGui.text("Should read R G B A");
            for (int i = 0; i < 4; i++)
            {
               printBytes(sensorImage.getBytedecoOpenCVMat().ptr(0, i).get(0),
                          sensorImage.getBytedecoOpenCVMat().ptr(0, i).get(1),
                          sensorImage.getBytedecoOpenCVMat().ptr(0, i).get(2),
                          sensorImage.getBytedecoOpenCVMat().ptr(0, i).get(3));
            }

            ImGui.spacing();
            ImGui.separator();

            ImGui.colorPicker4("Environment Color", imColor);
            GDXTools.toGDX(imColor, gdxColor);
         }

         private void printBytes(byte byte0, byte byte1, byte byte2, byte byte3)
         {
            printInts(Byte.toUnsignedInt(byte0),
                      Byte.toUnsignedInt(byte1),
                      Byte.toUnsignedInt(byte2),
                      Byte.toUnsignedInt(byte3));
         }

         private void printInts(int int0, int int1, int int2, int int3)
         {
            ImGui.text(int0 + " " + int1 + " " + int2 + " " + int3);
         }

         @Override
         public void render()
         {
            for (GDXEnvironmentObject allObject : environmentBuilder.getAllObjects())
            {
               allObject.getRealisticModelInstance().setDiffuseColor(gdxColor);
            }

            highLevelDepthSensorSimulator.render(baseUI.get3DSceneManager());

            openCVImagePanel.draw();

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
      new GDXOpenCVColorByteOrderDemo();
   }
}
