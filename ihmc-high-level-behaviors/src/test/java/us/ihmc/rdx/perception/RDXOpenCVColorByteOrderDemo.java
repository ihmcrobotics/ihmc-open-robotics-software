package us.ihmc.rdx.perception;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.simulation.environment.RDXEnvironmentBuilder;
import us.ihmc.rdx.simulation.environment.object.RDXEnvironmentObject;
import us.ihmc.rdx.simulation.sensors.RDXHighLevelDepthSensorSimulator;
import us.ihmc.rdx.simulation.sensors.RDXSimulatedSensorFactory;
import us.ihmc.rdx.tools.LibGDXTools;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;
import us.ihmc.perception.BytedecoImage;

import java.nio.ByteOrder;

public class RDXOpenCVColorByteOrderDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   private RDXHighLevelDepthSensorSimulator highLevelDepthSensorSimulator;
   private final RDXPose3DGizmo sensorPoseGizmo = new RDXPose3DGizmo();
   private RDXEnvironmentBuilder environmentBuilder;
   private RDXBytedecoImagePanel openCVImagePanel;
   private final float[] imColor = new float[] {0.8f, 0.0f, 0.0f, 1.0f};
   private final Color gdxColor = new Color();

   public RDXOpenCVColorByteOrderDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            environmentBuilder = new RDXEnvironmentBuilder(baseUI.getPrimary3DPanel());
            environmentBuilder.create();
            baseUI.getImGuiPanelManager().addPanel(environmentBuilder.getPanelName(), environmentBuilder::renderImGuiWidgets);
            environmentBuilder.loadEnvironment("FlatGround.json");

            sensorPoseGizmo.create(baseUI.getPrimary3DPanel());
            sensorPoseGizmo.setResizeAutomatically(true);
            baseUI.getPrimary3DPanel().addImGui3DViewPickCalculator(sensorPoseGizmo::calculate3DViewPick);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(sensorPoseGizmo::process3DViewInput);
            baseUI.getPrimaryScene().addRenderableProvider(sensorPoseGizmo, RDXSceneLevel.VIRTUAL);

            highLevelDepthSensorSimulator = RDXSimulatedSensorFactory.createRealsenseL515(sensorPoseGizmo.getGizmoFrame(), null);
            baseUI.getImGuiPanelManager().addPanel(highLevelDepthSensorSimulator);
            highLevelDepthSensorSimulator.setSensorEnabled(true);
            highLevelDepthSensorSimulator.getLowLevelSimulator().setDepthEnabled(false);
            highLevelDepthSensorSimulator.setRenderColorVideoDirectly(true);
            baseUI.getPrimaryScene().addRenderableProvider(highLevelDepthSensorSimulator::getRenderables);

            RDXPanel panel = new RDXPanel("Color Byte Order Debugging", this::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);

            openCVImagePanel = new RDXBytedecoImagePanel("OpenCV Image Panel", highLevelDepthSensorSimulator.getLowLevelSimulator().getRGBA8888ColorImage());
            baseUI.getImGuiPanelManager().addPanel(openCVImagePanel.getImagePanel());
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
            LibGDXTools.toLibGDX(imColor, gdxColor);
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
            for (RDXEnvironmentObject allObject : environmentBuilder.getAllObjects())
            {
               allObject.getRealisticModelInstance().setDiffuseColor(gdxColor);
            }

            highLevelDepthSensorSimulator.render(baseUI.getPrimaryScene());

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
      new RDXOpenCVColorByteOrderDemo();
   }
}
