package us.ihmc.gdx;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import us.ihmc.gdx.imgui.GDX3DSituatedImGuiPanel;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "3DSituatedPanelsDemo");
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();
   private GDX3DSituatedImGuiPanel situatedImGuiPanel;

   public GDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            baseUI.getImGuiPanelManager().addPanel("Window 1", this::renderWindow1);

            poseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());

            situatedImGuiPanel = new GDX3DSituatedImGuiPanel("Test Panel", this::renderWindow1);
            situatedImGuiPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5);
            situatedImGuiPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
//            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(situatedImGuiPanel::processImGuiInput);
            baseUI.getPrimaryScene().addRenderableProvider(situatedImGuiPanel::getRenderables);
         }

         @Override
         public void render()
         {
            situatedImGuiPanel.setTransformToReferenceFrame(poseGizmo.getGizmoFrame());
            situatedImGuiPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }

         private void renderWindow1()
         {
            ImGui.text("This is a 3D situated panel.");
            ImGui.button("Button");
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo();
   }
}
