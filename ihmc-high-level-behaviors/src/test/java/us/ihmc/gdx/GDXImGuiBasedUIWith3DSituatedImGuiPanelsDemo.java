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
   private final GDXPose3DGizmo poseGizmo2 = new GDXPose3DGizmo();
   private GDX3DSituatedImGuiPanel situatedImGuiPanel;
   private GDX3DSituatedImGuiPanel situatedImGuiPanel2;

   public GDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getPrimaryScene().addCoordinateFrame(0.3);

            baseUI.getImGuiPanelManager().addPanel("Window 1", this::renderWindow2);

            poseGizmo.createAndSetupDefault(baseUI.getPrimary3DPanel());

            poseGizmo2.createAndSetupDefault(baseUI.getPrimary3DPanel());
            poseGizmo2.getTransformToParent().getTranslation().setZ(0.5);

            situatedImGuiPanel = new GDX3DSituatedImGuiPanel("Window 2", this::renderWindow2);
            situatedImGuiPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
            situatedImGuiPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
//            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(situatedImGuiPanel::processImGuiInput);
            baseUI.getPrimaryScene().addRenderableProvider(situatedImGuiPanel::getRenderables);

            situatedImGuiPanel2 = new GDX3DSituatedImGuiPanel("Window 3", this::renderWindow3);
            situatedImGuiPanel2.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
            situatedImGuiPanel2.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
            baseUI.getPrimaryScene().addRenderableProvider(situatedImGuiPanel2::getRenderables);
         }

         @Override
         public void render()
         {
            situatedImGuiPanel.setTransformToReferenceFrame(poseGizmo.getGizmoFrame());
            situatedImGuiPanel.update();

            situatedImGuiPanel2.setTransformToReferenceFrame(poseGizmo2.getGizmoFrame());
            situatedImGuiPanel2.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }

         private void renderWindow2()
         {
            ImGui.text("This is a 3D situated panel.");
            ImGui.button("Button");
         }

         private void renderWindow3()
         {
            ImGui.text("Another one!");
            ImGui.button("Buttoneee");
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo();
   }
}
