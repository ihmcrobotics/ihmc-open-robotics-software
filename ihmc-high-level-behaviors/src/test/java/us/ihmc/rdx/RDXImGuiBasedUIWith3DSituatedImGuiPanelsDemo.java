package us.ihmc.rdx;

import com.badlogic.gdx.graphics.Color;
import imgui.ImGui;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.gizmo.RDXPose3DGizmo;

public class RDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("3DSituatedPanelsDemo");
   private final RDXPose3DGizmo poseGizmo = new RDXPose3DGizmo();
   private final RDXPose3DGizmo poseGizmo2 = new RDXPose3DGizmo();
   private RDX3DSituatedImGuiPanel situatedImGuiPanel;
   private RDX3DSituatedImGuiPanel situatedImGuiPanel2;

   public RDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
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

            situatedImGuiPanel = new RDX3DSituatedImGuiPanel("Window 2", this::renderWindow2);
            situatedImGuiPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
            situatedImGuiPanel.setBackgroundTransparency(new Color(0.3f, 0.3f, 0.3f, 0.75f));
//            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(situatedImGuiPanel::processImGuiInput);
            baseUI.getPrimaryScene().addRenderableProvider(situatedImGuiPanel::getRenderables);

            situatedImGuiPanel2 = new RDX3DSituatedImGuiPanel("Window 3", this::renderWindow3);
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
      new RDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo();
   }
}
