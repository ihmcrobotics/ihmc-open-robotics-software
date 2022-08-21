package us.ihmc.gdx;

import imgui.ImGui;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.gizmo.GDXPose3DGizmo;

public class GDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "3DSituatedPanelsDemo");
   private final GDXMultiContext3DSituatedImGuiPanelManager situatedImGuiPanelManager = new GDXMultiContext3DSituatedImGuiPanelManager();
   private final GDXPose3DGizmo poseGizmo = new GDXPose3DGizmo();

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

            situatedImGuiPanelManager.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3());
            GDX3DSituatedImGuiPanel panel = new GDX3DSituatedImGuiPanel("Test Panel", this::renderWindow1);
            situatedImGuiPanelManager.addPanel(panel);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(situatedImGuiPanelManager::processImGuiInput);
            baseUI.getPrimaryScene().addRenderableProvider(situatedImGuiPanelManager);
         }

         @Override
         public void render()
         {
            situatedImGuiPanelManager.setTransformToReferenceFrame(poseGizmo.getGizmoFrame());

            situatedImGuiPanelManager.render();

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
