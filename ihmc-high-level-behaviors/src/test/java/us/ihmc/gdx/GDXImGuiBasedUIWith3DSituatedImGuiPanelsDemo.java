package us.ihmc.gdx;

import com.badlogic.gdx.graphics.g3d.ModelInstance;
import imgui.internal.ImGui;
import us.ihmc.gdx.tools.GDXModelBuilder;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;

public class GDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources",
                                                              "3DSituatedPanelsDemo");
   private final GDXMultiContext3DSituatedImGuiPanelManager situatedImGuiPanelManager = new GDXMultiContext3DSituatedImGuiPanelManager();

   public GDXImGuiBasedUIWith3DSituatedImGuiPanelsDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getPrimaryScene().addModelInstance(new ModelInstance(GDXModelBuilder.createCoordinateFrame(0.3)));

            baseUI.getImGuiPanelManager().addPanel("Window 1", this::renderWindow1);

            situatedImGuiPanelManager.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(),
                                             baseUI.getImGuiWindowAndDockSystem().getImFont());
            GDX3DSituatedImGuiPanel panel = new GDX3DSituatedImGuiPanel("Test Panel", this::renderWindow1);
            situatedImGuiPanelManager.addPanel(panel);
            baseUI.getPrimary3DPanel().addImGui3DViewInputProcessor(situatedImGuiPanelManager::processImGuiInput);

            baseUI.getPrimaryScene().addRenderableProvider(situatedImGuiPanelManager);
         }

         @Override
         public void render()
         {
//            situatedImGuiPanelManager.render();

            baseUI.renderBeforeOnScreenUI();
//            baseUI.renderEnd(situatedImGuiPanelManager::render);
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
