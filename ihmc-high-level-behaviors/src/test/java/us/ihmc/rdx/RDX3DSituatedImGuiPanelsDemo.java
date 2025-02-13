package us.ihmc.rdx;

import imgui.internal.ImGui;
import us.ihmc.rdx.gdx.RDXLwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDX3DSituatedImGuiPanel;
import us.ihmc.rdx.demo.BoxesDemoModel;

public class RDX3DSituatedImGuiPanelsDemo
{
   private RDX3DSituatedImGuiPanel situatedImGuiPanel;

   public RDX3DSituatedImGuiPanelsDemo()
   {
      RDXBaseUI baseUI = new RDXBaseUI();
      baseUI.launchRDXApplication(new RDXLwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            situatedImGuiPanel = new RDX3DSituatedImGuiPanel("Test Panel", () ->
            {
               ImGui.text("This is a 3D situated panel.");
               ImGui.button("Button");
            });
            situatedImGuiPanel.create(baseUI.getImGuiWindowAndDockSystem().getImGuiGl3(), 0.3, 0.5, 10);
            baseUI.getPrimaryScene().addRenderableProvider(situatedImGuiPanel::getRenderables);

            baseUI.getPrimaryScene().addCoordinateFrame(0.3);
            baseUI.getPrimaryScene().addModelInstance(new BoxesDemoModel().newInstance());
         }

         @Override
         public void render()
         {
            situatedImGuiPanel.update();

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            situatedImGuiPanel.dispose();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDX3DSituatedImGuiPanelsDemo();
   }
}
