package us.ihmc.rdx.perception;

import imgui.internal.ImGui;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXZEDShapePointCounterDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();

   public RDXZEDShapePointCounterDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
            baseUI.getImGuiPanelManager().addPanel("Shape Point Counter", this::renderImGuiWidgets);
         }

         private void renderImGuiWidgets()
         {
            ImGui.text("RDX app running.");
         }

         @Override
         public void render()
         {
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXZEDShapePointCounterDemo();
   }
}
