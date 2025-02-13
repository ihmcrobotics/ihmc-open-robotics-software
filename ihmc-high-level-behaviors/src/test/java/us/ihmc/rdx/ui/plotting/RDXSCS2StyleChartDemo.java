package us.ihmc.rdx.ui.plotting;

import us.ihmc.rdx.gdx.RDXLwjgl3ApplicationAdapter;
import us.ihmc.rdx.RDXBaseUI;

public class RDXSCS2StyleChartDemo
{
   private final RDXBaseUI baseUI;

   public RDXSCS2StyleChartDemo()
   {
      baseUI = new RDXBaseUI();

      baseUI.launchRDXApplication(new RDXLwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            baseUI.getImGuiPanelManager().addPanel("Plot", this::renderImGuiWidgets);
         }

         private void renderImGuiWidgets()
         {

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
      new RDXSCS2StyleChartDemo();
   }
}
