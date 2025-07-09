package us.ihmc.rdx.ui;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.perception.RDXHeightMapPanel;

public class HeightMapDebuggingUI
{
   RDXBaseUI baseUI;
   RDXHeightMapPanel heightMapPanel;

   public HeightMapDebuggingUI()
   {
      baseUI = new RDXBaseUI("HeightMapDebuggingUI");
      heightMapPanel = new RDXHeightMapPanel();
      baseUI.getImGuiPanelManager().addPanel(heightMapPanel);

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();
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
      new HeightMapDebuggingUI();
   }
}
