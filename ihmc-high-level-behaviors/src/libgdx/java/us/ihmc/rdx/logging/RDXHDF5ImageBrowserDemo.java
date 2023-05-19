package us.ihmc.rdx.logging;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXHDF5ImageBrowserDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("HDF5 Image Browser Demo");
   private RDXHDF5ImageBrowser hdf5ImageBrowser;

   public RDXHDF5ImageBrowserDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            hdf5ImageBrowser = new RDXHDF5ImageBrowser();
            baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getControlPanel());
            baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getImagePanel().getImagePanel());
         }

         @Override
         public void render()
         {
            hdf5ImageBrowser.update();
            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            hdf5ImageBrowser.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXHDF5ImageBrowserDemo();
   }
}
