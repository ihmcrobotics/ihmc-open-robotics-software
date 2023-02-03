package us.ihmc.rdx.logging;

import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.Activator;

public class RDXHDF5ImageBrowserDemo
{
   private final Activator nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();
   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(),
                                                  "ihmc-open-robotics-software",
                                                  "ihmc-high-level-behaviors/src/libgdx/resources",
                                                  "HDF5 Image Browser Demo");
   private RDXHDF5ImageBrowser hdf5ImageBrowser;

   public RDXHDF5ImageBrowserDemo()
   {
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
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  hdf5ImageBrowser = new RDXHDF5ImageBrowser();
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getControlPanel());
                  baseUI.getImGuiPanelManager().addPanel(hdf5ImageBrowser.getImagePanel().getVideoPanel());
                  baseUI.getLayoutManager().reloadLayout();
               }

               hdf5ImageBrowser.update();
            }

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
