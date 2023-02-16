package us.ihmc.rdx.perception;

import us.ihmc.perception.BytedecoTools;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.ImGuiPanel;
import us.ihmc.rdx.sceneManager.RDXSceneLevel;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.tools.thread.Activator;

public class RDXNettyOusterDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI("ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources");
   private final Activator nativesLoadedActivator;
   private final RDXNettyOusterUI nettyOusterUI = new RDXNettyOusterUI();

   public RDXNettyOusterDemo()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            nettyOusterUI.create(baseUI);

            ImGuiPanel panel = new ImGuiPanel("Ouster", nettyOusterUI::renderImGuiWidgets);
            baseUI.getImGuiPanelManager().addPanel(panel);
         }

         /**
          * The threading here isn't really ideal. The rendered image and point cloud
          * only need to be rendered after onFrameReceived in a new thread. But I didn't
          * find it necessary to spend the time on the thread barriers for those yet,
          * so we just run the kernels everytime and sync over the copy.
          */
         @Override
         public void render()
         {
            if (nativesLoadedActivator.poll())
            {
               if (nativesLoadedActivator.isNewlyActivated())
               {
                  nettyOusterUI.createAfterNativesLoaded();
               }

               if (nettyOusterUI.isOusterInitialized())
               {
                  if (nettyOusterUI.getImagePanel() == null)
                  {
                     nettyOusterUI.createAfterOusterInitialized();

                     baseUI.getPrimaryScene().addRenderableProvider(nettyOusterUI::getRenderables);
                     baseUI.getImGuiPanelManager().addPanel(nettyOusterUI.getImagePanel().getImagePanel());
                     baseUI.getLayoutManager().reloadLayout();
                  }

                  nettyOusterUI.update();
               }
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            nettyOusterUI.destroy();
            baseUI.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXNettyOusterDemo();
   }
}