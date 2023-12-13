package us.ihmc.rdx.perception;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.imgui.RDXPanel;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXNettyOusterDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private final RDXNettyOusterUI nettyOusterUI = new RDXNettyOusterUI();

   public RDXNettyOusterDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            nettyOusterUI.create(baseUI);

            RDXPanel panel = new RDXPanel("Ouster", nettyOusterUI::renderImGuiWidgets);
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