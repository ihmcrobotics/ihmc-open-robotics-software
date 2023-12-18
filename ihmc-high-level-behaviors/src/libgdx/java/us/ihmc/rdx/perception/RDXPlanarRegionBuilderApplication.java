package us.ihmc.rdx.perception;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;

public class RDXPlanarRegionBuilderApplication
{
   private final RDXBaseUI baseUI;
   private RDXPlanarRegionBuilder planarRegionBuilder;

   public RDXPlanarRegionBuilderApplication()
   {
      baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources");
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            planarRegionBuilder = new RDXPlanarRegionBuilder(baseUI.getPrimary3DPanel());
            baseUI.getImGuiPanelManager().addPanel("Planar Region Builder", planarRegionBuilder::renderImGuiWidgets);
         }

         @Override
         public void render()
         {
            planarRegionBuilder.update();

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
      new RDXPlanarRegionBuilderApplication();
   }
}
