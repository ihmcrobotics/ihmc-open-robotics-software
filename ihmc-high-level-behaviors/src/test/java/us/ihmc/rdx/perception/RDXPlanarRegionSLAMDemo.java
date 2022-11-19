package us.ihmc.rdx.perception;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.perception.PlanarRegionRegistrationPanel;
import us.ihmc.rdx.visualizers.RDXPlanarRegionsGraphic;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.PlanarRegionRegistration;
import us.ihmc.tools.thread.Activator;

public class RDXPlanarRegionSLAMDemo
{
   private final RDXPlanarRegionsGraphic graphic = new RDXPlanarRegionsGraphic();

   private Activator nativesLoadedActivator;
   private PlanarRegionRegistration planeToPlaneICP;
   private PlanarRegionRegistrationPanel slamPanel;

   private final RDXBaseUI baseUI = new RDXBaseUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");

   public RDXPlanarRegionSLAMDemo()
   {

      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadGTSAMNativesOnAThread();

            baseUI.create();

            planeToPlaneICP = new PlanarRegionRegistration();
            slamPanel = new PlanarRegionRegistrationPanel("SLAM Module", planeToPlaneICP);

            graphic.generateMeshes(planeToPlaneICP.getCurrentRegions());
            graphic.update();

            baseUI.getPrimaryScene().addRenderableProvider(graphic);
            baseUI.getImGuiPanelManager().addPanel(slamPanel);
         }

         @Override
         public void render()
         {
            if (planeToPlaneICP.modified())
            {
               graphic.clear();
               graphic.generateMeshes(planeToPlaneICP.getCurrentRegions());
               graphic.update();
               planeToPlaneICP.setModified(false);
            }

            baseUI.renderBeforeOnScreenUI();
            baseUI.renderEnd();
         }

         @Override
         public void dispose()
         {
            baseUI.dispose();
            graphic.destroy();
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new RDXPlanarRegionSLAMDemo();
   }
}
