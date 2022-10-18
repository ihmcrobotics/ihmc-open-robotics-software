package us.ihmc.gdx.perception;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.perception.PlanarRegionRegistrationPanel;
import us.ihmc.gdx.visualizers.GDXPlanarRegionsGraphic;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.PlanarRegionRegistration;
import us.ihmc.tools.thread.Activator;

public class GDXPlanarRegionSLAMDemo
{
   private final GDXPlanarRegionsGraphic graphic = new GDXPlanarRegionsGraphic();

   private Activator nativesLoadedActivator;
   private PlanarRegionRegistration planeToPlaneICP;
   private PlanarRegionRegistrationPanel slamPanel;

   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");

   public GDXPlanarRegionSLAMDemo()
   {



      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();
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
            if(planeToPlaneICP.modified())
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
      new GDXPlanarRegionSLAMDemo();
   }
}
