package us.ihmc.gdx.perception;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.tools.thread.Activator;

public class GDXPlanarRegionSLAMDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(),
                                                              "ihmc-open-robotics-software",
                                                              "ihmc-high-level-behaviors/src/test/resources");
   private Activator nativesLoadedActivator;

   public GDXPlanarRegionSLAMDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

            baseUI.create();


         }

         @Override
         public void render()
         {
            super.render();
         }

         @Override
         public void dispose()
         {
            super.dispose();
         }
      });
   }

   public static void main(String[] args)
   {
      new GDXPlanarRegionSLAMDemo();
   }
}
