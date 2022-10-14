package us.ihmc.gdx.ui;

import us.ihmc.gdx.Lwjgl3ApplicationAdapter;

import java.net.URISyntaxException;

public class VisualOdometryUI
{

   private final GDXImGuiBasedUI baseUI;

   public VisualOdometryUI()
   {
      baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/libgdx/resources", "Visual Odometry UI");

      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
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

   public static void main(String[] args) throws URISyntaxException
   {
      new VisualOdometryUI();
   }
}
