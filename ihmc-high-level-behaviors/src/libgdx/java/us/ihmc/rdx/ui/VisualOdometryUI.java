package us.ihmc.rdx.ui;

import us.ihmc.rdx.Lwjgl3ApplicationAdapter;

import java.net.URISyntaxException;

public class VisualOdometryUI
{

   private final RDXBaseUI baseUI;

   public VisualOdometryUI()
   {
      baseUI = new RDXBaseUI("Visual Odometry UI");

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

   public static void main(String[] args) throws URISyntaxException
   {
      new VisualOdometryUI();
   }
}
