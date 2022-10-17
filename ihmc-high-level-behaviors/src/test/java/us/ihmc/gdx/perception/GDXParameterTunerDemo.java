package us.ihmc.gdx.perception;

import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.gdx.Lwjgl3ApplicationAdapter;
import us.ihmc.gdx.ui.GDXImGuiBasedUI;
import us.ihmc.gdx.ui.ImGuiStoredPropertySetTuner;

public class GDXParameterTunerDemo
{
   private final GDXImGuiBasedUI baseUI = new GDXImGuiBasedUI(getClass(), "ihmc-open-robotics-software", "ihmc-high-level-behaviors/src/test/resources");
   private ImGuiStoredPropertySetTuner parametersTuner;

   public GDXParameterTunerDemo()
   {
      baseUI.launchGDXApplication(new Lwjgl3ApplicationAdapter()
      {
         @Override
         public void create()
         {
            baseUI.create();

            GPUPlanarRegionExtractionParameters parameters = new GPUPlanarRegionExtractionParameters();
            parametersTuner = new ImGuiStoredPropertySetTuner(parameters.getTitle());
            parametersTuner.create(parameters);
            baseUI.getImGuiPanelManager().addPanel(parametersTuner);
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

   public static void main(String[] args)
   {
      new GDXParameterTunerDemo();
   }
}
