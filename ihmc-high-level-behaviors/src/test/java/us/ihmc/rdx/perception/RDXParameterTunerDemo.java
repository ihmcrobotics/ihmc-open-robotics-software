package us.ihmc.rdx.perception;

import us.ihmc.avatar.gpuPlanarRegions.GPUPlanarRegionExtractionParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.RDXBaseUI;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;

public class RDXParameterTunerDemo
{
   private final RDXBaseUI baseUI = new RDXBaseUI();
   private ImGuiStoredPropertySetTuner parametersTuner;

   public RDXParameterTunerDemo()
   {
      baseUI.launchRDXApplication(new Lwjgl3ApplicationAdapter()
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
      new RDXParameterTunerDemo();
   }
}
