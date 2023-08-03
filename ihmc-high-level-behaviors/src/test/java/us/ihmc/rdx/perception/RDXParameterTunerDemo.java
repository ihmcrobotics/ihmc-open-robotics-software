package us.ihmc.rdx.perception;

import us.ihmc.perception.rapidRegions.RapidRegionsExtractorParameters;
import us.ihmc.rdx.Lwjgl3ApplicationAdapter;
import us.ihmc.rdx.ui.ImGuiStoredPropertySetTuner;
import us.ihmc.rdx.ui.RDXBaseUI;

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

            RapidRegionsExtractorParameters parameters = new RapidRegionsExtractorParameters();
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
