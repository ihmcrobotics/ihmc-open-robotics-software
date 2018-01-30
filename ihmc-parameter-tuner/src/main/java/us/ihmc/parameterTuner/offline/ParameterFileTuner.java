package us.ihmc.parameterTuner.offline;

import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;

public class ParameterFileTuner extends ParameterTuningApplication
{
   @Override
   protected ParameterGuiInterface createInputManager()
   {
      return new FileInputManager();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
