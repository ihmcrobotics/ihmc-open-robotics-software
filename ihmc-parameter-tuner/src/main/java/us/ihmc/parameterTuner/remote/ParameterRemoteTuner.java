package us.ihmc.parameterTuner.remote;

import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;

public class ParameterRemoteTuner extends ParameterTuningApplication
{
   @Override
   protected ParameterGuiInterface createInputManager()
   {
      return new RemoteInputManager();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
