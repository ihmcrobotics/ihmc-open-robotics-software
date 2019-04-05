package us.ihmc.parameterTuner.remote;

import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;

public class ParameterTuner extends ParameterTuningApplication
{
   private final ParameterGuiInterface inputManager = new RemoteInputManager();

   @Override
   protected ParameterGuiInterface createInputManager()
   {
      return inputManager;
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
