package us.ihmc.parameterTuner.remote;

import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;

public class ParameterTuner extends ParameterTuningApplication
{
   private final ParameterGuiInterface inputManager;

   public ParameterTuner()
   {
      inputManager = new RemoteInputManager();
   }

   public ParameterTuner(String serverAddress)
   {
      inputManager = new RemoteInputManager(serverAddress);
   }

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
