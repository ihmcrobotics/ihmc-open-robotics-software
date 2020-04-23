package us.ihmc.parameterTuner.remote;

import javafx.stage.Stage;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;

public class ParameterTuner extends ParameterTuningApplication
{
   private ParameterGuiInterface inputManager;

   public ParameterTuner()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      inputManager = new RemoteInputManager(getAutoDiscoveryProperty());

      super.start(primaryStage);
   }

   public boolean getAutoDiscoveryProperty()
   {
      String enableAutoDiscoveryParameter = getParameters().getNamed().getOrDefault("enableAutoDiscovery", "true");
      return Boolean.parseBoolean(enableAutoDiscoveryParameter);
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
