package us.ihmc.parameterTuner.remote;

import java.io.IOException;

import javafx.stage.Stage;
import us.ihmc.javafx.JavaFXMissingTools;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.parameterTuner.guiElements.main.ParameterGuiInterface;
import us.ihmc.parameterTuner.guiElements.main.ParameterTuningApplication;
import us.ihmc.robotDataLogger.gui.DataServerSelectorGUI;
import us.ihmc.robotDataLogger.websocket.client.discovery.HTTPDataServerConnection;

public class ParameterTuner extends ParameterTuningApplication
{
   private final ParameterGuiInterface inputManager;

   public ParameterTuner(boolean enableAutoDiscovery) throws IOException
   {
      inputManager = new RemoteInputManager(enableAutoDiscovery);
   }

   public ParameterTuner(String serverAddress) throws IOException
   {
      inputManager = new RemoteInputManager(serverAddress);
   }

   public ParameterTuner(HTTPDataServerConnection connection) throws IOException
   {
      inputManager = new RemoteInputManager(connection);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {

      super.start(primaryStage);
   }

   @Override
   protected ParameterGuiInterface createInputManager()
   {
      return inputManager;
   }

   public static void main(String[] args)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();
      String enableAutoDiscoveryParameter = System.getProperty("enableAutoDiscovery", "true");
      DataServerSelectorGUI selector = new DataServerSelectorGUI(Boolean.parseBoolean(enableAutoDiscoveryParameter));

      HTTPDataServerConnection connection = selector.select();

      if (connection == null)
      {
         LogTools.warn("No host selected. Shutting down.");
         System.exit(0);
      }
      else
      {
         try
         {
            JavaFXMissingTools.runApplication(new ParameterTuner(connection));
         }
         catch (IOException e)
         {
            throw new RuntimeException(e);
         }
      }
   }
}
