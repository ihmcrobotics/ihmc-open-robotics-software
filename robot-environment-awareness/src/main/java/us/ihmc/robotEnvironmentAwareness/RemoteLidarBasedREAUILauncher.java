package us.ihmc.robotEnvironmentAwareness;

import java.util.Map;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;

public class RemoteLidarBasedREAUILauncher extends Application
{
   public RemoteLidarBasedREAUILauncher()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      Parameters parameters = getParameters();
      Map<String, String> namedParameters = parameters.getNamed();
      String host = namedParameters.getOrDefault("host", "localhost");
      PrintTools.info("Creating REA UI with the module address: " + host);

      if (!parameters.getRaw().isEmpty())
         PrintTools.info("Received the program arguments: " + parameters.getRaw());

      LIDARBasedEnvironmentAwarenessUI remoteUI = LIDARBasedEnvironmentAwarenessUI.creatRemoteUI(primaryStage, host);
      remoteUI.show();
   }

   public static void main(String[] args)
   {
      PrintTools.info("To change the address of the module, enter its IP address as a program argument. For instance: " + "--host=127.0.0.1");
      launch(args);
   }
}
