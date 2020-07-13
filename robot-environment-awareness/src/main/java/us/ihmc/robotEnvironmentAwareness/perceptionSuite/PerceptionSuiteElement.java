package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.application.Platform;
import javafx.stage.Stage;

public class PerceptionSuiteElement<M extends PerceptionModule, U extends PerceptionUI>
{
   private final Stage stage;
   private final M perceptionModule;
   private final U uiModule;

   public PerceptionSuiteElement(ModuleProvider<M> moduleProvider, UIProvider<U> uiProvider) throws Exception
   {
      stage = new Stage();
      perceptionModule = moduleProvider.createModule();
      uiModule = uiProvider.createUI(stage);

      stage.setOnCloseRequest((event) -> hide());
   }

   public M getPerceptionModule()
   {
      return perceptionModule;
   }

   public void stop()
   {
      Platform.runLater(() ->
                        {
                           perceptionModule.stop();
                           uiModule.stop();
                           stage.close();
                        });
   }

   public void show()
   {
      stage.show();
      uiModule.show();
   }

   public void hide()
   {
      stage.hide();
   }

   interface ModuleProvider<T>
   {
      T createModule() throws Exception;
   }

   interface UIProvider<T>
   {
      T createUI(Stage stage) throws Exception;
   }
}
