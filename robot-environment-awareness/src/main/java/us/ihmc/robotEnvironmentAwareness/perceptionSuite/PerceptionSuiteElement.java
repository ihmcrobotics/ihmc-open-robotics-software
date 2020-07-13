package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.application.Platform;
import javafx.stage.Stage;

public interface PerceptionSuiteElement<M extends PerceptionModule, U extends PerceptionUI>
{
   M getPerceptionModule();

   U getPerceptionUI();

   Stage getStage();

   default void stopInternal()
   {
   }

   default void stop()
   {
      stopInternal();

      Platform.runLater(() ->
                        {
                           getPerceptionModule().stop();
                           getPerceptionUI().stop();
                           getStage().close();
                        });
   }

   default void show()
   {
      getStage().show();
      getPerceptionUI().show();
   }

   default void hide()
   {
      getStage().hide();
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
