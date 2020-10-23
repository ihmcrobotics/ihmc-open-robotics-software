package us.ihmc.robotEnvironmentAwareness.perceptionSuite;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.messager.Messager;

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
      Platform.runLater(() ->
                        {
                           getPerceptionUI().stop();
                           getPerceptionModule().stop();
                           getStage().close();

                           this.stopInternal();
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
      T createModule(Messager messager) throws Exception;
   }

   interface UIProvider<T>
   {
      T createUI(Messager messager, Stage stage) throws Exception;
   }
}
