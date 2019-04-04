package us.ihmc.parameterTuner;

import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.layout.Region;

public class JavaFXExceptionTools
{
   public static void setupExceptionHandling()
   {
      Thread.setDefaultUncaughtExceptionHandler((t, e) -> {
         createExceptionDialog(e);
      });
   }

   public static void createExceptionDialog(Throwable e)
   {
      e.printStackTrace();

      Alert alert = new Alert(AlertType.ERROR);
      alert.setTitle("Exception");
      alert.setHeaderText("An exception was thrown.\nSee console for stacktrace.");
      alert.setContentText(e.getMessage());
      alert.getDialogPane().setMinHeight(Region.USE_PREF_SIZE);
      alert.getDialogPane().setMinWidth(Region.USE_PREF_SIZE);
      alert.resizableProperty().set(true);
      alert.showAndWait();
   }
}
