package us.ihmc.robotics.parameterGui.tuning;

import java.util.ArrayList;
import java.util.List;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.layout.VBox;
import us.ihmc.yoVariables.parameters.xml.Parameter;

public class TuningBoxManager
{
   private final VBox tuningBox;
   private final List<String> parametersBeingTuned = new ArrayList<>();

   public TuningBoxManager(VBox tuningBox)
   {
      this.tuningBox = tuningBox;
   }

   public void handleNewParameter(Parameter parameter)
   {
      String parameterName = parameter.getName();
      if (parametersBeingTuned.contains(parameterName))
      {
         Alert alert = new Alert(AlertType.INFORMATION);
         alert.setTitle("Information Dialog");
         alert.setHeaderText("Can not add parameter");
         alert.setContentText("A parameter with the same name is being tuned.");
         alert.showAndWait();
         return;
      }

      parametersBeingTuned.add(parameterName);
      Tuner tuner = new Tuner(parameter);
      EventHandler<ActionEvent> closeHandler = event -> {
         tuningBox.getChildren().remove(tuner);
         parametersBeingTuned.remove(parameterName);
      };
      tuner.setCloseHandler(closeHandler);
      tuningBox.getChildren().add(tuner);
   }

   public void clearAllParameters()
   {
      tuningBox.getChildren().clear();
      parametersBeingTuned.clear();
   }
}
