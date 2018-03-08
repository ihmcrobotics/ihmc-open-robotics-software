package us.ihmc.parameterTuner.guiElements.tuners;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import javafx.application.Platform;
import javafx.geometry.Orientation;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.control.Separator;
import javafx.scene.layout.VBox;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public class TuningBoxManager
{
   private final VBox tuningBox;

   private final List<String> parametersBeingTuned = new ArrayList<>();
   private final Map<String, Button> removeButtons = new HashMap<>();

   private Map<String, Tuner> tunerMap;

   public TuningBoxManager(VBox tuningBox)
   {
      this.tuningBox = tuningBox;
   }

   public void handleNewParameter(GuiParameter parameter)
   {
      String uniqueName = parameter.getUniqueName();

      if (parametersBeingTuned.contains(uniqueName))
      {
         Platform.runLater(() -> {
            Alert alert = new Alert(AlertType.INFORMATION);
            alert.setTitle("Information Dialog");
            alert.setHeaderText("Can not add parameter");
            alert.setContentText("A parameter with the same name is being tuned.");
            alert.showAndWait();
         });
         return;
      }

      Button remove = new Button("Remove");
      removeButtons.put(uniqueName, remove);
      remove.setOnAction(event -> {
         parametersBeingTuned.remove(uniqueName);
         updateView();
      });

      parametersBeingTuned.add(uniqueName);
      updateView();
   }

   public void setTunerMap(Map<String, Tuner> tunerMap)
   {
      parametersBeingTuned.clear();
      this.tunerMap = tunerMap;
      updateView();
   }

   public void updateView()
   {
      tuningBox.getChildren().clear();
      parametersBeingTuned.forEach(uniqueName ->
      {
         tuningBox.getChildren().add(tunerMap.get(uniqueName));
         tuningBox.getChildren().add(removeButtons.get(uniqueName));
         tuningBox.getChildren().add(new Separator(Orientation.HORIZONTAL));
      });
   }
}
