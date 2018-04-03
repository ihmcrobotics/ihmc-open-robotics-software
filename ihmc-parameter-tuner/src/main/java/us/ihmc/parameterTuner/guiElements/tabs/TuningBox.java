package us.ihmc.parameterTuner.guiElements.tabs;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javafx.application.Platform;
import javafx.geometry.Pos;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;

public class TuningBox extends VBox
{
   private static final double buttonSize = 12.0;
   private final Image image;

   private final List<String> parametersBeingTuned = new ArrayList<>();
   private final Map<String, Button> removeButtons = new HashMap<>();

   private Map<String, Tuner> tunerMap;

   public TuningBox()
   {
      setFillWidth(true);
      setSpacing(10.0);
      image = new Image(TuningBox.class.getResourceAsStream("/close.png"));
   }

   public void handleNewParameter(String uniqueName)
   {
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

      Button remove = new Button(null);
      ImageView graphic = new ImageView(image);
      graphic.setFitWidth(buttonSize);
      graphic.setFitHeight(buttonSize);
      remove.setGraphic(graphic);
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
      this.tunerMap = tunerMap;
      List<String> invalidParameters = parametersBeingTuned.stream().filter(name -> !tunerMap.containsKey(name)).collect(Collectors.toList());
      invalidParameters.forEach(name -> parametersBeingTuned.remove(name));
      updateView();
   }

   public void updateView()
   {
      getChildren().clear();
      parametersBeingTuned.forEach(uniqueName ->
      {
         HBox box = new HBox(10.0);
         box.setAlignment(Pos.CENTER_LEFT);
         box.getChildren().add(removeButtons.get(uniqueName));
         box.getChildren().add(tunerMap.get(uniqueName));
         getChildren().add(box);
      });
   }

   public List<String> getParameterUniqueNames()
   {
      return Collections.unmodifiableList(parametersBeingTuned);
   }
}
