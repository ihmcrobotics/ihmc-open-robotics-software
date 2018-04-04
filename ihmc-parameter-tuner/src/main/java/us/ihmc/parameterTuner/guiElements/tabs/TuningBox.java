package us.ihmc.parameterTuner.guiElements.tabs;

import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.stream.Collectors;

import javafx.application.Platform;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Node;
import javafx.scene.control.Alert;
import javafx.scene.control.Alert.AlertType;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.image.Image;
import javafx.scene.image.ImageView;
import javafx.scene.input.ClipboardContent;
import javafx.scene.input.DragEvent;
import javafx.scene.input.Dragboard;
import javafx.scene.input.MouseEvent;
import javafx.scene.input.TransferMode;
import javafx.scene.layout.HBox;
import javafx.scene.layout.Region;
import javafx.scene.layout.VBox;
import javafx.scene.shape.Rectangle;
import us.ihmc.commons.PrintTools;
import us.ihmc.parameterTuner.guiElements.GuiElement;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;

public class TuningBox extends VBox
{
   private static final double imageSize = 20.0;
   private static final double buttonPadding = 5.0;

   private final Image closeImage;
   private final Image moveImage;
   private final Image sliderImage;

   private final List<String> parametersBeingTuned = new ArrayList<>();
   private final Map<String, Button> removeButtons = new HashMap<>();
   private final Map<String, Button> sliderButtons = new HashMap<>();
   private final Map<String, Node> dragNodes = new HashMap<>();
   private final Map<String, EventHandler<DragEvent>> dragOverEvents = new HashMap<>();

   private Map<String, Tuner> tunerMap;

   private final Rectangle placeholder = new Rectangle(100, 3);

   public TuningBox()
   {
      setFillWidth(true);
      setSpacing(10.0);
      closeImage = new Image(TuningBox.class.getResourceAsStream("/close.png"));
      moveImage = new Image(TuningBox.class.getResourceAsStream("/move.png"));
      sliderImage = new Image(TuningBox.class.getResourceAsStream("/sliders.png"));
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

      String[] split = uniqueName.split(GuiElement.SEPERATOR);
      String simpleName = split[split.length - 1];

      Button remove = new Button(null);
      remove.setPadding(new Insets(buttonPadding));
      ImageView removeGraphic = new ImageView(closeImage);
      removeGraphic.setFitWidth(imageSize);
      removeGraphic.setFitHeight(imageSize);
      remove.setGraphic(removeGraphic);
      removeButtons.put(uniqueName, remove);
      remove.setOnAction(event -> {
         parametersBeingTuned.remove(uniqueName);
         updateView();
      });

      Button slider = new Button(null);
      slider.setPadding(new Insets(buttonPadding));
      ImageView sliderGraphic = new ImageView(sliderImage);
      sliderGraphic.setFitWidth(imageSize);
      sliderGraphic.setFitHeight(imageSize);
      slider.setGraphic(sliderGraphic);
      sliderButtons.put(uniqueName, slider);
      slider.setOnAction(event -> {
         // TODO: add some code to make this work.
      });

      Label drag = new Label();
      ImageView moveGraphic = new ImageView(moveImage);
      moveGraphic.setFitWidth(imageSize);
      moveGraphic.setFitHeight(imageSize);
      drag.setGraphic(moveGraphic);
      dragNodes.put(uniqueName, drag);
      // When a drag starts remove the parameter from the tuning box and replace it with the placeholder.
      drag.setOnDragDetected(new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            Dragboard dragboard = drag.startDragAndDrop(TransferMode.MOVE);
            dragboard.setDragView(TextImage.create(simpleName), -15.0, 5.0);
            ClipboardContent clipboardContent = new ClipboardContent();
            clipboardContent.putString(uniqueName);
            dragboard.setContent(clipboardContent);

            int oldIndex = parametersBeingTuned.lastIndexOf(uniqueName);
            parametersBeingTuned.remove(uniqueName);
            updateView();
            getChildren().add(oldIndex, placeholder);
         }
      });
      // When a drag ends insert the dragged parameter at the placeholder position.
      drag.setOnDragDone(new EventHandler<DragEvent>()
      {
         @Override
         public void handle(DragEvent event)
         {
            String uniqueNameToBeDropped = event.getDragboard().getString();
            int newIndex = getChildren().lastIndexOf(placeholder);
            if (uniqueNameToBeDropped == null || uniqueNameToBeDropped.isEmpty() || newIndex == -1)
            {
               event.consume();
               return;
            }

            parametersBeingTuned.add(newIndex, uniqueNameToBeDropped);
            updateView();
            event.consume();
         }
      });
      // As the drag passes over the parameters being tunes move the placeholder accordingly.
      dragOverEvents.put(uniqueName, new EventHandler<DragEvent>()
      {
         @Override
         public void handle(DragEvent event)
         {
            event.acceptTransferModes(TransferMode.MOVE);

            String uniqueNameToBeDropped = event.getDragboard().getString();
            int newIndex = parametersBeingTuned.lastIndexOf(uniqueName);
            if (uniqueNameToBeDropped == null || uniqueNameToBeDropped.isEmpty() || newIndex == -1)
            {
               event.consume();
               return;
            }
            if (event.getGestureTarget() == null)
            {
               event.consume();
               return;
            }
            if (!(event.getGestureTarget() instanceof Region))
            {
               PrintTools.warn("Gesture Target should be a region. Is a " + event.getGestureTarget().getClass().getSimpleName());
               event.consume();
               return;
            }
            if (2.0 * event.getY() > ((Region) event.getGestureTarget()).getHeight())
            {
               newIndex++;
            }

            getChildren().remove(placeholder);
            getChildren().add(newIndex, placeholder);
            event.consume();
         }
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
      parametersBeingTuned.forEach(uniqueName -> {
         HBox box = new HBox(10.0);
         box.setAlignment(Pos.CENTER_LEFT);
         box.getChildren().add(dragNodes.get(uniqueName));
         box.getChildren().add(sliderButtons.get(uniqueName));
         box.getChildren().add(removeButtons.get(uniqueName));
         box.getChildren().add(tunerMap.get(uniqueName));

         box.setOnDragOver(dragOverEvents.get(uniqueName));

         getChildren().add(box);
      });
   }

   public List<String> getParameterUniqueNames()
   {
      return Collections.unmodifiableList(parametersBeingTuned);
   }
}
