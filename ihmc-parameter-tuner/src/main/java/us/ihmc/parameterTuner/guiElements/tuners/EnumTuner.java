package us.ihmc.parameterTuner.guiElements.tuners;

import java.io.IOException;

import gnu.trove.map.TObjectIntMap;
import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Node;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import us.ihmc.commons.PrintTools;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public class EnumTuner extends HBox implements InputNode
{
   private static final String FXML_PATH = "enum_tuner.fxml";

   @FXML
   private StackPane tuningPane;

   public EnumTuner(GuiParameter parameter)
   {
      FXMLLoader loader = new FXMLLoader(getClass().getResource(FXML_PATH));
      loader.setRoot(this);
      loader.setController(this);
      try
      {
         loader.load();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }

      TObjectIntMap<String> valueOptions = parameter.getValueOptions();
      if (valueOptions == null)
      {
         createTextField(parameter);
      }
      else
      {
         createChoiceBox(parameter, valueOptions);
      }

   }

   private TextField enumString;
   private void createTextField(GuiParameter parameter)
   {
      enumString = new TextField();

      enumString.setText(parameter.getCurrentValue());
      ParameterTuningTools.addThreadSafeListeners(enumString, () -> parameter.setValue(enumString.getText()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         enumString.setText(parameter.getCurrentValue());
      });

      tuningPane.getChildren().add(enumString);
   }

   private ChoiceBox<String> choiceBox;
   private void createChoiceBox(GuiParameter parameter, TObjectIntMap<String> valueOptions)
   {
      choiceBox = new ChoiceBox<>();

      ObservableList<String> items = choiceBox.getItems();
      valueOptions.forEachKey(option -> {
         items.add(option);
         return true;
      });
      setChoiceBoxValue(parameter, choiceBox);

      choiceBox.getSelectionModel().selectedItemProperty().addListener((observable, oldValue, newValue) -> {
         Platform.runLater(() -> parameter.setValue(choiceBox.getSelectionModel().getSelectedItem()));
      });

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         setChoiceBoxValue(parameter, choiceBox);
      });

      tuningPane.getChildren().add(choiceBox);
   }

   private void setChoiceBoxValue(GuiParameter parameter, ChoiceBox<String> choiceBox)
   {
      ObservableList<String> items = choiceBox.getItems();
      String newValue = parameter.getCurrentValue();
      if (!items.contains(newValue))
      {
         PrintTools.info("Enum parameter " + parameter.getName() + " did not provide the current value as option.");
         items.add(newValue);
      }
      choiceBox.getSelectionModel().select(newValue);
   }

   @Override
   public Node getSimpleInputNode()
   {
      if (choiceBox != null)
      {
         ChoiceBox<String> duplicate = new ChoiceBox<>(choiceBox.getItems());
         duplicate.setSelectionModel(choiceBox.getSelectionModel());
         return duplicate;
      }
      else
      {
         TextField duplicate = new TextField(enumString.getText());
         enumString.textProperty().addListener((observable, oldValue, newValue) -> duplicate.setText(newValue));
         duplicate.textProperty().addListener((observable, oldValue, newValue) -> enumString.setText(newValue));
         return null;
      }
   }
}
