package us.ihmc.robotics.parameterGui.tuning;

import java.io.IOException;

import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.TextField;
import javafx.scene.layout.HBox;
import javafx.scene.layout.StackPane;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotics.parameterGui.GuiParameter;
import us.ihmc.robotics.parameterGui.ParameterTuningTools;

public class EnumTuner extends HBox
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

      String[] valueOptions = parameter.getValueOptions();
      if (valueOptions == null || valueOptions.length == 0)
      {
         createTextField(parameter);
      }
      else
      {
         createChoiceBox(parameter, valueOptions);
      }

   }

   private void createTextField(GuiParameter parameter)
   {
      TextField enumString = new TextField();
      enumString.setText(parameter.getCurrentValue());
      ParameterTuningTools.addThreadSafeListeners(enumString, () -> parameter.setValue(enumString.getText()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         enumString.setText(parameter.getCurrentValue());
      });

      tuningPane.getChildren().add(enumString);
   }

   private void createChoiceBox(GuiParameter parameter, String[] valueOptions)
   {
      ChoiceBox<String> choiceBox = new ChoiceBox<>();
      ObservableList<String> items = choiceBox.getItems();
      for (int i = 0; i < valueOptions.length; i++)
      {
         String option = valueOptions[i];
         items.add(option);
      }
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
}
