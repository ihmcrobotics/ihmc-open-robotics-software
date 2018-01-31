package us.ihmc.parameterTuner.guiElements.tuners;

import java.io.IOException;

import javafx.application.Platform;
import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.CheckBox;
import javafx.scene.layout.HBox;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public class BooleanTuner extends HBox
{
   private static final String FXML_PATH = "boolean_tuner.fxml";

   @FXML
   private CheckBox value;

   private final GuiParameter parameter;

   public BooleanTuner(GuiParameter parameter)
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

      this.parameter = parameter;
      value.setSelected(Boolean.parseBoolean(parameter.getCurrentValue()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         value.setSelected(Boolean.parseBoolean(parameter.getCurrentValue()));
      });
   }

   @FXML
   protected void handleButton(ActionEvent event)
   {
      Platform.runLater(() -> parameter.setValue(Boolean.toString(value.isSelected())));
   }
}
