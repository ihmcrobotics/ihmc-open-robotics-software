package us.ihmc.robotics.parameterGui.tuning;

import java.io.IOException;

import javafx.event.ActionEvent;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.CheckBox;
import javafx.scene.layout.HBox;
import us.ihmc.yoVariables.parameters.xml.Parameter;

public class BooleanTuner extends HBox
{
   private static final String FXML_PATH = "boolean_tuner.fxml";

   @FXML
   private CheckBox value;

   private final Parameter parameter;

   public BooleanTuner(Parameter parameter)
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
      value.setSelected(Boolean.parseBoolean(parameter.getValue()));
   }

   @FXML
   protected void handleButton(ActionEvent event)
   {
      parameter.setValue(Boolean.toString(value.isSelected()));
   }
}
