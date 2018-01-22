package us.ihmc.robotics.parameterGui.tuning;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.TextField;
import javafx.scene.layout.HBox;
import us.ihmc.yoVariables.parameters.xml.Parameter;

public class EnumTuner extends HBox
{
   private static final String FXML_PATH = "enum_tuner.fxml";

   @FXML
   private TextField enumString;

   public EnumTuner(Parameter parameter)
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

      enumString.setText(parameter.getValue());
      enumString.textProperty().addListener((observable, oldValue, newValue) -> parameter.setValue(enumString.getText()));
   }
}
