package us.ihmc.robotics.parameterGui.tuning;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.TextField;
import javafx.scene.layout.HBox;
import us.ihmc.robotics.parameterGui.GuiParameter;
import us.ihmc.robotics.parameterGui.ParameterTuningTools;

public class EnumTuner extends HBox
{
   private static final String FXML_PATH = "enum_tuner.fxml";

   @FXML
   private TextField enumString;

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

      enumString.setText(parameter.getCurrentValue());
      ParameterTuningTools.addThreadSafeListeners(enumString, () -> parameter.setValue(enumString.getText()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         enumString.setText(parameter.getCurrentValue());
      });
   }
}
