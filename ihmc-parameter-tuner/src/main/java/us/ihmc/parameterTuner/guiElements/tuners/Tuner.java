package us.ihmc.parameterTuner.guiElements.tuners;

import java.io.IOException;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.VBox;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.TextFormatterTools;
import us.ihmc.parameterTuner.ParameterTuningTools;
import us.ihmc.parameterTuner.guiElements.GuiParameter;

public class Tuner extends VBox
{
   private static final int MAX_DESCRIPTION_CHARACTERS = 255;

   private static final String FXML_PATH = "tuner.fxml";

   @FXML
   private Label name;

   @FXML
   private TextField description;

   @FXML
   private Button remove;

   public Tuner(GuiParameter parameter)
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

      setId("tuner-window");
      name.setId("parameter-name-in-tuner");

      name.setText(parameter.getName());
      description.setText(parameter.getCurrentDescription());
      description.setTextFormatter(TextFormatterTools.maxLengthTextFormatter(MAX_DESCRIPTION_CHARACTERS));
      ParameterTuningTools.addThreadSafeListeners(description, () -> parameter.setDescription(description.getText()));

      parameter.addChangedListener(p -> {
         // This listener will be triggered by an external change and is called from the animation timer.
         description.setText(parameter.getCurrentDescription());
      });

      switch (parameter.getType())
      {
      case "DoubleParameter":
         getChildren().add(new DoubleTuner(parameter));
         break;
      case "IntegerParameter":
         getChildren().add(new IntegerTuner(parameter));
         break;
      case "BooleanParameter":
         getChildren().add(new BooleanTuner(parameter));
         break;
      case "EnumParameter":
         getChildren().add(new EnumTuner(parameter));
         break;
      default:
         PrintTools.info("Implement me.");
      }
   }

   public void setCloseHandler(EventHandler<ActionEvent> closeHandler)
   {
      remove.setOnAction(closeHandler);
   }
}
