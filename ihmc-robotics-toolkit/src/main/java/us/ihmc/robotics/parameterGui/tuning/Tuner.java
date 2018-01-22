package us.ihmc.robotics.parameterGui.tuning;

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
import us.ihmc.yoVariables.parameters.xml.Parameter;

public class Tuner extends VBox
{
   private static final String FXML_PATH = "tuner.fxml";

   @FXML
   private Label name;

   @FXML
   private TextField description;

   @FXML
   private Button remove;

   public Tuner(Parameter parameter)
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

      name.setText(parameter.getName());
      description.setText(parameter.getDescription());
      description.textProperty().addListener(observable -> parameter.setDescription(description.getText()));

      switch (parameter.getType())
      {
      case "DoubleParameter":
         getChildren().add(new DoubleTuner(parameter));
         break;
      case "IntegerParameter":
         getChildren().add(new IntegerTuner(parameter));
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
