package us.ihmc.parameterTuner.guiElements.tabs;

import java.util.Map;

import javafx.geometry.Insets;
import javafx.scene.control.Label;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.ScrollPane.ScrollBarPolicy;
import javafx.scene.control.Tab;
import javafx.scene.control.TabPane;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.VBox;
import us.ihmc.parameterTuner.guiElements.GuiParameter;
import us.ihmc.parameterTuner.guiElements.tuners.Tuner;
import us.ihmc.parameterTuner.guiElements.tuners.TuningBoxManager;

public class TuningTab extends Tab
{
   private final Label label = new Label();
   private final TextField textField = new TextField();

   private final TuningBoxManager tuningBoxManager;

   public TuningTab(String name, TabPane parent)
   {
      textField.setText(name);
      showLabel();

      label.setOnMouseClicked(event -> {
         if (event.getClickCount() == 2)
         {
            showTextInput();
         }
      });
      textField.setOnKeyPressed(event -> {
         if (event.getCode() == KeyCode.ENTER)
         {
            showLabel();
         }
      });
      textField.focusedProperty().addListener((observable, oldValue, newValue) -> {
         if (oldValue && !newValue)
         {
            showLabel();
         }
      });

      VBox tuningBox = new VBox();
      tuningBox.setFillWidth(true);
      tuningBox.setSpacing(10.0);
      tuningBoxManager = new TuningBoxManager(tuningBox);

      ScrollPane scrollPane = new ScrollPane();
      AnchorPane.setLeftAnchor(scrollPane, 0.0);
      AnchorPane.setRightAnchor(scrollPane, 0.0);
      AnchorPane.setTopAnchor(scrollPane, 0.0);
      AnchorPane.setBottomAnchor(scrollPane, 0.0);

      scrollPane.setHbarPolicy(ScrollBarPolicy.NEVER);
      scrollPane.setFitToWidth(true);
      scrollPane.setPadding(new Insets(10.0, 10.0, 10.0, 10.0));

      AnchorPane anchorPane = new AnchorPane();
      setContent(anchorPane);
      anchorPane.getChildren().add(scrollPane);
      scrollPane.setContent(tuningBox);

      parent.getSelectionModel().selectedItemProperty().addListener((observable, oldValue, newValue) -> {
         if (this == newValue)
         {
            tuningBoxManager.updateView();
         }
      });
   }

   private void showTextInput()
   {
      textField.setText(label.getText());
      textField.requestFocus();
      setGraphic(textField);
   }

   private void showLabel()
   {
      label.setText(textField.getText());
      setGraphic(label);
   }

   public String getName()
   {
      return textField.getText();
   }

   public void setTunerMap(Map<String, Tuner> tunerMap)
   {
      tuningBoxManager.setTunerMap(tunerMap);
   }

   public void handleNewParameter(GuiParameter parameter)
   {
      tuningBoxManager.handleNewParameter(parameter);
   }
}
