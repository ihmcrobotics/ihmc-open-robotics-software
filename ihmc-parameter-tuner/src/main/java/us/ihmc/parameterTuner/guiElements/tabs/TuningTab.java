package us.ihmc.parameterTuner.guiElements.tabs;

import javafx.geometry.Insets;
import javafx.scene.control.Label;
import javafx.scene.control.ScrollPane;
import javafx.scene.control.ScrollPane.ScrollBarPolicy;
import javafx.scene.control.Tab;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.AnchorPane;
import javafx.scene.layout.VBox;

public class TuningTab extends Tab
{
   private final Label label = new Label();
   private final TextField textField = new TextField();
   private final VBox tuningBox = new VBox();

   public TuningTab(String name)
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

      tuningBox.setFillWidth(true);
      tuningBox.setSpacing(10.0);

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

   public VBox getTuningBox()
   {
      return tuningBox;
   }

   public String getName()
   {
      return textField.getText();
   }
}
