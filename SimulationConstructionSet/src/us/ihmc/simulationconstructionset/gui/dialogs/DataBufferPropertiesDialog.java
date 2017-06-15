package us.ihmc.simulationconstructionset.gui.dialogs;

import javafx.event.ActionEvent;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.geometry.HPos;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.*;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.RowConstraints;
import javafx.stage.Stage;
import us.ihmc.yoVariables.dataBuffer.DataBuffer;
import us.ihmc.simulationconstructionset.gui.DataBufferChangeListener;

import javax.swing.*;
import java.awt.*;

public class DataBufferPropertiesDialog extends Stage implements EventHandler {
    private TextField maxTextField, currentTextField;
    private RadioButton wrapButton, enlargeButton;

    private int newMaxVal, newCurrentVal;

    private Button okButton, applyButton, cancelButton;
    private BufferPropertiesPanel bufferPropertiesPane;

    private DataBufferChangeListener listener;
    private DataBuffer dataBuffer;

    public DataBufferPropertiesDialog(Container parentContainer, JFrame frame, DataBuffer dataBuffer, DataBufferChangeListener listener) {
        super();

        this.setTitle("Data Buffer Properties");

        this.listener = listener;
        this.dataBuffer = dataBuffer;

        GridPane pane = new GridPane();

        RowConstraints properties = new RowConstraints();
        properties.setPercentHeight(75.00);
        RowConstraints buttons = new RowConstraints();
        buttons.setPercentHeight(25.00);

        pane.getRowConstraints().addAll(
                properties,
                buttons
        );

        bufferPropertiesPane = new BufferPropertiesPanel();
        GridPane.setConstraints(bufferPropertiesPane, 1, 0);
        //GridPane.setMargin(bufferPropertiesPane, new Insets(10, 10, 5, 10));

        okButton = new Button("OK");
        okButton.addEventHandler(ActionEvent.ACTION, this);

        applyButton = new Button("Apply");
        applyButton.addEventHandler(ActionEvent.ACTION, this);
        HBox.setMargin(applyButton, new Insets(0, 5, 0, 5));

        cancelButton = new Button("Cancel");
        cancelButton.addEventHandler(ActionEvent.ACTION, this);

        HBox hbox = new HBox();
        hbox.setAlignment(Pos.CENTER);
        hbox.getChildren().addAll(
                okButton,
                applyButton,
                cancelButton
        );
        hbox.setPrefSize(200, okButton.getHeight());
        GridPane.setConstraints(hbox, 1, 1);

        pane.getChildren().addAll(
                bufferPropertiesPane,
                hbox
        );

        Dimension frameSize = parentContainer.getSize();

        this.setScene(new Scene(pane));
        this.setX(frameSize.width / 2);
        this.setY(frameSize.height / 4);
        this.setResizable(false);
        this.show();
    }

    @Override
    public void handle(Event event) {
        if (event.getSource() == cancelButton) {
            this.close();
        } else if (event.getSource() == applyButton) {
            bufferPropertiesPane.commitChanges();
        } else if (event.getSource() == okButton) {
            bufferPropertiesPane.commitChanges();
            this.close();
        }

        if (listener != null) {
            listener.dataBufferChanged();
        }
    }

    public class BufferPropertiesPanel extends GridPane implements EventHandler {
        public BufferPropertiesPanel() {
            super();

            // Constraints to balance rows
            RowConstraints radios = new RowConstraints();
            radios.setPercentHeight(34.00);
            RowConstraints max = new RowConstraints();
            max.setPercentHeight(33.00);
            RowConstraints current = new RowConstraints();
            current.setPercentHeight(33.00);

            // Add all to this (since this is a GridPane)
            this.getRowConstraints().addAll(
                    radios,
                    max,
                    current
            );

            newMaxVal = dataBuffer.getMaxBufferSize();
            newCurrentVal = dataBuffer.getBufferSize();

            // RadioButton toggling group for Wrap and Enlarge
            ToggleGroup group = new ToggleGroup();

            // RadioButtons:
            // Wrap
            wrapButton = new RadioButton("Wrap");
            wrapButton.addEventHandler(ActionEvent.ACTION, this);
            wrapButton.setToggleGroup(group);
            GridPane.setConstraints(wrapButton, 1, 0);
            GridPane.setHalignment(wrapButton, HPos.CENTER);

            // Enlarge
            enlargeButton = new RadioButton("Enlarge");
            enlargeButton.addEventHandler(ActionEvent.ACTION, this);
            enlargeButton.setToggleGroup(group);
            GridPane.setConstraints(enlargeButton, 2, 0);
            GridPane.setHalignment(enlargeButton, HPos.CENTER);

            // Max Settings:
            // Label
            Label maxSettingsLabel = new Label("Max Size:");
            GridPane.setConstraints(maxSettingsLabel, 1, 1);
            GridPane.setHalignment(maxSettingsLabel, HPos.RIGHT);

            // TextField
            String maxValString = String.valueOf(newMaxVal);
            maxTextField = new TextField(maxValString);
            maxTextField.addEventHandler(ActionEvent.ACTION, this);
            GridPane.setConstraints(maxTextField, 2, 1);
            GridPane.setHalignment(maxTextField, HPos.LEFT);
            GridPane.setMargin(maxTextField, new Insets(5, 5, 5, 5));

            // Current Settings:
            // Label
            Label currentSettingsLabel = new Label("Current Size:");
            GridPane.setConstraints(currentSettingsLabel, 1, 2);
            GridPane.setHalignment(currentSettingsLabel, HPos.RIGHT);

            // TextField
            String currentValString = String.valueOf(newCurrentVal);
            currentTextField = new TextField(currentValString);
            currentTextField.addEventHandler(ActionEvent.ACTION, this);
            GridPane.setConstraints(currentTextField, 2, 2);
            GridPane.setHalignment(currentTextField, HPos.LEFT);
            GridPane.setMargin(currentTextField, new Insets(5, 5, 5, 5));

            // Set Wrap as default selected RadioButton and disable max TextField
            group.selectToggle(wrapButton);
            maxTextField.setDisable(true);

            // Add all to this (since this is a GridPane)
            this.getChildren().addAll(
                    wrapButton,
                    enlargeButton,
                    maxSettingsLabel,
                    maxTextField,
                    currentSettingsLabel,
                    currentTextField
            );
        }

        public void commitChanges() {
            updateMaxTextField();
            updateCurrentTextField();

            dataBuffer.setMaxBufferSize(newMaxVal);
            dataBuffer.changeBufferSize(newCurrentVal);

            dataBuffer.setWrapBuffer(wrapButton.isSelected());
        }

        @Override
        public void handle(Event event) {
            if (event.getSource() == maxTextField) {
                updateMaxTextField();
            } else if (event.getSource() == currentTextField) {
                updateCurrentTextField();
            } else if (event.getSource() == wrapButton) {
                maxTextField.setDisable(true);
            } else if (event.getSource() == enlargeButton) {
                maxTextField.setDisable(false);
            }
        }

        public void updateMaxTextField() {
            String text = maxTextField.getText();

            try {
                newMaxVal = Integer.parseInt(text);
            } catch (NumberFormatException e) {
                maxTextField.setText(String.valueOf(newMaxVal));
            }
        }

        public void updateCurrentTextField() {
            String text = currentTextField.getText();

            try {
                newCurrentVal = Integer.parseInt(text);
            } catch (NumberFormatException e) {
                currentTextField.setText(String.valueOf(newCurrentVal));
            }
        }
    }
}
