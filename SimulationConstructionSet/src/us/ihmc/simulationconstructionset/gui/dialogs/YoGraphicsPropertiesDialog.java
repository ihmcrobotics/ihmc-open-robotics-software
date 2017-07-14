package us.ihmc.simulationconstructionset.gui.dialogs;

import javafx.event.ActionEvent;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.RowConstraints;
import javafx.stage.Stage;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;

import javax.swing.*;
import java.awt.*;

public class YoGraphicsPropertiesDialog extends Stage implements EventHandler<ActionEvent> {
    private Button okButton, applyButton, cancelButton;
    private YoGraphicsPropertiesPanel yoGraphicsPropertiesPanel;
    private Container parentContainer;

    private SimulationConstructionSet sim;

    public YoGraphicsPropertiesDialog(Container parentContainer, JFrame ownerFrame, SimulationConstructionSet sim) {
        super();

        this.setTitle("YoGraphics Properties");

        this.parentContainer = parentContainer;
        this.sim = sim;

        GridPane pane = new GridPane();

        RowConstraints properties = new RowConstraints();
        properties.setPercentHeight(50.00);
        RowConstraints buttons = new RowConstraints();
        buttons.setPercentHeight(50.00);

        pane.getRowConstraints().addAll(
                properties,
                buttons
        );

        yoGraphicsPropertiesPanel = new YoGraphicsPropertiesPanel();
        GridPane.setConstraints(yoGraphicsPropertiesPanel, 0, 0);
        GridPane.setMargin(yoGraphicsPropertiesPanel, new javafx.geometry.Insets(10, 10, 5, 10));

        okButton = new Button("OK");
        okButton.addEventHandler(ActionEvent.ACTION, this);

        applyButton = new Button("Apply");
        applyButton.addEventHandler(ActionEvent.ACTION, this);
        HBox.setMargin(applyButton, new javafx.geometry.Insets(0, 5, 0, 5));

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
        GridPane.setConstraints(hbox, 0, 1);

        pane.getChildren().addAll(
                yoGraphicsPropertiesPanel,
                hbox
        );

        parentContainer.repaint(); // This is a horrible way to get the graphs to repaint...

        Dimension frameSize = parentContainer.getSize();

        this.setScene(new Scene(pane));
        this.setX(frameSize.width / 2);
        this.setY(frameSize.height / 4);
        this.setResizable(false);
        this.show();
    }

    @Override
    public void handle(ActionEvent event) {
        if (event.getSource() == cancelButton) {
            this.hide();
        } else if (event.getSource() == applyButton) {
            yoGraphicsPropertiesPanel.commitChanges();
        } else if (event.getSource() == okButton) {
            yoGraphicsPropertiesPanel.commitChanges();
            this.hide();
        }

        parentContainer.repaint();
    }

    public class YoGraphicsPropertiesPanel extends GridPane implements EventHandler {
        private double newYoGraphicsGlobalScaleVal;
        private TextField yoGraphicsGlobalScaleTextField;
        private Label yoGraphicsGlobalScaleLabel;

        public YoGraphicsPropertiesPanel() {
            yoGraphicsGlobalScaleLabel = new Label("YoGraphics Global Scale:");

            yoGraphicsGlobalScaleTextField = new TextField();
            yoGraphicsGlobalScaleTextField.setPrefSize(60, 21);

            GridPane.setConstraints(yoGraphicsGlobalScaleLabel, 0, 0);
            GridPane.setConstraints(yoGraphicsGlobalScaleTextField, 1, 0);

            newYoGraphicsGlobalScaleVal = sim.getGlobalYoGraphicsScale();
            yoGraphicsGlobalScaleTextField.setText(String.valueOf(newYoGraphicsGlobalScaleVal));

            this.getChildren().addAll(
                    yoGraphicsGlobalScaleLabel,
                    yoGraphicsGlobalScaleTextField
            );
        }

        public void commitChanges() {
            updateYoGraphicsGlobalScaleTextField();

            sim.setYoGraphicsGlobalScale(newYoGraphicsGlobalScaleVal);
        }

        private void updateYoGraphicsGlobalScaleTextField() {
            String text = yoGraphicsGlobalScaleTextField.getText();

            try {
                newYoGraphicsGlobalScaleVal = Double.parseDouble(text);

                if (newYoGraphicsGlobalScaleVal < 0.0) {
                    newYoGraphicsGlobalScaleVal = Math.abs(newYoGraphicsGlobalScaleVal);
                    yoGraphicsGlobalScaleTextField.setText(String.valueOf(newYoGraphicsGlobalScaleVal));
                }
            } catch (NumberFormatException e) {
                yoGraphicsGlobalScaleTextField.setText(String.valueOf(newYoGraphicsGlobalScaleVal));
            }
        }

        @Override
        public void handle(Event event) {
            if (event.getSource() == yoGraphicsGlobalScaleTextField) {
                updateYoGraphicsGlobalScaleTextField();
            }
        }
    }
}
