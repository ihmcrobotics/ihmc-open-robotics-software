package us.ihmc.simulationconstructionset.gui;

import javafx.event.EventHandler;
import javafx.scene.control.*;
import javafx.scene.layout.GridPane;
import javafx.scene.text.Font;
import us.ihmc.yoVariables.dataBuffer.DataEntry;

public class VarPropertiesPanel extends GridPane implements EventHandler<javafx.event.ActionEvent> {

    private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

    private TextField minTextField, maxTextField;
    private RadioButton autoButton, manualButton;
    private CheckBox invertCheckBox;

    private final DataEntry entry;

    private double newMinVal, newMaxVal;

    public VarPropertiesPanel(DataEntry entry) {
        super();

        this.entry = entry;

        // entry.reCalcMinMax(); TODO: can this be removed?
        newMinVal = entry.getManualMinScaling();
        newMaxVal = entry.getManualMaxScaling();

        Label varLabel = new Label(entry.getVariableName());
        varLabel.setFont(new Font(Font.getDefault().getName(), 14));
        GridPane.setConstraints(varLabel, 0, 0);

        ToggleGroup group = new ToggleGroup();

        // Row 0:

        Label scalingLabel = new Label("Scaling:");
        GridPane.setConstraints(scalingLabel, 0, 1);

        autoButton = new RadioButton("Auto");
        autoButton.setSelected(entry.isAutoScaleEnabled());
        autoButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        autoButton.setToggleGroup(group);
        GridPane.setConstraints(autoButton, 1, 1);

        manualButton = new RadioButton("Manual");
        manualButton.setSelected(!entry.isAutoScaleEnabled());
        manualButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        manualButton.setToggleGroup(group);
        GridPane.setConstraints(manualButton, 3, 1);

        // Row 1:

        Label settingsLabel = new Label("Manual Settings:");
        GridPane.setConstraints(settingsLabel, 0, 2);

        Label minSettingsLabel = new Label("Min:");
        GridPane.setConstraints(minSettingsLabel, 1, 2);

        String minValString = numFormat.format(newMinVal);
        minTextField = new TextField(minValString);
        minTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        if (entry.isAutoScaleEnabled()) {
            minTextField.setDisable(true);
        }
        GridPane.setConstraints(minTextField, 2, 2);

        Label maxSettingsLabel = new Label("Max:");
        GridPane.setConstraints(maxSettingsLabel, 4, 2);

        String maxValString = numFormat.format(newMaxVal);
        maxTextField = new TextField(maxValString);
        maxTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        if (entry.isAutoScaleEnabled()) {
            maxTextField.setDisable(true);
        }
        GridPane.setConstraints(maxTextField, 5, 2);

        // Row 2:

        Label rangeLabel = new Label("Data Range:");
        GridPane.setConstraints(rangeLabel, 0, 3);

        Label minRangeLabel = new Label("Min:");
        GridPane.setConstraints(minRangeLabel, 1, 3);

        minValString = numFormat.format(entry.getMin());
        Label minTextLabel = new Label(minValString);
        GridPane.setConstraints(minTextLabel, 2, 3);

        Label maxRangeLabel = new Label("Max:");
        GridPane.setConstraints(maxRangeLabel, 4, 3);

        maxValString = numFormat.format(entry.getMax());
        Label maxTextLabel = new Label(maxValString);
        GridPane.setConstraints(maxTextLabel, 5, 3);

        invertCheckBox = new CheckBox("Invert");
        invertCheckBox.setSelected(entry.getInverted());
        GridPane.setConstraints(invertCheckBox, 0, 4);

        this.getChildren().addAll(
                varLabel,
                scalingLabel,
                autoButton,
                manualButton,
                settingsLabel,
                minSettingsLabel,
                minTextField,
                maxSettingsLabel,
                maxTextField,
                rangeLabel,
                minRangeLabel,
                minTextLabel,
                maxRangeLabel,
                maxTextLabel,
                invertCheckBox
        );
    }

    public void commitChanges() {
        updateMinTextField();
        updateMaxTextField();

        entry.setManualScaling(newMinVal, newMaxVal);

        if (this.autoButton.isSelected())
            entry.enableAutoScale(true);
        else
            entry.enableAutoScale(false);


        entry.setInverted(invertCheckBox.isSelected());
    }

    public void updateMaxTextField() {
        String text = maxTextField.getText();

        try {
            newMaxVal = Double.valueOf(text);
        } catch (NumberFormatException e) {
            maxTextField.setText(numFormat.format(newMaxVal));
        }

    }

    public void updateMinTextField() {
        String text = minTextField.getText();

        try {
            newMinVal = Double.valueOf(text);
        } catch (NumberFormatException e) {
            minTextField.setText(numFormat.format(newMinVal));
        }
    }

    @Override
    public void handle(javafx.event.ActionEvent event) {
        if (event.getSource() == maxTextField) {
            updateMaxTextField();
        } else if (event.getSource() == minTextField) {
            updateMinTextField();
        } else if (event.getSource() == autoButton) {
            maxTextField.setDisable(true);
            minTextField.setDisable(true);
        } else if (event.getSource() == manualButton) {
            maxTextField.setDisable(false);
            minTextField.setDisable(false);
        }
    }
}
