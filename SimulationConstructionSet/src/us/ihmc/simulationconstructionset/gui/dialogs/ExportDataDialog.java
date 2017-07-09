package us.ihmc.simulationconstructionset.gui.dialogs;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.geometry.VPos;
import javafx.scene.Scene;
import javafx.scene.control.ComboBox;
import javafx.scene.control.RadioButton;
import javafx.scene.control.ToggleGroup;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.stage.Stage;
import us.ihmc.simulationconstructionset.gui.ExportDataDialogListener;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;

import javax.swing.*;
import java.awt.*;

public class ExportDataDialog extends Stage implements EventHandler<ActionEvent> {
    public static final int
            STATE = 0, DATA = 1, COMPRESS = 5, NO_COMPRESS = 6;

    private javafx.scene.control.Button exportButton, cancelButton;
    private ExportDataPanel exportPanel;
    private JFrame parentFrame;
    private ExportDataDialogListener listener;
    private VarGroupList varGroupList;

    public ExportDataDialog(JFrame frame, VarGroupList varGroupList, ExportDataDialogListener listener) {
        super();

        this.setTitle("Export Data");

        this.parentFrame = frame;
        this.listener = listener;
        this.varGroupList = varGroupList;

        GridPane main = new GridPane();

        exportPanel = new ExportDataPanel();
        GridPane.setConstraints(exportPanel, 0, 0);
        GridPane.setMargin(exportPanel, new Insets(10, 10, 0, 10));

        // Bottom Buttons:

        exportButton = new javafx.scene.control.Button("Export");
        exportButton.addEventHandler(ActionEvent.ACTION, this);
        HBox.setMargin(exportButton, new Insets(0, 5, 0, 0));

        cancelButton = new javafx.scene.control.Button("Cancel");
        cancelButton.addEventHandler(ActionEvent.ACTION, this);

        HBox buttons = new HBox();
        buttons.setAlignment(Pos.CENTER);
        buttons.getChildren().addAll(
                exportButton,
                cancelButton
        );
        GridPane.setConstraints(buttons, 0, 1);
        GridPane.setMargin(buttons, new Insets(10, 0, 10, 0));

        main.getChildren().addAll(
                exportPanel,
                buttons
        );

        Dimension frameSize = frame.getSize();

        this.setScene(new Scene(main));
        this.setX(frameSize.width / 2);
        this.setY(frameSize.height / 4);
        this.setResizable(false);
        this.show();

        parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
    }

    @Override
    public void handle(javafx.event.ActionEvent event) {
        if (event.getSource() == cancelButton) {
            this.hide();
        } else if (event.getSource() == exportButton) {
            this.hide();
            exportPanel.export();
        }

        parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
    }

    @SuppressWarnings("serial")
    private class ExportDataPanel extends GridPane implements EventHandler<javafx.event.ActionEvent> {
        private javafx.scene.control.Label varGroupLabel;
        private ComboBox varGroupComboBox;
        private ToggleGroup stateDataButtonGroup;
        private RadioButton stateRadioButton;
        private RadioButton dataRadioButton;
        private RadioButton asciiRadioButton;
        private RadioButton matlabRadioButton;
        private RadioButton spreadsheetRadioButton;
        private RadioButton binaryRadioButton;
        private ToggleGroup dataFormatButtonGroup;
        private RadioButton compressRadioButton;
        private RadioButton noCompressRadioButton;
        private ToggleGroup compressButtonGroup;

        public ExportDataPanel() {
            varGroupLabel = new javafx.scene.control.Label("VarGroup:");
            GridPane.setConstraints(varGroupLabel, 0, 0);

            stateDataButtonGroup = new ToggleGroup();
            dataFormatButtonGroup = new ToggleGroup();
            compressButtonGroup = new ToggleGroup();

            stateRadioButton = new RadioButton("State");
            stateRadioButton.setSelected(false);
            stateRadioButton.setToggleGroup(stateDataButtonGroup);
            stateRadioButton.addEventHandler(ActionEvent.ACTION, this);
            GridPane.setConstraints(stateRadioButton, 0, 2);

            dataRadioButton = new RadioButton("Data");
            dataRadioButton.setSelected(true);
            dataRadioButton.setToggleGroup(stateDataButtonGroup);
            dataRadioButton.addEventHandler(ActionEvent.ACTION, this);
            GridPane.setConstraints(dataRadioButton, 0, 1);

            // for some reason, need this to fix whitespace above "Data" radiobutton
            GridPane.setValignment(dataRadioButton, VPos.TOP);

            binaryRadioButton = new RadioButton("Binary");
            binaryRadioButton.setSelected(true);
            binaryRadioButton.addEventHandler(ActionEvent.ACTION, this);
            binaryRadioButton.setToggleGroup(dataFormatButtonGroup);
            GridPane.setConstraints(binaryRadioButton, 1, 1);
            GridPane.setMargin(binaryRadioButton, new Insets(0, 0, 5, 0));

            asciiRadioButton = new RadioButton("ASCII (Similar to Matlab/Octave Script)");
            asciiRadioButton.setSelected(false);
            asciiRadioButton.addEventHandler(ActionEvent.ACTION, this);
            asciiRadioButton.setToggleGroup(dataFormatButtonGroup);
            GridPane.setConstraints(asciiRadioButton, 1, 2);
            GridPane.setMargin(asciiRadioButton, new Insets(0, 0, 5, 0));

            matlabRadioButton = new RadioButton("Matlab/Octave (*.mat)");
            matlabRadioButton.setSelected(false);
            matlabRadioButton.addEventHandler(ActionEvent.ACTION, this);
            matlabRadioButton.setToggleGroup(dataFormatButtonGroup);
            GridPane.setConstraints(matlabRadioButton, 1, 3);
            GridPane.setMargin(matlabRadioButton, new Insets(0, 0, 5, 0));

            spreadsheetRadioButton = new RadioButton("Comma-Separated-Values (CSV)");
            spreadsheetRadioButton.setSelected(false);
            spreadsheetRadioButton.addEventHandler(ActionEvent.ACTION, this);
            spreadsheetRadioButton.setToggleGroup(dataFormatButtonGroup);
            GridPane.setConstraints(spreadsheetRadioButton, 1, 4);

            compressRadioButton = new RadioButton("Compress");
            compressRadioButton.setSelected(true);
            compressRadioButton.addEventHandler(ActionEvent.ACTION, this);
            compressRadioButton.setToggleGroup(compressButtonGroup);
            GridPane.setConstraints(compressRadioButton, 2, 1);

            noCompressRadioButton = new RadioButton("No Compression");
            noCompressRadioButton.setSelected(false);
            noCompressRadioButton.addEventHandler(ActionEvent.ACTION, this);
            noCompressRadioButton.setToggleGroup(compressButtonGroup);
            GridPane.setConstraints(noCompressRadioButton, 2, 2);

            varGroupComboBox = new ComboBox();
            varGroupComboBox.setMaxSize(125, 25);
            varGroupComboBox.setMinSize(125, 25);
            GridPane.setConstraints(varGroupComboBox, 1, 0);
            GridPane.setMargin(varGroupComboBox, new Insets(0, 0, 5, 10));
            GridPane.setValignment(varGroupComboBox, VPos.TOP);

            // Add the VarGroups to the varGroupComboBox:
            String[] names = varGroupList.getVarGroupNames();
            for (int i = 0; i < names.length; i++) {
                varGroupComboBox.getItems().add(names[i]);
            }

            this.getChildren().addAll(
                    varGroupComboBox,
                    varGroupLabel,
                    stateRadioButton,
                    dataRadioButton,
                    binaryRadioButton,
                    asciiRadioButton,
                    matlabRadioButton,
                    spreadsheetRadioButton,
                    compressRadioButton,
                    noCompressRadioButton
            );
        }

        public void export() {
            String varGroup = varGroupComboBox.getSelectionModel().getSelectedItem().toString();
            int dataType = stateRadioButton.isSelected() ? STATE : DATA;
            SCSExportDataFormat dataFormat;
            if (asciiRadioButton.isSelected()) {
                dataFormat = SCSExportDataFormat.ASCII;
            } else if (binaryRadioButton.isSelected()) {
                dataFormat = SCSExportDataFormat.BINARY;
            } else if (matlabRadioButton.isSelected()) {
                dataFormat = SCSExportDataFormat.MATLAB;
            } else if (spreadsheetRadioButton.isSelected()) {
                dataFormat = SCSExportDataFormat.SPREADSHEET;
            } else {
                throw new RuntimeException("unknown data format");
            }

            int dataCompression = compressRadioButton.isSelected() ? COMPRESS : NO_COMPRESS;

            listener.export(varGroup, dataType, dataFormat, dataCompression);
        }

        @Override
        public void handle(javafx.event.ActionEvent event) {
            if (event.getSource() == stateRadioButton) {
                asciiRadioButton.setSelected(true);
                binaryRadioButton.setDisable(true);
                matlabRadioButton.setDisable(true);
                spreadsheetRadioButton.setDisable(true);
                noCompressRadioButton.setSelected(true);
                noCompressRadioButton.setDisable(true);
                compressRadioButton.setDisable(true);
            } else if (event.getSource() == dataRadioButton) {
                binaryRadioButton.setDisable(false);
                binaryRadioButton.setSelected(true);
                matlabRadioButton.setDisable(false);
                spreadsheetRadioButton.setDisable(false);
                compressRadioButton.setSelected(true);
                compressRadioButton.setDisable(false);
                noCompressRadioButton.setDisable(false);
            } else if (event.getSource() == asciiRadioButton) {
                noCompressRadioButton.setDisable(false);
                compressRadioButton.setDisable(false);
                noCompressRadioButton.setSelected(true);
            } else if (event.getSource() == binaryRadioButton) {
                noCompressRadioButton.setDisable(false);
                compressRadioButton.setDisable(false);
                compressRadioButton.setSelected(true);
            } else if (event.getSource() == matlabRadioButton || event.getSource() == spreadsheetRadioButton) {
                compressRadioButton.setDisable(true);
                noCompressRadioButton.setDisable(true);
                noCompressRadioButton.setSelected(true);
            }
        }
    }
}
