package us.ihmc.simulationconstructionset.gui.dialogs;

import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.layout.*;
import javafx.stage.Stage;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraPropertiesHolder;
import us.ihmc.simulationconstructionset.gui.ActiveCameraHolder;

import javax.swing.*;
import java.awt.*;
import java.text.NumberFormat;

public class CameraPropertiesDialog extends Stage implements EventHandler<javafx.event.ActionEvent> {
    private NumberFormat numFormat;

    private CheckBox trackCheckBox, trackXCheckBox, trackYCheckBox, trackZCheckBox;
    private CheckBox dollyCheckBox, dollyXCheckBox, dollyYCheckBox, dollyZCheckBox;

    private TextField trackXTextField, trackYTextField, trackZTextField;
    private TextField dollyXTextField, dollyYTextField, dollyZTextField;

    private TextField fixXTextField, fixYTextField, fixZTextField;
    private TextField camXTextField, camYTextField, camZTextField;

    private double newTrackDx, newTrackDy, newTrackDz, newDollyDx, newDollyDy, newDollyDz;
    private double newFixX, newFixY, newFixZ, newCamX, newCamY, newCamZ;

    private Button trackCurrentButton, dollyCurrentButton;


    // private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

    private Button okButton, applyButton, cancelButton;
    private CameraPropertiesPanel cameraPropertiesPanel;
    private Container parentContainer;
    @SuppressWarnings("unused")
    private JFrame parentFrame;

    // private CameraPropertiesDialogListener listener;
    private ActiveCameraHolder cameraHolder;
    private JCheckBox GUITrackCheckBox, GUIDollyCheckBox;

    public CameraPropertiesDialog(Container parentContainer, JFrame frame, JCheckBox GUITrackCheckBox, JCheckBox GUIDollyCheckBox,
                                  ActiveCameraHolder cameraHolder)    // CameraPropertiesDialogListener listener)
    {
        //super(frame, "Camera Properties", false);
        super();

        this.setTitle("Camera Properties");

        this.parentContainer = parentContainer;
        this.parentFrame = frame;

        this.cameraHolder = cameraHolder;

        // this.listener = listener;
        this.GUITrackCheckBox = GUITrackCheckBox;
        this.GUIDollyCheckBox = GUIDollyCheckBox;

        this.numFormat = NumberFormat.getInstance();
        this.numFormat.setMaximumFractionDigits(4);
        this.numFormat.setMinimumFractionDigits(1);
        this.numFormat.setGroupingUsed(false);

        GridPane main = new GridPane();

        cameraPropertiesPanel = new CameraPropertiesPanel();
        GridPane.setConstraints(cameraPropertiesPanel, 0, 0);
        GridPane.setMargin(cameraPropertiesPanel, new Insets(10,5,0,5));

        // Bottom Buttons:

        okButton = new Button("OK");
        okButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);

        applyButton = new Button("Apply");
        applyButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        HBox.setMargin(applyButton, new Insets(0, 5, 0, 5));

        cancelButton = new Button("Cancel");
        cancelButton.addEventHandler(javafx.event.ActionEvent.ACTION, this);

        HBox buttons = new HBox();
        buttons.setAlignment(Pos.CENTER);
        buttons.getChildren().addAll(
                okButton,
                applyButton,
                cancelButton
        );
        GridPane.setConstraints(buttons, 0, 1);
        GridPane.setMargin(buttons, new Insets(10,0,10,0));

        main.getChildren().addAll(
                cameraPropertiesPanel,
                buttons
        );

        // Enlarge Text Fields:

        GUITrackCheckBox.setEnabled(false);
        GUIDollyCheckBox.setEnabled(false);

        parentContainer.repaint();    // This is a horrible way to get the graphs to repaint...

        Dimension frameSize = parentContainer.getSize();

        this.setScene(new Scene(main));
        this.setX(frameSize.width / 2);
        this.setY(frameSize.height / 4);
        this.setResizable(false);
        this.show();
    }

    @Override
    public void handle(javafx.event.ActionEvent event) {
        if (event.getSource() == cancelButton) {
            this.hide();
            GUITrackCheckBox.setEnabled(true);
            GUIDollyCheckBox.setEnabled(true);
        } else if (event.getSource() == applyButton) {
            cameraPropertiesPanel.commitChanges();
        } else if (event.getSource() == okButton) {
            cameraPropertiesPanel.commitChanges();
            this.hide();
            GUITrackCheckBox.setEnabled(true);
            GUIDollyCheckBox.setEnabled(true);
        }

        parentContainer.repaint();    // This is a horrible way to get the graphs to repaint...
    }

    @SuppressWarnings("serial")
    public class CameraPropertiesPanel extends GridPane implements EventHandler<javafx.event.ActionEvent> {
        public CameraPropertiesPanel() {
            super();

            ColumnConstraints firstLabels = new ColumnConstraints();
            firstLabels.setPercentWidth(5.00);
            ColumnConstraints firstTextFields = new ColumnConstraints();
            firstTextFields.setPercentWidth(30.00);
            ColumnConstraints checkboxes = new ColumnConstraints();
            checkboxes.setPercentWidth(16.00);
            ColumnConstraints secondLabels = new ColumnConstraints();
            secondLabels.setPercentWidth(19.00);
            ColumnConstraints secondTextFields = new ColumnConstraints();
            secondTextFields.setPercentWidth(30.00);

            this.getColumnConstraints().addAll(
                    firstLabels,
                    firstTextFields,
                    checkboxes,
                    secondLabels,
                    secondTextFields
            );

            RowConstraints row = new RowConstraints();
            row.setPercentHeight(12.00);
            RowConstraints space = new RowConstraints();
            space.setPercentHeight(4.00);

            this.getRowConstraints().addAll(
                row,
                row,
                row,
                row,
                space,
                row,
                row,
                row,
                row
            );

            CameraPropertiesHolder listener = cameraHolder.getCameraPropertiesForActiveCamera();

            newTrackDx = listener.getTrackingXOffset();
            newTrackDy = listener.getTrackingYOffset();
            newTrackDz = listener.getTrackingZOffset();

            newDollyDx = listener.getDollyXOffset();
            newDollyDy = listener.getDollyYOffset();
            newDollyDz = listener.getDollyZOffset();

            newFixX = listener.getFixX();
            newFixY = listener.getFixY();
            newFixZ = listener.getFixZ();

            newCamX = listener.getCamX();
            newCamY = listener.getCamY();
            newCamZ = listener.getCamZ();

            // Row 0:

            Label fixLabel = new Label("Fix Position:");
            GridPane.setConstraints(fixLabel, 1, 0);

            trackCheckBox = new CheckBox("Track");
            trackCheckBox.setSelected(listener.isTracking());
            trackCheckBox.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(trackCheckBox, 2, 0);

            trackCurrentButton = new TrackCurrentButton();
            GridPane.setConstraints(trackCurrentButton, 3, 0);

            // Row 1:

            Label fixXLabel = new Label("X:");
            GridPane.setConstraints(fixXLabel, 0, 1);

            String fixXVal = numFormat.format(newFixX);
            fixXTextField = new TextField(fixXVal);
            fixXTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(fixXTextField, 1, 1);

            trackXCheckBox = new CheckBox("Track X");
            trackXCheckBox.setSelected(listener.isTrackingX());
            trackXCheckBox.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(trackXCheckBox, 2, 1);

            Label trackDXLabel = new Label("X Offset:");
            GridPane.setConstraints(trackDXLabel, 3, 1);

            String trackDXVal = numFormat.format(newTrackDx);
            trackXTextField = new TextField(trackDXVal);
            trackXTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(trackXTextField, 4, 1);

            // Row 2:

            Label fixYLabel = new Label("Y:");
            GridPane.setConstraints(fixYLabel, 0, 2);

            String fixYVal = numFormat.format(newFixY);
            fixYTextField = new TextField(fixYVal);
            fixYTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(fixYTextField, 1, 2);

            trackYCheckBox = new CheckBox("Track Y");
            trackYCheckBox.setSelected(listener.isTrackingY());
            trackYCheckBox.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(trackYCheckBox, 2, 2);

            Label trackDYLabel = new Label("Y Offset:");
            GridPane.setConstraints(trackDYLabel, 3, 2);

            String trackDYVal = numFormat.format(newTrackDy);
            trackYTextField = new TextField(trackDYVal);
            trackYTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(trackYTextField, 4, 2);

            // Row 3:

            Label fixZLabel = new Label("Z:");
            GridPane.setConstraints(fixZLabel, 0, 3);

            String fixZVal = numFormat.format(newFixZ);
            fixZTextField = new TextField(fixZVal);
            fixZTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(fixZTextField, 1, 3);

            trackZCheckBox = new CheckBox("Track Z");
            trackZCheckBox.setSelected(listener.isTrackingZ());
            trackZCheckBox.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(trackZCheckBox, 2, 3);

            Label trackDZLabel = new Label("Z Offset:");
            GridPane.setConstraints(trackDZLabel, 3, 3);

            String trackDZVal = numFormat.format(newTrackDz);
            trackZTextField = new TextField(trackDZVal);
            trackZTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(trackZTextField, 4, 3);

            this.getChildren().addAll(
                    fixLabel,
                    fixXTextField,
                    trackCheckBox,
                    trackCurrentButton,
                    fixXLabel,
                    trackXCheckBox,
                    trackDXLabel,
                    fixYLabel,
                    fixYTextField,
                    trackYCheckBox,
                    trackDYLabel,
                    trackYTextField,
                    fixZLabel,
                    fixZTextField,
                    trackZCheckBox,
                    trackDZLabel,
                    trackZTextField
            );

            // Dolly:
            // Row 0:

            Label cameraLabel = new Label("Camera Position:");
            GridPane.setConstraints(cameraLabel, 1, 5);

            dollyCheckBox = new CheckBox("Dolly");
            dollyCheckBox.setSelected(listener.isDolly());
            dollyCheckBox.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(dollyCheckBox, 2, 5);

            dollyCurrentButton = new DollyCurrentButton();
            GridPane.setConstraints(dollyCurrentButton, 3, 5);

            // Row 1:

            Label camXLabel = new Label("X:");
            GridPane.setConstraints(camXLabel, 0, 6);

            String camXVal = numFormat.format(newCamX);
            camXTextField = new TextField(camXVal);
            camXTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(camXTextField, 1, 6);

            dollyXCheckBox = new CheckBox("Dolly X");
            dollyXCheckBox.setSelected(listener.isDollyX());
            dollyXCheckBox.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(dollyXCheckBox, 2, 6);

            Label dollyDXLabel = new Label("X Offset:");
            GridPane.setConstraints(dollyDXLabel, 3, 6);

            String dollyDXVal = numFormat.format(newDollyDx);
            dollyXTextField = new TextField(dollyDXVal);
            dollyXTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(dollyXTextField, 4, 6);

            // Row 2:

            Label camYLabel = new Label("Y:");
            GridPane.setConstraints(camYLabel, 0, 7);

            String camYVal = numFormat.format(newCamY);
            camYTextField = new TextField(camYVal);
            camYTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(camYTextField, 1, 7);

            dollyYCheckBox = new CheckBox("Dolly Y");
            dollyYCheckBox.setSelected(listener.isDollyY());
            dollyYCheckBox.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(dollyYCheckBox, 2, 7);

            Label dollyDYLabel = new Label("Y Offset:");
            GridPane.setConstraints(dollyDYLabel, 3, 7);

            String dollyDYVal = numFormat.format(newDollyDy);
            dollyYTextField = new TextField(dollyDYVal);
            dollyYTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(dollyYTextField, 4, 7);

            // Row 3:

            Label camZLabel = new Label("Z:");
            GridPane.setConstraints(camZLabel, 0, 8);

            String camZVal = numFormat.format(newCamZ);
            camZTextField = new TextField(camZVal);
            camZTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(camZTextField, 1, 8);

            dollyZCheckBox = new CheckBox("Dolly Z");
            dollyZCheckBox.setSelected(listener.isDollyZ());
            dollyZCheckBox.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(dollyZCheckBox, 2, 8);

            Label dollyDZLabel = new Label("Z Offset:");
            GridPane.setConstraints(dollyDZLabel, 3, 8);

            String dollyDZVal = numFormat.format(newDollyDz);
            dollyZTextField = new TextField(dollyDZVal);
            dollyZTextField.addEventHandler(javafx.event.ActionEvent.ACTION, this);
            GridPane.setConstraints(dollyZTextField, 4, 8);

            this.getChildren().addAll(
                    cameraLabel,
                    dollyCheckBox,
                    dollyCurrentButton,
                    camXLabel,
                    camXTextField,
                    dollyXCheckBox,
                    dollyDXLabel,
                    camYLabel,
                    camYTextField,
                    dollyYCheckBox,
                    dollyDYLabel,
                    dollyYTextField,
                    camZLabel,
                    camZTextField,
                    dollyZCheckBox,
                    dollyDZLabel,
                    dollyZTextField
            );

            this.updateTracking();
            this.updateEnabling();
        }

        public void commitChanges() {
            updateTracking();
            updateEnabling();

            CameraPropertiesHolder listener = cameraHolder.getCameraPropertiesForActiveCamera();

            listener.setFixX(newFixX);
            listener.setFixY(newFixY);
            listener.setFixZ(newFixZ);

            listener.setCamX(newCamX);
            listener.setCamY(newCamY);
            listener.setCamZ(newCamZ);

            listener.setTracking(trackCheckBox.isSelected());
            listener.setTrackingX(trackXCheckBox.isSelected());
            listener.setTrackingY(trackYCheckBox.isSelected());
            listener.setTrackingZ(trackZCheckBox.isSelected());

            listener.setDolly(dollyCheckBox.isSelected());
            listener.setDollyX(dollyXCheckBox.isSelected());
            listener.setDollyY(dollyYCheckBox.isSelected());
            listener.setDollyZ(dollyZCheckBox.isSelected());

            listener.setTrackingXOffset(newTrackDx);
            listener.setTrackingYOffset(newTrackDy);
            listener.setTrackingZOffset(newTrackDz);

            listener.setDollyXOffset(newDollyDx);
            listener.setDollyYOffset(newDollyDy);
            listener.setDollyZOffset(newDollyDz);

            listener.update();

            GUITrackCheckBox.setSelected(trackCheckBox.isSelected());
            GUIDollyCheckBox.setSelected(dollyCheckBox.isSelected());

            fixXTextField.setText(numFormat.format(listener.getFixX()));
            fixYTextField.setText(numFormat.format(listener.getFixY()));
            fixZTextField.setText(numFormat.format(listener.getFixZ()));

            camXTextField.setText(numFormat.format(listener.getCamX()));
            camYTextField.setText(numFormat.format(listener.getCamY()));
            camZTextField.setText(numFormat.format(listener.getCamZ()));

        }

        private void updateEnabling() {
            if (trackCheckBox.isSelected()) {
                trackXCheckBox.setDisable(false);
                trackYCheckBox.setDisable(false);
                trackZCheckBox.setDisable(false);
                if (trackXCheckBox.isSelected())
                    trackXTextField.setDisable(false);
                else
                    trackXTextField.setDisable(true);
                if (trackYCheckBox.isSelected())
                    trackYTextField.setDisable(false);
                else
                    trackYTextField.setDisable(true);
                if (trackZCheckBox.isSelected())
                    trackZTextField.setDisable(false);
                else
                    trackZTextField.setDisable(true);
            } else {
                trackXCheckBox.setDisable(true);
                trackYCheckBox.setDisable(true);
                trackZCheckBox.setDisable(true);
                trackXTextField.setDisable(true);
                trackYTextField.setDisable(true);
                trackZTextField.setDisable(true);
            }

            if (dollyCheckBox.isSelected()) {
                dollyXCheckBox.setDisable(false);
                dollyYCheckBox.setDisable(false);
                dollyZCheckBox.setDisable(false);
                if (dollyXCheckBox.isSelected())
                    dollyXTextField.setDisable(false);
                else
                    dollyXTextField.setDisable(true);
                if (dollyYCheckBox.isSelected())
                    dollyYTextField.setDisable(false);
                else
                    dollyYTextField.setDisable(true);
                if (dollyZCheckBox.isSelected())
                    dollyZTextField.setDisable(false);
                else
                    dollyZTextField.setDisable(true);
            } else {
                dollyXCheckBox.setDisable(true);
                dollyYCheckBox.setDisable(true);
                dollyZCheckBox.setDisable(true);
                dollyXTextField.setDisable(true);
                dollyYTextField.setDisable(true);
                dollyZTextField.setDisable(true);
            }
        }


        private void updateTracking() {
            updateTrackXOffset();
            updateTrackYOffset();
            updateTrackZOffset();

            updateDollyXOffset();
            updateDollyYOffset();
            updateDollyZOffset();

            updateFixX();
            updateFixY();
            updateFixZ();

            updateCamX();
            updateCamY();
            updateCamZ();
        }

        private void updateTrackXOffset() {
            newTrackDx = updateTrackDolly(trackXTextField, newTrackDx);
        }

        private void updateTrackYOffset() {
            newTrackDy = updateTrackDolly(trackYTextField, newTrackDy);
        }

        private void updateTrackZOffset() {
            newTrackDz = updateTrackDolly(trackZTextField, newTrackDz);
        }

        private void updateDollyXOffset() {
            newDollyDx = updateTrackDolly(dollyXTextField, newDollyDx);
        }

        private void updateDollyYOffset() {
            newDollyDy = updateTrackDolly(dollyYTextField, newDollyDy);
        }

        private void updateDollyZOffset() {
            newDollyDz = updateTrackDolly(dollyZTextField, newDollyDz);
        }

        private void updateFixX() {
            newFixX = updateTrackDolly(fixXTextField, newFixX);
        }

        private void updateFixY() {
            newFixY = updateTrackDolly(fixYTextField, newFixY);
        }

        private void updateFixZ() {
            newFixZ = updateTrackDolly(fixZTextField, newFixZ);
        }

        private void updateCamX() {
            newCamX = updateTrackDolly(camXTextField, newCamX);
        }

        private void updateCamY() {
            newCamY = updateTrackDolly(camYTextField, newCamY);
        }

        private void updateCamZ() {
            newCamZ = updateTrackDolly(camZTextField, newCamZ);
        }


        private double updateTrackDolly(TextField textField, double oldVal) {
            double newVal = oldVal;
            String text = textField.getText();

            try {
                newVal = Double.parseDouble(text);
            } catch (NumberFormatException e) {
                textField.setText(numFormat.format(newVal));
            }

            return newVal;
        }

        @Override
        public void handle(javafx.event.ActionEvent event) {
            updateTracking();
            updateEnabling();
        }
    }

    @SuppressWarnings("serial")
    public class TrackCurrentButton extends Button implements EventHandler<javafx.event.ActionEvent> {
        public TrackCurrentButton() {
            super("Track Current");
            this.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        }

        @Override
        public void handle(javafx.event.ActionEvent actionEvent) {
            CameraPropertiesHolder cameraPropertiesHolder = cameraHolder.getCameraPropertiesForActiveCamera();

            newTrackDx = -cameraPropertiesHolder.getTrackXVar() + newFixX;
            trackXTextField.setText(numFormat.format(newTrackDx));
            newTrackDy = -cameraPropertiesHolder.getTrackYVar() + newFixY;
            trackYTextField.setText(numFormat.format(newTrackDy));
            newTrackDz = -cameraPropertiesHolder.getTrackZVar() + newFixZ;
            trackZTextField.setText(numFormat.format(newTrackDz));
        }
    }


    public class DollyCurrentButton extends Button implements EventHandler<javafx.event.ActionEvent> {
        public DollyCurrentButton() {
            super("Dolly Current");
            this.addEventHandler(javafx.event.ActionEvent.ACTION, this);
        }

        @Override
        public void handle(javafx.event.ActionEvent actionEvent) {
            CameraPropertiesHolder cameraPropertiesHolder = cameraHolder.getCameraPropertiesForActiveCamera();

            newDollyDx = -cameraPropertiesHolder.getDollyXVar() + newCamX;
            dollyXTextField.setText(numFormat.format(newDollyDx));
            newDollyDy = -cameraPropertiesHolder.getDollyYVar() + newCamY;
            dollyYTextField.setText(numFormat.format(newDollyDy));
            newDollyDz = -cameraPropertiesHolder.getDollyZVar() + newCamZ;
            dollyZTextField.setText(numFormat.format(newDollyDz));
        }
    }
}
