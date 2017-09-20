package us.ihmc.simulationconstructionset.gui.dialogs;

import javafx.event.ActionEvent;
import javafx.event.EventHandler;
import javafx.geometry.Insets;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.layout.*;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.yoVariables.dataBuffer.DataBufferEntry;
import us.ihmc.simulationconstructionset.gui.VarPropertiesPanel;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

public class VarPropertiesDialog extends Stage implements EventHandler<javafx.event.ActionEvent> {
    private Button okButton, applyButton, cancelButton;
    private VarPropertiesPanel[] varPropertiesPanels;
    private JFrame parentFrame;

    public VarPropertiesDialog(JFrame frame, ArrayList<DataBufferEntry> entries) {
        super();

        this.setTitle("Variable Properties");

        this.parentFrame = frame;

        GridPane main = new GridPane();
        main.setPadding(new Insets(10, 10, 10, 10));

        varPropertiesPanels = new VarPropertiesPanel[entries.size()];
        for (int i = 0; i < entries.size(); i++) {
            varPropertiesPanels[i] = new VarPropertiesPanel(entries.get(i));
            varPropertiesPanels[i].setBorder(new Border(new BorderStroke(Color.BLACK, BorderStrokeStyle.SOLID, new CornerRadii(5), BorderWidths.DEFAULT)));
            varPropertiesPanels[i].setPadding(new Insets(10, 10, 10, 10));
            GridPane.setConstraints(varPropertiesPanels[i], 0, i);
            GridPane.setMargin(varPropertiesPanels[i], new Insets(0, 0, 5, 0));
            main.getChildren().add(varPropertiesPanels[i]);
        }

        // Buttons:

        okButton = new Button("OK");
        okButton.addEventHandler(ActionEvent.ACTION, this);

        applyButton = new Button("Apply");
        applyButton.addEventHandler(ActionEvent.ACTION, this);
        GridPane.setMargin(applyButton, new Insets(0, 5, 0, 5));

        cancelButton = new Button("Cancel");
        cancelButton.addEventHandler(ActionEvent.ACTION, this);

        HBox buttons = new HBox();
        buttons.setAlignment(Pos.CENTER);
        buttons.getChildren().addAll(
                okButton,
                applyButton,
                cancelButton
        );
        GridPane.setConstraints(buttons, 0, entries.size());
        GridPane.setMargin(buttons, new Insets(5, 0, 0, 0));

        main.getChildren().add(buttons);

        Dimension frameSize = frame.getSize();

        this.setX(frameSize.width / 4);
        this.setY(frameSize.height / 2);
        this.setScene(new Scene(main));
        this.setResizable(false);
        this.show();

        parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
    }

    @Override
    public void handle(javafx.event.ActionEvent event) {
        if (event.getSource() == cancelButton) {
            this.hide();
        } else if (event.getSource() == applyButton) {
            for (VarPropertiesPanel varPropertiesPanel : varPropertiesPanels) {
                varPropertiesPanel.commitChanges();
            }
        } else if (event.getSource() == okButton) {
            for (VarPropertiesPanel varPropertiesPanel : varPropertiesPanels) {
                varPropertiesPanel.commitChanges();
            }

            this.hide();
        }

        parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
    }
}
