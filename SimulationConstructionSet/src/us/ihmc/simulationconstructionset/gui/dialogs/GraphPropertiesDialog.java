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
import us.ihmc.yoVariables.dataBuffer.DataEntry;
import us.ihmc.simulationconstructionset.gui.GraphPropertiesPanel;
import us.ihmc.simulationconstructionset.gui.VarPropertiesPanel;
import us.ihmc.simulationconstructionset.gui.YoGraph;

import javax.swing.*;
import java.awt.*;
import java.util.ArrayList;

public class GraphPropertiesDialog extends Stage implements EventHandler<javafx.event.ActionEvent> {
    private final Button okButton, applyButton, cancelButton;
    private final GraphPropertiesPanel graphPropertiesPanel;
    private final VarPropertiesPanel[] varPropertiesPanels;
    private final JFrame parentFrame;
    private final YoGraph graph;

    public GraphPropertiesDialog(JFrame frame, YoGraph graph) {
        super();

        this.setTitle("Graph Properties");

        this.parentFrame = frame;
        this.graph = graph;

        GridPane main = new GridPane();
        main.setPadding(new Insets(10, 10, 10, 10));

        ArrayList<DataEntry> entries = graph.getEntriesOnThisGraph();

        graphPropertiesPanel = new GraphPropertiesPanel(graph);
        GridPane.setConstraints(graphPropertiesPanel, 0, 0);
        GridPane.setMargin(graphPropertiesPanel, new Insets(0, 0, 5, 0));
        main.getChildren().add(graphPropertiesPanel);

        varPropertiesPanels = new VarPropertiesPanel[entries.size()];

        for (int i = 0; i < entries.size(); i++) {
            varPropertiesPanels[i] = new VarPropertiesPanel(entries.get(i));
            varPropertiesPanels[i].setBorder(new Border(new BorderStroke(Color.BLACK, BorderStrokeStyle.SOLID, new CornerRadii(5), BorderWidths.DEFAULT)));
            varPropertiesPanels[i].setPadding(new Insets(10, 10, 10, 10));
            GridPane.setConstraints(varPropertiesPanels[i], 0, i + 1);
            GridPane.setMargin(varPropertiesPanels[i], new Insets(0, 0, 5, 0));
            main.getChildren().add(varPropertiesPanels[i]);
        }

        // Buttons:

        okButton = new Button("OK");
        okButton.addEventHandler(ActionEvent.ACTION, this);

        applyButton = new Button("Apply");
        applyButton.addEventHandler(ActionEvent.ACTION, this);
        HBox.setMargin(applyButton, new Insets(0, 5, 0, 5));

        cancelButton = new Button("Cancel");
        cancelButton.addEventHandler(ActionEvent.ACTION, this);

        HBox buttons = new HBox();
        buttons.setAlignment(Pos.CENTER);
        buttons.getChildren().addAll(
                okButton,
                applyButton,
                cancelButton
        );
        GridPane.setConstraints(buttons, 0, entries.size() + 1);
        GridPane.setMargin(buttons, new Insets(10, 0, 0, 0));

        main.getChildren().add(buttons);

        Dimension frameSize = frame.getSize();

        this.setScene(new Scene(main));
        this.setX(frameSize.width / 4);
        this.setY(frameSize.height / 2);
        this.setResizable(false);
        this.show();

        // parentFrame.repaint(); // TODO: This is a horrible way to get the graphs to repaint... and is it needed?
    }

    @Override
    public void handle(javafx.event.ActionEvent event) {
        if (event.getSource() == cancelButton) {
            this.hide();
        } else if (event.getSource() == applyButton) {
            graphPropertiesPanel.commitChanges();

            for (VarPropertiesPanel varPropertiesPanel : varPropertiesPanels) {
                varPropertiesPanel.commitChanges();
            }

            graph.repaint();
        } else if (event.getSource() == okButton) {
            graphPropertiesPanel.commitChanges();

            for (VarPropertiesPanel varPropertiesPanel : varPropertiesPanels) {
                varPropertiesPanel.commitChanges();
            }

            graph.repaint();
            this.hide();
        }

        parentFrame.repaint(); // TODO: This is a horrible way to get the graphs to repaint...
    }
}
