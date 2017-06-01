package us.ihmc.simulationconstructionset.gui.dialogs;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.*;
import javax.swing.border.Border;

import javafx.embed.swing.JFXPanel;
import javafx.event.Event;
import javafx.event.EventHandler;
import javafx.event.EventType;
import javafx.geometry.Pos;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.Dialog;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.Pane;
import javafx.stage.Modality;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.gui.DataBufferChangeListener;

public class DataBufferPropertiesDialog extends JDialog implements ActionListener, EventHandler {
    private static final long serialVersionUID = 3689240296926827447L;
    private JTextField maxTextField, currentTextField;
    private JRadioButton wrapButton, enlargeButton;

    private int newMaxVal, newCurrentVal;

    // private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

    private Button okButton, applyButton, cancelButton;
    private BufferPropertiesPanel bufferPropertiesPanel;

    private DataBufferChangeListener listener;
    private DataBuffer dataBuffer;


    public DataBufferPropertiesDialog(Container parentContainer, JFrame frame, DataBuffer dataBuffer, DataBufferChangeListener listener) {
        super(frame, "Data Buffer Properties", false);

        this.listener = listener;
        this.dataBuffer = dataBuffer;

        Container contentPane = this.getContentPane();
        bufferPropertiesPanel = new BufferPropertiesPanel();

        contentPane.add(bufferPropertiesPanel);

        // Buttons:

        okButton = new Button("OK");
        okButton.addEventHandler(EventType.ROOT, this);
        applyButton = new Button("Apply");
        applyButton.addEventHandler(EventType.ROOT, this);
        cancelButton = new Button("Cancel");
        cancelButton.addEventHandler(EventType.ROOT, this);
        JFXPanel buttonPanel = new JFXPanel();
        FlowPane pane = new FlowPane();
        pane.setAlignment(Pos.BOTTOM_CENTER);
        pane.getChildren().add(okButton);
        pane.getChildren().add(applyButton);
        pane.getChildren().add(cancelButton);
        Scene scene = new Scene(pane);
        buttonPanel.setScene(scene);
        buttonPanel.setPreferredSize(new Dimension((int)scene.getWidth(), (int)scene.getHeight()));

        contentPane.add(buttonPanel, BorderLayout.SOUTH);

        Point point = parentContainer.getLocation();
        Dimension frameSize = parentContainer.getSize();

        point.translate(frameSize.width / 2, frameSize.height / 4);
        this.setLocation(point);

        this.setResizable(false);
        this.pack();

        Dimension size = maxTextField.getSize();
        size.width = size.width * 5 / 4;
        maxTextField.setSize(size);
        maxTextField.setPreferredSize(size);
        maxTextField.setMinimumSize(size);

        size = currentTextField.getSize();
        size.width = size.width * 5 / 4;
        currentTextField.setSize(size);
        currentTextField.setPreferredSize(size);
        currentTextField.setMinimumSize(size);

        this.pack();
        this.setVisible(true);

        // parentFrame.repaint(); // This is a horrible way to get the graphs to repaint...
    }

    /**
     * <p>Handles Swing events from JButtons, as opposed to JavaFX's Buttons.</p>
     * <p>
     * <p>Immediately calls <code>doAction</code> using the <code>event</code>'s
     * source (<code>.getSource()</code>).</p>
     *
     * @param event the Swing event
     */
    @Override
    public void actionPerformed(ActionEvent event) {
        doAction(event.getSource());
    }

    /**
     * <p>Handles JavaFX events from Buttons, as opposed to Swing's JButtons.</p>
     * <p>
     * <p>Checks to make sure the event is a JavaFX ActionEvent, meaning that
     * the button was clicked and not merely entered/exited. If so, calls
     * <code>doAction</code> using the <code>event</code>'s source
     * (<code>.getSource()</code>).</p>
     *
     * @param event the JavaFX event
     */
    @Override
    public void handle(Event event) {
        if (event.getEventType().equals(javafx.event.ActionEvent.ACTION)) {
            doAction(event.getSource());
        }
    }

    /**
     * <p>Wrapper for Swing and JavaFX events. Takes a <code>source</code>
     * from either and performs the associated action.</p>
     *
     * @param source triggered the event handler from either Swing or JavaFX
     */
    private void doAction(Object source) {
        if (source == cancelButton) {
            this.setVisible(false);
        } else if (source == applyButton) {
            bufferPropertiesPanel.commitChanges();
        } else if (source == okButton) {
            bufferPropertiesPanel.commitChanges();
            this.setVisible(false);
        }

        // if (myGUI != null) myGUI.zoomFullView();
        if (listener != null) {
            listener.dataBufferChanged();
        }

        // parentFrame.repaint(); // This is a horrible way to get the graphs to repaint...
    }

    public class BufferPropertiesPanel extends JFXPanel implements ActionListener, EventHandler {
        public BufferPropertiesPanel() {
            super();

            newMaxVal = dataBuffer.getMaxBufferSize();
            newCurrentVal = dataBuffer.getBufferSize();
            GridBagLayout gridbag = new GridBagLayout();

            this.setLayout(gridbag);

            Border blackLine = BorderFactory.createLineBorder(Color.black);

            // TitledBorder title = BorderFactory.createTitledBorder(blackLine,selectedVariable.getName());
            // this.setBorder(title);
            this.setBorder(blackLine);

            GridBagConstraints constraints = new GridBagConstraints();

            // Row 0:

            JLabel policyLabel = new JLabel("   Filled Policy:  ");
            constraints.gridx = 0;
            constraints.gridy = 0;
            constraints.gridwidth = 1;
            constraints.anchor = GridBagConstraints.EAST;
            gridbag.setConstraints(policyLabel, constraints);
            this.add(policyLabel);

            wrapButton = new JRadioButton("Wrap", dataBuffer.getWrapBuffer());
            wrapButton.addActionListener(this);
            constraints.gridx = 1;
            constraints.gridy = 0;
            constraints.gridwidth = 1;
            constraints.anchor = GridBagConstraints.WEST;
            gridbag.setConstraints(wrapButton, constraints);
            this.add(wrapButton);

            enlargeButton = new JRadioButton("Enlarge", !dataBuffer.getWrapBuffer());
            enlargeButton.addActionListener(this);
            constraints.gridx = 2;
            constraints.gridy = 0;
            constraints.gridwidth = 1;
            constraints.anchor = GridBagConstraints.WEST;
            gridbag.setConstraints(enlargeButton, constraints);
            this.add(enlargeButton);

            ButtonGroup group = new ButtonGroup();
            group.add(wrapButton);
            group.add(enlargeButton);

            // Row 1:

            JLabel maxSettingsLabel = new JLabel("  Max Size:  ");
            constraints.gridx = 0;
            constraints.gridy = 1;
            constraints.gridwidth = 1;
            constraints.anchor = GridBagConstraints.EAST;
            gridbag.setConstraints(maxSettingsLabel, constraints);
            this.add(maxSettingsLabel);

            String maxValString = String.valueOf(newMaxVal);
            maxTextField = new JTextField(maxValString);
            maxTextField.addActionListener(this);
            if (dataBuffer.getWrapBuffer())
                maxTextField.setEnabled(false);
            constraints.gridx = 1;
            constraints.gridy = 1;
            constraints.gridwidth = 2;
            constraints.anchor = GridBagConstraints.WEST;
            gridbag.setConstraints(maxTextField, constraints);
            this.add(maxTextField);


            // Row 2:

            JLabel currentSettingsLabel = new JLabel("  Current Size:  ");
            constraints.gridx = 0;
            constraints.gridy = 2;
            constraints.gridwidth = 1;
            constraints.anchor = GridBagConstraints.EAST;
            gridbag.setConstraints(currentSettingsLabel, constraints);
            this.add(currentSettingsLabel);


            String currentValString = String.valueOf(newCurrentVal);
            currentTextField = new JTextField(currentValString);

            // currentTextField.setText(currentValString);
            // size = currentTextField.getSize();
            // size.width = size.width*2;
            // currentTextField.setSize(size);
            currentTextField.addActionListener(this);
            currentTextField.setEnabled(true);
            constraints.gridx = 1;
            constraints.gridy = 2;
            constraints.gridwidth = 2;
            constraints.anchor = GridBagConstraints.WEST;
            gridbag.setConstraints(currentTextField, constraints);
            this.add(currentTextField);

        }

        public void commitChanges() {
            updateMaxTextField();
            updateCurrentTextField();

            dataBuffer.setMaxBufferSize(newMaxVal);
            dataBuffer.changeBufferSize(newCurrentVal);

            dataBuffer.setWrapBuffer(wrapButton.isSelected());
        }

        @Override
        public void actionPerformed(ActionEvent event) {
            doAction(event.getSource());
        }

        @Override
        public void handle(Event event) {
            if (event.getEventType().equals(javafx.event.ActionEvent.ACTION)) {
                doAction(event.getSource());
            }
        }

        private void doAction(Object source) {
            if (source == maxTextField) {
                updateMaxTextField();
            } else if (source == currentTextField) {
                updateCurrentTextField();
            } else if (source == wrapButton) {
                maxTextField.setEnabled(false);
            } else if (source == enlargeButton) {
                maxTextField.setEnabled(true);
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
