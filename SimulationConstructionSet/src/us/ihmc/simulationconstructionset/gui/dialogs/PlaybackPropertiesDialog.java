package us.ihmc.simulationconstructionset.gui.dialogs;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.Border;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;


public class PlaybackPropertiesDialog extends JDialog implements ActionListener
{
   private static final long serialVersionUID = -8226475433536336684L;
   private JButton okButton, applyButton, cancelButton;
   private PlaybackPropertiesPanel playbackPropertiesPanel;
   @SuppressWarnings("unused")
   private JFrame ownerFrame;
   private Container parentContainer;

   private SimulationConstructionSet sim;

   public PlaybackPropertiesDialog(Container parentContainer, JFrame ownerFrame, SimulationConstructionSet sim)
   {
      super(ownerFrame, "Playback Properties", false);
      this.parentContainer = parentContainer;
      this.ownerFrame = ownerFrame;
      this.sim = sim;

      Container contentPane = this.getContentPane();
      playbackPropertiesPanel = new PlaybackPropertiesPanel();

      contentPane.add(playbackPropertiesPanel);

      // Buttons:

      okButton = new JButton("OK");
      okButton.addActionListener(this);
      applyButton = new JButton("Apply");
      applyButton.addActionListener(this);
      cancelButton = new JButton("Cancel");
      cancelButton.addActionListener(this);
      JPanel buttonPanel = new JPanel();
      buttonPanel.add(okButton);
      buttonPanel.add(applyButton);
      buttonPanel.add(cancelButton);

      contentPane.add(buttonPanel, BorderLayout.SOUTH);

      Point point = parentContainer.getLocation();
      Dimension frameSize = parentContainer.getSize();

      point.translate(frameSize.width / 2, frameSize.height / 4);
      this.setLocation(point);

      this.setResizable(false);
      this.pack();

      this.pack();
      this.setVisible(true);

      parentContainer.repaint();    // This is a horrible way to get the graphs to repaint...
   }

   @Override
   public void actionPerformed(ActionEvent event)
   {
      if (event.getSource() == cancelButton)
         this.setVisible(false);

      if (event.getSource() == applyButton)
      {
         playbackPropertiesPanel.commitChanges();
      }

      if (event.getSource() == okButton)
      {
         playbackPropertiesPanel.commitChanges();
         this.setVisible(false);
      }

      parentContainer.repaint();    // This is a horrible way to get the graphs to repaint...
   }

   public class PlaybackPropertiesPanel extends JPanel implements ActionListener
   {
      private static final long serialVersionUID = -6809250560000939671L;

      private double newRealTimeVal, newFrameRateVal, newSimulateDurationVal;

      private GridBagLayout gridBagLayout1 = new GridBagLayout();
      private JCheckBox updateGraphsDuringPlaybackCheckbox = new JCheckBox();
      private JLabel realTimeRateLabel = new JLabel();
      private JLabel desiredFrameRateLabel = new JLabel();
      private JLabel simulateDurationLabel = new JLabel();
      private JTextField realTimeTextField = new JTextField();
      private JTextField frameRateTextField = new JTextField();
      private JTextField simulateDurationTextField = new JTextField();
      private JCheckBox simulateNoFasterThanRealTimeCheckbox = new JCheckBox();

      public PlaybackPropertiesPanel()
      {
         setLayout(gridBagLayout1);
         updateGraphsDuringPlaybackCheckbox.setText("Update Graphs");
         realTimeRateLabel.setText("Real Time Rate:");
         desiredFrameRateLabel.setText("Desired Frame Rate:");
         simulateDurationLabel.setText("Simulate Duration:");
         realTimeTextField.setMinimumSize(new Dimension(60, 21));
         realTimeTextField.setPreferredSize(new Dimension(60, 21));
         frameRateTextField.setMinimumSize(new Dimension(60, 21));
         frameRateTextField.setPreferredSize(new Dimension(60, 21));
         simulateDurationTextField.setMinimumSize(new Dimension(60, 21));
         simulateDurationTextField.setPreferredSize(new Dimension(60, 21));
         simulateNoFasterThanRealTimeCheckbox.setText("Simulate No Faster Than Real Time");
         
         add(updateGraphsDuringPlaybackCheckbox,
               new GridBagConstraints(0, 0, 1, 1, 0.0, 0.0, GridBagConstraints.EAST, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));
         add(realTimeRateLabel, new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0, GridBagConstraints.EAST, GridBagConstraints.NONE, new Insets(0, 6, 0, 0), 6, 0));
         add(desiredFrameRateLabel,
             new GridBagConstraints(0, 2, 1, 1, 0.0, 0.0, GridBagConstraints.EAST, GridBagConstraints.NONE, new Insets(0, 6, 0, 0), 6, 0));
         add(simulateDurationLabel,
               new GridBagConstraints(0, 3, 1, 1, 0.0, 0.0, GridBagConstraints.EAST, GridBagConstraints.NONE, new Insets(0, 6, 0, 0), 6, 0));
         add(realTimeTextField, new GridBagConstraints(1, 1, 2, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));
         add(frameRateTextField,
             new GridBagConstraints(2, 2, 1, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));
         add(simulateDurationTextField,
               new GridBagConstraints(2, 3, 1, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));
         add(simulateNoFasterThanRealTimeCheckbox,
               new GridBagConstraints(0, 4, 1, 1, 0.0, 0.0, GridBagConstraints.EAST, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));

         newRealTimeVal = sim.getPlaybackRealTimeRate();
         newFrameRateVal = sim.getPlaybackFrameRate();
         newSimulateDurationVal = sim.getSimulateDuration();

         updateGraphsDuringPlaybackCheckbox.setSelected(sim.areGraphsUpdatedDuringPlayback());
         realTimeTextField.setText(String.valueOf(newRealTimeVal));
         frameRateTextField.setText(String.valueOf(newFrameRateVal));
         simulateDurationTextField.setText(String.valueOf(newSimulateDurationVal));
         simulateNoFasterThanRealTimeCheckbox.setSelected(sim.getSimulateNoFasterThanRealTime());

         // sim.setUpdateGraphsDuringPlayback(updateGraphsCheckbox.isSelected());


         Border blackLine = BorderFactory.createLineBorder(Color.black);

         // TitledBorder title = BorderFactory.createTitledBorder(blackLine,selectedVariable.getName());
         // this.setBorder(title);
         this.setBorder(blackLine);
      }

      public void commitChanges()
      {
         updateRealTimeTextField();
         updateFrameRateTextField();
         updateSimulateDurationTextField();

         sim.setSimulateNoFasterThanRealTime(simulateNoFasterThanRealTimeCheckbox.isSelected());  
         sim.setPlaybackRealTimeRate(newRealTimeVal);
         sim.setPlaybackDesiredFrameRate(newFrameRateVal);
         sim.setSimulateDuration(newSimulateDurationVal);
         sim.setGraphsUpdatedDuringPlayback(updateGraphsDuringPlaybackCheckbox.isSelected());

         /*
          * dataBuffer.setMaxBufferSize(newMaxVal);
          * dataBuffer.changeBufferSize(newCurrentVal);
          *
          * dataBuffer.setWrapBuffer(wrapButton.isSelected());
          */
      }

      @Override
      public void actionPerformed(ActionEvent event)
      {
         if (event.getSource() == realTimeTextField)
            updateRealTimeTextField();
         if (event.getSource() == frameRateTextField)
            updateFrameRateTextField();
         if (event.getSource() == simulateDurationTextField)
            updateSimulateDurationTextField();
      }

      private void updateRealTimeTextField()
      {
         String text = realTimeTextField.getText();

         try
         {
            double val = Double.parseDouble(text);
            newRealTimeVal = val;
         }
         catch (NumberFormatException e)
         {
            realTimeTextField.setText(String.valueOf(newRealTimeVal));
         }

      }

      private void updateFrameRateTextField()
      {
         String text = frameRateTextField.getText();

         try
         {
            double val = Double.parseDouble(text);
            newFrameRateVal = val;
         }
         catch (NumberFormatException e)
         {
            frameRateTextField.setText(String.valueOf(newFrameRateVal));
         }
      }
      
      private void updateSimulateDurationTextField()
      {
         String text = simulateDurationTextField.getText();

         try
         {
            double val = Double.parseDouble(text);
            newSimulateDurationVal = val;
         }
         catch (NumberFormatException e)
         {
            simulateDurationTextField.setText(String.valueOf(newSimulateDurationVal));
         }
      }

   }
}
