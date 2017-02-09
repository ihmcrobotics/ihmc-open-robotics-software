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

import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JTextField;
import javax.swing.border.Border;

import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.gui.DataBufferChangeListener;


public class DataBufferPropertiesDialog extends JDialog implements ActionListener
{
   private static final long serialVersionUID = 3689240296926827447L;
   private JTextField maxTextField, currentTextField;
   private JRadioButton wrapButton, enlargeButton;

   private int newMaxVal, newCurrentVal;

   // private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

   private JButton okButton, applyButton, cancelButton;
   private BufferPropertiesPanel bufferPropertiesPanel;

   private DataBufferChangeListener listener;
   private DataBuffer dataBuffer;


   public DataBufferPropertiesDialog(Container parentContainer, JFrame frame, DataBuffer dataBuffer, DataBufferChangeListener listener)
   {
      super(frame, "Data Buffer Properties", false);

      this.listener = listener;
      this.dataBuffer = dataBuffer;

      Container contentPane = this.getContentPane();
      bufferPropertiesPanel = new BufferPropertiesPanel();

      contentPane.add(bufferPropertiesPanel);

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

   @Override
   public void actionPerformed(ActionEvent event)
   {
      if (event.getSource() == cancelButton)
         this.setVisible(false);

      if (event.getSource() == applyButton)
      {
         bufferPropertiesPanel.commitChanges();
      }

      if (event.getSource() == okButton)
      {
         bufferPropertiesPanel.commitChanges();
         this.setVisible(false);
      }

      // if (myGUI != null) myGUI.zoomFullView();
      if (listener != null)
         listener.dataBufferChanged();

      // parentFrame.repaint(); // This is a horrible way to get the graphs to repaint...
   }

   public class BufferPropertiesPanel extends JPanel implements ActionListener
   {
      /**
       *
       */
      private static final long serialVersionUID = -3906496391997459515L;

      public BufferPropertiesPanel()
      {
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

      public void commitChanges()
      {
         updateMaxTextField();
         updateCurrentTextField();

         dataBuffer.setMaxBufferSize(newMaxVal);
         dataBuffer.changeBufferSize(newCurrentVal);

         dataBuffer.setWrapBuffer(wrapButton.isSelected());
      }

      @Override
      public void actionPerformed(ActionEvent event)
      {
         if (event.getSource() == maxTextField)
            updateMaxTextField();
         if (event.getSource() == currentTextField)
            updateCurrentTextField();

         if (event.getSource() == wrapButton)
         {
            maxTextField.setEnabled(false);
         }

         if (event.getSource() == enlargeButton)
         {
            maxTextField.setEnabled(true);
         }

      }

      public void updateMaxTextField()
      {
         String text = maxTextField.getText();

         try
         {
            int val = Integer.parseInt(text);
            newMaxVal = val;
         }
         catch (NumberFormatException e)
         {
            maxTextField.setText(String.valueOf(newMaxVal));
         }

      }

      public void updateCurrentTextField()
      {
         String text = currentTextField.getText();

         try
         {
            int val = Integer.parseInt(text);
            newCurrentVal = val;
         }
         catch (NumberFormatException e)
         {
            currentTextField.setText(String.valueOf(newCurrentVal));
         }
      }

   }


}
