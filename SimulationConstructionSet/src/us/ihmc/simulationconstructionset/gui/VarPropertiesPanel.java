package us.ihmc.simulationconstructionset.gui;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.JCheckBox;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JTextField;
import javax.swing.border.Border;
import javax.swing.border.TitledBorder;

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;


public class VarPropertiesPanel extends JPanel implements ActionListener
{
   private static final long serialVersionUID = -1729688086769099783L;

   private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

   private JTextField minTextField, maxTextField;
   private JRadioButton autoButton, manualButton;
   private JCheckBox invertCheckBox;
   
   private final DataEntry entry;
   
   private double newMinVal, newMaxVal;

   public VarPropertiesPanel(DataEntry entry)
   {
      super();

      this.entry = entry;

//    entry.reCalcMinMax();
      newMinVal = entry.getManualMinScaling();
      newMaxVal = entry.getManualMaxScaling();
      GridBagLayout gridbag = new GridBagLayout();

      this.setLayout(gridbag);

      Border blackLine = BorderFactory.createLineBorder(Color.black);
      TitledBorder title = BorderFactory.createTitledBorder(blackLine, entry.getVariableName());
      this.setBorder(title);

      GridBagConstraints constraints = new GridBagConstraints();

      // Row 0:

      JLabel scalingLabel = new JLabel("Scaling:  ");
      constraints.gridx = 0;
      constraints.gridy = 0;
      constraints.gridwidth = 1;
      constraints.anchor = GridBagConstraints.EAST;
      gridbag.setConstraints(scalingLabel, constraints);
      this.add(scalingLabel);

      autoButton = new JRadioButton("Auto", entry.isAutoScaleEnabled());
      autoButton.addActionListener(this);
      constraints.gridx = 1;
      constraints.gridy = 0;
      constraints.gridwidth = 2;
      gridbag.setConstraints(autoButton, constraints);
      this.add(autoButton);

      manualButton = new JRadioButton("Manual", !entry.isAutoScaleEnabled());
      manualButton.addActionListener(this);
      constraints.gridx = 3;
      constraints.gridy = 0;
      constraints.gridwidth = 3;
      constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(manualButton, constraints);
      this.add(manualButton);

      ButtonGroup group = new ButtonGroup();
      group.add(autoButton);
      group.add(manualButton);

      // Row 1:

      JLabel settingsLabel = new JLabel("Manual Settings:  ");
      constraints.gridx = 0;
      constraints.gridy = 1;
      constraints.gridwidth = 1;
      gridbag.setConstraints(settingsLabel, constraints);
      this.add(settingsLabel);

      JLabel minSettingsLabel = new JLabel("  Min:  ");
      constraints.gridx = 1;
      constraints.gridy = 1;
      constraints.gridwidth = 1;
      gridbag.setConstraints(minSettingsLabel, constraints);
      this.add(minSettingsLabel);

      String minValString = numFormat.format(newMinVal);
      minTextField = new JTextField(minValString);
      minTextField.addActionListener(this);
      if (entry.isAutoScaleEnabled())
         minTextField.setEnabled(false);
      constraints.gridx = 2;
      constraints.gridy = 1;
      constraints.gridwidth = 2;
      constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(minTextField, constraints);
      this.add(minTextField);

      JLabel maxSettingsLabel = new JLabel("  Max:  ");
      constraints.gridx = 4;
      constraints.gridy = 1;
      constraints.gridwidth = 1;
      gridbag.setConstraints(maxSettingsLabel, constraints);
      this.add(maxSettingsLabel);


      String maxValString = numFormat.format(newMaxVal);
      maxTextField = new JTextField(maxValString);
      maxTextField.addActionListener(this);
      if (entry.isAutoScaleEnabled())
         maxTextField.setEnabled(false);
      constraints.gridx = 5;
      constraints.gridy = 1;
      constraints.gridwidth = 2;
      gridbag.setConstraints(maxTextField, constraints);
      this.add(maxTextField);


      // Row 2:

      JLabel rangeLabel = new JLabel("Data Range:  ");
      constraints.gridx = 0;
      constraints.gridy = 2;
      constraints.gridwidth = 1;
      constraints.anchor = GridBagConstraints.EAST;
      gridbag.setConstraints(rangeLabel, constraints);
      this.add(rangeLabel);

      JLabel minRangeLabel = new JLabel("  Min:  ");
      constraints.gridx = 1;
      constraints.gridy = 2;
      constraints.gridwidth = 1;
      gridbag.setConstraints(minRangeLabel, constraints);
      this.add(minRangeLabel);

      minValString = numFormat.format(entry.getMin());
      JLabel minTextLabel = new JLabel(minValString);
      constraints.gridx = 2;
      constraints.gridy = 2;
      constraints.gridwidth = 2;
      constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(minTextLabel, constraints);
      this.add(minTextLabel);

      JLabel maxRangeLabel = new JLabel("  Max:  ");
      constraints.gridx = 4;
      constraints.gridy = 2;
      constraints.gridwidth = 1;
      gridbag.setConstraints(maxRangeLabel, constraints);
      this.add(maxRangeLabel);


      maxValString = numFormat.format(entry.getMax());
      JLabel maxTextLabel = new JLabel(maxValString);

      // jTextField.addActionListener(this);
      constraints.gridx = 5;
      constraints.gridy = 2;
      constraints.gridwidth = 2;
      gridbag.setConstraints(maxTextLabel, constraints);
      this.add(maxTextLabel);
      
      
      invertCheckBox = new JCheckBox("Invert");
      invertCheckBox.setSelected(entry.getInverted());
      
      constraints.gridx = 0;
      constraints.gridy = 3;
      constraints.gridwidth = 1;
      constraints.anchor = GridBagConstraints.EAST;
      gridbag.setConstraints(invertCheckBox, constraints);
      this.add(invertCheckBox);

   }

   public void commitChanges()
   {
      updateMinTextField();
      updateMaxTextField();

      entry.setManualScaling(newMinVal, newMaxVal);

      if (this.autoButton.isSelected())
         entry.enableAutoScale(true);
      else
         entry.enableAutoScale(false);
      
  
      entry.setInverted(invertCheckBox.isSelected());      
   }

   @Override
   public void actionPerformed(ActionEvent event)
   {
      if (event.getSource() == maxTextField)
         updateMaxTextField();
      if (event.getSource() == minTextField)
         updateMinTextField();

      if (event.getSource() == autoButton)
      {
         maxTextField.setEnabled(false);
         minTextField.setEnabled(false);
      }

      if (event.getSource() == manualButton)
      {
         maxTextField.setEnabled(true);
         minTextField.setEnabled(true);
      }
   }

   public void updateMaxTextField()
   {
      String text = maxTextField.getText();

      try
      {
         double val = Double.valueOf(text).doubleValue();
         newMaxVal = val;
      }
      catch (NumberFormatException e)
      {
         maxTextField.setText(numFormat.format(newMaxVal));
      }

   }

   public void updateMinTextField()
   {
      String text = minTextField.getText();

      try
      {
         double val = Double.valueOf(text).doubleValue();
         newMinVal = val;
      }
      catch (NumberFormatException e)
      {
         minTextField.setText(numFormat.format(newMinVal));
      }
   }

}
