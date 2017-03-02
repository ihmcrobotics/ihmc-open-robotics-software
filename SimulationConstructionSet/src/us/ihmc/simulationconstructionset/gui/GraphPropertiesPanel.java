package us.ihmc.simulationconstructionset.gui;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JTextField;
import javax.swing.border.Border;
import javax.swing.border.TitledBorder;


@SuppressWarnings("serial")
public class GraphPropertiesPanel extends JPanel implements ActionListener
{
   private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

   private final JTextField minTextField, maxTextField, baseLineTextField;
   private final JRadioButton individualButton, autoButton, manualButton;

   private final JRadioButton timePlotButton, phasePlotButton, baseLineButton;
   private final JButton createFFTPlotButton;
   private final JButton createBodePlotButton;

   private final YoGraph graph;
   private double newMinVal, newMaxVal;
   private double[] newBaseVal;

   public GraphPropertiesPanel(YoGraph graph)
   {
      super();

      this.graph = graph;

      // this.selectedVariable = variable;
      // selectedVariable.reCalcMinMax();
      newMinVal = graph.getManualMinScaling();
      newMaxVal = graph.getManualMaxScaling();

      double[] baseLines = graph.getBaseLines();
      if ((baseLines != null) && (baseLines.length > 0))
         newBaseVal = baseLines;
      else
         newBaseVal = new double[] {0.0};

      GridBagLayout gridbag = new GridBagLayout();

      this.setLayout(gridbag);

      Border blackLine = BorderFactory.createLineBorder(Color.black);
      TitledBorder title = BorderFactory.createTitledBorder(blackLine, "Master Setting");
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

      individualButton = new JRadioButton("Individual", (graph.getScalingMethod() == YoGraph.INDIVIDUAL_SCALING));
      individualButton.addActionListener(this);
      constraints.gridx = 1;
      constraints.gridy = 0;
      constraints.gridwidth = 2;
      gridbag.setConstraints(individualButton, constraints);
      this.add(individualButton);

      autoButton = new JRadioButton("Auto", (graph.getScalingMethod() == YoGraph.AUTO_SCALING));
      autoButton.addActionListener(this);
      constraints.gridx = 3;
      constraints.gridy = 0;
      constraints.gridwidth = 2;
      gridbag.setConstraints(autoButton, constraints);
      this.add(autoButton);

      manualButton = new JRadioButton("Manual", (graph.getScalingMethod() == YoGraph.MANUAL_SCALING));
      manualButton.addActionListener(this);
      constraints.gridx = 5;
      constraints.gridy = 0;
      constraints.gridwidth = 1;
      constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(manualButton, constraints);
      this.add(manualButton);

      ButtonGroup group = new ButtonGroup();
      group.add(individualButton);
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
      minTextField.setEnabled(graph.getScalingMethod() == YoGraph.MANUAL_SCALING);
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
      maxTextField.setEnabled(graph.getScalingMethod() == YoGraph.MANUAL_SCALING);
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

      minValString = numFormat.format(graph.getMin());
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

      maxValString = numFormat.format(graph.getMax());
      JLabel maxTextLabel = new JLabel(maxValString);

      // jTextField.addActionListener(this);
      constraints.gridx = 5;
      constraints.gridy = 2;
      constraints.gridwidth = 2;
      gridbag.setConstraints(maxTextLabel, constraints);
      this.add(maxTextLabel);

      // Row 3
      JLabel typeLabel = new JLabel("Plot Type:  ");
      constraints.gridx = 0;
      constraints.gridy = 3;
      constraints.gridwidth = 1;
      constraints.anchor = GridBagConstraints.EAST;
      gridbag.setConstraints(typeLabel, constraints);
      this.add(typeLabel);

      timePlotButton = new JRadioButton("Time", (graph.getPlotType() == YoGraph.TIME_PLOT));
      timePlotButton.addActionListener(this);
      constraints.gridx = 1;
      constraints.gridy = 3;
      constraints.gridwidth = 2;
      gridbag.setConstraints(timePlotButton, constraints);
      this.add(timePlotButton);

      /*
       * scatterPlotButton = new JRadioButton("Scatter",(graph.getPlotType()==graph.SCATTER_PLOT));
       * scatterPlotButton.addActionListener(this);
       * constraints.gridx = 3; constraints.gridy = 3;
       * constraints.gridwidth = 2;
       * gridbag.setConstraints(scatterPlotButton,constraints);
       * this.add(scatterPlotButton);
       */

      phasePlotButton = new JRadioButton("Phase", (graph.getPlotType() == YoGraph.PHASE_PLOT));
      phasePlotButton.addActionListener(this);
      constraints.gridx = 3;
      constraints.gridy = 3;
      constraints.gridwidth = 2;
      constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(phasePlotButton, constraints);
      this.add(phasePlotButton);

      createFFTPlotButton = new JButton("FFT");
      createFFTPlotButton.addActionListener(this);
      constraints.gridx = 4;
      constraints.gridy = 3;
      constraints.gridwidth = 1;
      constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(createFFTPlotButton, constraints);
      this.add(createFFTPlotButton);

      createBodePlotButton = new JButton("Bode");
      createBodePlotButton.addActionListener(this);
      constraints.gridx = 5;
      constraints.gridy = 3;
      constraints.gridwidth = 1;
      constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(createBodePlotButton, constraints);
      this.add(createBodePlotButton);

      JLabel baseLineLabel = new JLabel("Base Line:  ");
      constraints.gridx = 0;
      constraints.gridy = 4;
      constraints.gridwidth = 1;
      constraints.anchor = GridBagConstraints.EAST;
      gridbag.setConstraints(baseLineLabel, constraints);
      this.add(baseLineLabel);


      String baseLineValString = "";

      for (double current : newBaseVal)
      {
         baseLineValString += numFormat.format(current) + ",";
      }

      baseLineTextField = new JTextField(baseLineValString);
      baseLineTextField.addActionListener(this);
      baseLineTextField.setEnabled(true);
      constraints.gridx = 1;
      constraints.gridy = 4;
      constraints.gridwidth = 2;
      constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(baseLineTextField, constraints);
      this.add(baseLineTextField);


      baseLineButton = new JRadioButton("Show Base Line", graph.getShowBaseLines());
      baseLineButton.setSelected(true);
      baseLineButton.addActionListener(this);
      constraints.gridx = 3;
      constraints.gridy = 4;
      constraints.gridwidth = 1;

      // constraints.anchor = GridBagConstraints.WEST;
      gridbag.setConstraints(baseLineButton, constraints);
      this.add(baseLineButton);

      ButtonGroup plotGroup = new ButtonGroup();
      plotGroup.add(timePlotButton);

      // plotGroup.add(scatterPlotButton);
      plotGroup.add(phasePlotButton);

   }

   public void commitChanges()
   {
      updateMinTextField();
      updateMaxTextField();
      updateBaseLineTextField();

      graph.setManualScaling(newMinVal, newMaxVal);

      if (this.individualButton.isSelected())
         graph.setScalingMethod(YoGraph.INDIVIDUAL_SCALING);
      else if (this.autoButton.isSelected())
         graph.setScalingMethod(YoGraph.AUTO_SCALING);
      else
         graph.setScalingMethod(YoGraph.MANUAL_SCALING);

      if (this.timePlotButton.isSelected())
         graph.setPlotType(YoGraph.TIME_PLOT);

      // if (this.scatterPlotButton.isSelected()) graph.setPlotType(graph.SCATTER_PLOT);
      if (this.phasePlotButton.isSelected())
         graph.setPlotType(YoGraph.PHASE_PLOT);

      graph.setShowBaseLines(baseLineButton.isSelected());
      graph.setBaseLines(newBaseVal);


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

      if (event.getSource() == this.individualButton)
      {
         maxTextField.setEnabled(false);
         minTextField.setEnabled(false);
      }

      if (event.getSource() == this.baseLineButton)
      {
         baseLineTextField.setEnabled(baseLineButton.isSelected());
      }

      if (event.getSource() == this.createFFTPlotButton)
      {
         graph.createFFTPlotsFromEntriesBetweenInOutPoints();
      }

      if (event.getSource() == this.createBodePlotButton)
      {
         graph.createBodePlotFromEntriesBetweenInOutPoints();
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

   public void updateBaseLineTextField()
   {
      String text = baseLineTextField.getText();

      try
      {
         newBaseVal = setBaseLines(text);

      }
      catch (NumberFormatException e)
      {
         String baseLineValString = "";

         for (double current : newBaseVal)
         {
            baseLineValString += numFormat.format(current) + ",";
         }

         baseLineTextField.setText(baseLineValString);
      }
   }

   public double[] setBaseLines(String baseLinesCommaSeperated)
   {
      String[] lines = baseLinesCommaSeperated.split(",");
      double[] values = new double[lines.length];
      for (int i = 0; i < lines.length; i++)
      {
//       try
//       {
         values[i] = Double.valueOf(lines[i]).doubleValue();

//       }
//       catch (NumberFormatException e)
//       {
//          values[i] = 0;
//       }
      }

      return values;
   }


}
