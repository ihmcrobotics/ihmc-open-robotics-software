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
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;
import javax.swing.border.Border;

import us.ihmc.simulationconstructionset.SimulationConstructionSet;

public class YoGraphicsPropertiesDialog extends JDialog implements ActionListener
{
   private static final long serialVersionUID = -8226475433536336684L;
   private JButton okButton, applyButton, cancelButton;
   private YoGraphicsPropertiesPanel yoGraphicsPropertiesPanel;
   @SuppressWarnings("unused")
   private JFrame ownerFrame;
   private Container parentContainer;

   private SimulationConstructionSet sim;

   public YoGraphicsPropertiesDialog(Container parentContainer, JFrame ownerFrame, SimulationConstructionSet sim)
   {
      super(ownerFrame, "YoGraphics Properties", false);
      this.parentContainer = parentContainer;
      this.ownerFrame = ownerFrame;
      this.sim = sim;

      Container contentPane = this.getContentPane();
      yoGraphicsPropertiesPanel = new YoGraphicsPropertiesPanel();

      contentPane.add(yoGraphicsPropertiesPanel);

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

      parentContainer.repaint(); // This is a horrible way to get the graphs to repaint...
   }

   @Override
   public void actionPerformed(ActionEvent event)
   {
      if (event.getSource() == cancelButton)
         this.setVisible(false);

      if (event.getSource() == applyButton)
      {
         yoGraphicsPropertiesPanel.commitChanges();
      }

      if (event.getSource() == okButton)
      {
         yoGraphicsPropertiesPanel.commitChanges();
         this.setVisible(false);
      }

      parentContainer.repaint(); // This is a horrible way to get the graphs to repaint...
   }

   public class YoGraphicsPropertiesPanel extends JPanel implements ActionListener
   {
      private static final long serialVersionUID = -4338288396995414370L;

      private double newYoGraphicsGlobalScaleVal;

      private GridBagLayout gridBagLayout = new GridBagLayout();

      private JTextField yoGraphicsGlobalScaleTextField = new JTextField();

      private JLabel yoGraphicsGlobalScaleLabel = new JLabel();

      public YoGraphicsPropertiesPanel()
      {
         setLayout(gridBagLayout);
         yoGraphicsGlobalScaleLabel.setText("YoGraphics Global Scale:");

         yoGraphicsGlobalScaleTextField.setMinimumSize(new Dimension(60, 21));
         yoGraphicsGlobalScaleTextField.setPreferredSize(new Dimension(60, 21));

         add(yoGraphicsGlobalScaleLabel,
             new GridBagConstraints(0, 1, 1, 1, 0.0, 0.0, GridBagConstraints.EAST, GridBagConstraints.NONE, new Insets(0, 6, 0, 0), 6, 0));
         add(yoGraphicsGlobalScaleTextField,
             new GridBagConstraints(1, 1, 2, 1, 0.0, 0.0, GridBagConstraints.CENTER, GridBagConstraints.NONE, new Insets(0, 0, 0, 0), 0, 0));

         newYoGraphicsGlobalScaleVal = sim.getGlobalYoGraphicsScale();
         yoGraphicsGlobalScaleTextField.setText(String.valueOf(newYoGraphicsGlobalScaleVal));

         Border blackLine = BorderFactory.createLineBorder(Color.black);

         // TitledBorder title = BorderFactory.createTitledBorder(blackLine,selectedVariable.getName());
         // this.setBorder(title);
         this.setBorder(blackLine);
      }

      public void commitChanges()
      {
         updateYoGraphicsGlobalScaleTextField();

         sim.setYoGraphicsGlobalScale(newYoGraphicsGlobalScaleVal);
      }

      @Override
      public void actionPerformed(ActionEvent event)
      {
         if (event.getSource() == yoGraphicsGlobalScaleTextField)
            updateYoGraphicsGlobalScaleTextField();
      }

      private void updateYoGraphicsGlobalScaleTextField()
      {
         String text = yoGraphicsGlobalScaleTextField.getText();

         try
         {
            newYoGraphicsGlobalScaleVal = Double.parseDouble(text);

            if (newYoGraphicsGlobalScaleVal < 0.0)
            {
               newYoGraphicsGlobalScaleVal = Math.abs(newYoGraphicsGlobalScaleVal);
               yoGraphicsGlobalScaleTextField.setText(String.valueOf(newYoGraphicsGlobalScaleVal));
            }
         }
         catch (NumberFormatException e)
         {
            yoGraphicsGlobalScaleTextField.setText(String.valueOf(newYoGraphicsGlobalScaleVal));
         }
      }
   }
}
