package us.ihmc.simulationconstructionset.gui.dialogs;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.GridLayout;
import java.awt.Point;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JDialog;
import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.simulationconstructionset.DataBufferEntry;
import us.ihmc.simulationconstructionset.gui.VarPropertiesPanel;

public class VarPropertiesDialog extends JDialog implements ActionListener
{
   private static final long serialVersionUID = 5673404867619146740L;
   private JButton okButton, applyButton, cancelButton;
   private VarPropertiesPanel[] varPropertiesPanels;
   private JFrame parentFrame;


   public VarPropertiesDialog(JFrame frame, ArrayList<DataBufferEntry> entries)
   {
      super(frame, "Variable Properties", false);
      this.parentFrame = frame;

      Container contentPane = this.getContentPane();

      JPanel panels = new JPanel(new GridLayout(entries.size(), 1));
      varPropertiesPanels = new VarPropertiesPanel[entries.size()];

      for (int i = 0; i < entries.size(); i++)
      {
         varPropertiesPanels[i] = new VarPropertiesPanel(entries.get(i));
         panels.add(varPropertiesPanels[i]);
      }

      contentPane.add(panels);

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


      Point point = frame.getLocation();
      Dimension frameSize = frame.getSize();

      point.translate(frameSize.width / 4, frameSize.height / 2);
      this.setLocation(point);

      this.setResizable(false);


      parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
   }

   @Override
   public void actionPerformed(ActionEvent event)
   {
      if (event.getSource() == cancelButton)
         this.setVisible(false);

      if (event.getSource() == applyButton)
      {
         for (int i = 0; i < varPropertiesPanels.length; i++)
         {
            varPropertiesPanels[i].commitChanges();
         }
      }

      if (event.getSource() == okButton)
      {
         for (int i = 0; i < varPropertiesPanels.length; i++)
         {
            varPropertiesPanels[i].commitChanges();
         }

         this.setVisible(false);
      }

      parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
   }



}
