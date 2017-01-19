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

import us.ihmc.graphicsDescription.dataBuffer.DataEntry;
import us.ihmc.simulationconstructionset.gui.GraphPropertiesPanel;
import us.ihmc.simulationconstructionset.gui.VarPropertiesPanel;
import us.ihmc.simulationconstructionset.gui.YoGraph;


public class GraphPropertiesDialog extends JDialog implements ActionListener
{
   // private final static java.text.NumberFormat numFormat = new java.text.DecimalFormat(" 0.00000;-0.00000");

   private static final long serialVersionUID = 723781387756094595L;
   private final JButton okButton, applyButton, cancelButton;
   private final GraphPropertiesPanel graphPropertiesPanel;
   private final VarPropertiesPanel[] varPropertiesPanels;
   private final JFrame parentFrame;
   private final YoGraph graph;
   
   public GraphPropertiesDialog(JFrame frame, YoGraph graph)  
   {
      super(frame, "Graph Properties", false);
      this.parentFrame = frame;
      this.graph = graph;

      Container contentPane = this.getContentPane();

      ArrayList<DataEntry> entries = graph.getEntriesOnThisGraph();

      JPanel panels = new JPanel(new GridLayout(entries.size() + 1, 1));
      graphPropertiesPanel = new GraphPropertiesPanel(graph);
      panels.add(graphPropertiesPanel);

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

//      parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
   }

   public void actionPerformed(ActionEvent event)
   {
      if (event.getSource() == cancelButton)
         this.setVisible(false);

      if (event.getSource() == applyButton)
      {
         graphPropertiesPanel.commitChanges();

         for (int i = 0; i < varPropertiesPanels.length; i++)
         {
            varPropertiesPanels[i].commitChanges();
         }
         
         graph.repaint();
      }

      if (event.getSource() == okButton)
      {
         graphPropertiesPanel.commitChanges();

         for (int i = 0; i < varPropertiesPanels.length; i++)
         {
            varPropertiesPanels[i].commitChanges();
         }

         graph.repaint();
         this.setVisible(false);
      }

      parentFrame.repaint();    // This is a horrible way to get the graphs to repaint...
   }



}
