package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;

import javax.swing.JFrame;
import javax.swing.JPanel;

public class NewDRCDashboard
{
   private JFrame frame;
   private JPanel panel;
   
   private GridBagConstraints c;

   public NewDRCDashboard()
   {
      frame = new JFrame("Drag and Drop");
      panel = new JPanel(new GridBagLayout());
      
      c = new GridBagConstraints();
      
      c.insets = new Insets(5, 5, 5, 5);
      
      TaskSelectionPanel taskSelectionPanel = new TaskSelectionPanel();
      LocalCloudListPanel cloudListPanel = new LocalCloudListPanel(new GazeboRemoteSimulationAdapter());
      JPanel dropListPanel = new JPanel(new GridBagLayout());
      
      c.gridx = 0;
      c.gridy = 0;
      dropListPanel.add(new DragAndDropListPanel("Empty Panel:"), c);      
      c.gridy++;
      dropListPanel.add(new DragAndDropListPanel("Empty Panel:"), c);
      c.gridy++;
      dropListPanel.add(new DragAndDropListPanel("Empty Panel:"), c);
      
      c.gridx = 0;
      c.gridy = 0;
      panel.add(taskSelectionPanel, c);
      
      c.gridy = 1;
      panel.add(cloudListPanel, c);
      
      c.gridx = 1;
      panel.add(dropListPanel, c);
      
      frame.add(panel);
      
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setMinimumSize(new Dimension(600, 450));
      frame.setVisible(true);
   }

   public static void main(String[] args)
   {
      new NewDRCDashboard();
   }

}