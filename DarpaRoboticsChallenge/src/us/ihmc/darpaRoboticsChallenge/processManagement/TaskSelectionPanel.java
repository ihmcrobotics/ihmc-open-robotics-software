package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;

import javax.swing.BorderFactory;
import javax.swing.DefaultComboBoxModel;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;

import us.ihmc.darpaRoboticsChallenge.processManagement.DRCDashboardTypes.DRCROSTasks;

public class TaskSelectionPanel extends JPanel
{

   private static final long serialVersionUID = -8753653110398404717L;

   private JLabel taskLabel;
   private JComboBox taskCombo;
   
   private GridBagConstraints c;
   
   public TaskSelectionPanel()
   {
      this.setLayout(new GridBagLayout());
      this.setBorder(BorderFactory.createEtchedBorder());
      
      c = new GridBagConstraints();
      c.insets = new Insets(5, 5, 5, 5);
      
      c.fill = GridBagConstraints.HORIZONTAL;
      c.weightx = 1.0;

      c.gridx = 0;
      c.gridy = 0;
      c.gridwidth = 1;
      c.gridheight = 1;
      c.weighty = 0.3;
      taskLabel = new JLabel("Select DRC Task:", JLabel.CENTER);
      this.add(taskLabel, c);

      c.gridy = 1;

      DefaultComboBoxModel model = new DefaultComboBoxModel();
      
      for (DRCROSTasks task : DRCROSTasks.values())
      {
         model.addElement(task);
      }
      
      taskCombo = new JComboBox(model);
      this.add(taskCombo, c);
   }

   public static void main(String[] args)
   {
      JFrame frame = new JFrame("Task Selection");
      TaskSelectionPanel taskPanel = new TaskSelectionPanel();
      
      frame.add(taskPanel);
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setMinimumSize(new Dimension(200, 100));
      frame.setVisible(true);
   }
}
