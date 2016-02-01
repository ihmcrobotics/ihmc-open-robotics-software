package us.ihmc.atlas.processManagement;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;

import javax.swing.BorderFactory;
import javax.swing.JComboBox;
import javax.swing.JLabel;
import javax.swing.JPanel;

import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.darpaRoboticsChallenge.processManagement.DRCDashboardTypes.DRCROSTasks;

public class TaskSelectionPanel extends JPanel
{

   private static final long serialVersionUID = -8753653110398404717L;

   private JLabel taskLabel, robotModelLabel;
   private JComboBox taskCombo, robotModelCombo;
   
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
      taskLabel = new JLabel("Select DRC Task", JLabel.CENTER);
      this.add(taskLabel, c);

      c.gridy++;
      taskCombo = new JComboBox(DRCROSTasks.values());
      this.add(taskCombo, c);
      
      c.gridy++;
      robotModelLabel = new JLabel("Select Robot Model", JLabel.CENTER);
      this.add(robotModelLabel, c);
      
      c.gridy++;
      robotModelCombo = new JComboBox(AtlasRobotModelFactory.getAvailableRobotModels());
      this.add(robotModelCombo, c);
   }
}
