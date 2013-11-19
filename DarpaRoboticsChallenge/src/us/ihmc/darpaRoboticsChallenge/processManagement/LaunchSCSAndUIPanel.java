package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.BorderFactory;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JPanel;

public class LaunchSCSAndUIPanel extends JPanel
{
   private static final long serialVersionUID = 9183065160326743265L;

   private GridBagConstraints c;
   
   private JCheckBox launchSCSCheckBox, launchUICheckBox;
   private JButton launchSCSButton, launchUIButton;
   
   private JCheckBox initializeEstimatorToActualCheckBox;
   
   public LaunchSCSAndUIPanel()
   {
      this.setLayout(new GridBagLayout());
      this.setBorder(BorderFactory.createEtchedBorder());
      
      c = new GridBagConstraints();
      c.insets = new Insets(5, 5, 5, 5);
      
      setupCheckBoxes();
      
      setupButtons();
   }
   
   private void setupCheckBoxes()
   {
      launchSCSCheckBox = new JCheckBox("Launch SCS with Gazebo");
      launchUICheckBox = new JCheckBox("Launch UI with Gazebo");
      initializeEstimatorToActualCheckBox = new JCheckBox("Initialize estimator to actual");
      initializeEstimatorToActualCheckBox.setEnabled(false);
      launchSCSCheckBox.addActionListener(new ActionListener()
      {

         @Override
         public void actionPerformed(ActionEvent arg0)
         {
            initializeEstimatorToActualCheckBox.setEnabled(launchSCSCheckBox.isSelected());            
         }
         
      });
      
      c.gridx = 0;
      c.gridy = 0;
      c.anchor = GridBagConstraints.WEST;
      this.add(launchUICheckBox, c);
      c.gridy++;
      this.add(launchSCSCheckBox, c);
      c.gridy++;
      this.add(initializeEstimatorToActualCheckBox, c);
   }
   
   private void setupButtons()
   {
      launchSCSButton = new JButton("Launch SCS");
      launchUIButton = new JButton("Launch UI");

      c.insets.set(0, 0, 0, 0);
      c.anchor = GridBagConstraints.CENTER;
      c.fill = GridBagConstraints.BOTH;
      c.weightx = 1.0;
      c.ipady = 20;
      c.gridy++;
      this.add(launchUIButton, c);
      c.gridy++;
      this.add(launchSCSButton, c);
   }
}
