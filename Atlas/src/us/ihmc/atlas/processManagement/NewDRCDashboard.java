package us.ihmc.atlas.processManagement;

import java.awt.Dimension;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.Insets;

import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.darpaRoboticsChallenge.processManagement.DragAndDropTreePanel;
import us.ihmc.darpaRoboticsChallenge.processManagement.GazeboRemoteSimulationAdapter;
import us.ihmc.darpaRoboticsChallenge.processManagement.ImagePanel;
import us.ihmc.darpaRoboticsChallenge.processManagement.LaunchSCSAndUIPanel;
import us.ihmc.darpaRoboticsChallenge.processManagement.LocalCloudListPanel;

public class NewDRCDashboard
{
   private JFrame frame;
   private JPanel mainPanel, dropListPanel, taskLaunchPanel;
      
   private final GazeboRemoteSimulationAdapter sshSimLauncher = new GazeboRemoteSimulationAdapter();
   private TaskSelectionPanel taskSelectionPanel;
   private LocalCloudListPanel cloudListPanel;
   private LaunchSCSAndUIPanel launchPanel;
   private ImagePanel ihmcLogo;
   
   private final GridBagConstraints c;


   public NewDRCDashboard()
   {
      frame = new JFrame("DRC Dashboard v2.0");
      mainPanel = new JPanel(new GridBagLayout());
      
      c = new GridBagConstraints();
      
      c.insets = new Insets(5, 5, 5, 5);
      
      setupLeftPanel();
      setupCenterPanel();
      setupRightPanel();
      
      frame.add(mainPanel);
      
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setMinimumSize(new Dimension(820, 460));
//      frame.setResizable(false);
      frame.setVisible(true);
   }
   
   private void setupLeftPanel()
   {
      cloudListPanel = new LocalCloudListPanel(sshSimLauncher);
      
      c.gridx = 0;
      c.gridy = 0;
      c.weightx = 0;
      c.fill = GridBagConstraints.VERTICAL;
      mainPanel.add(cloudListPanel, c);
   }
   
   private void setupCenterPanel()
   {
      dropListPanel = new JPanel(new GridBagLayout());
      
      GridBagConstraints c2 = new GridBagConstraints();
      c2.gridx = 0;
      c2.gridy = 0;
      dropListPanel.add(new DragAndDropTreePanel("Controller", sshSimLauncher), c2);      
      c2.gridy++;
      dropListPanel.add(new DragAndDropTreePanel("Network Processor", sshSimLauncher), c2);
      c2.gridy++;
      dropListPanel.add(new DragAndDropTreePanel("Gazebo", sshSimLauncher), c2);
      
      c.gridx++;
      mainPanel.add(dropListPanel, c);
   }
   
   private void setupRightPanel()
   {
      taskLaunchPanel = new JPanel(new GridBagLayout());
      
      taskSelectionPanel = new TaskSelectionPanel();
      launchPanel = new LaunchSCSAndUIPanel();
      ihmcLogo = new ImagePanel("ihmcRoboticsBlue_cropped.png", 300, 90);
      
      GridBagConstraints c2 = new GridBagConstraints();
      c2.fill = GridBagConstraints.HORIZONTAL;
      c2.gridx = 0;
      c2.gridy = 0;
      taskLaunchPanel.add(taskSelectionPanel, c2);
      
      c2.gridy++;
      taskLaunchPanel.add(launchPanel, c2);
      
      c2.gridy++;
      c2.fill = GridBagConstraints.BOTH;
      c2.insets = new Insets(0, 0, 0, 0);
      c2.ipadx = 0;
      c2.ipady = 0;
      taskLaunchPanel.add(ihmcLogo, c2);
      
      c.weighty = 1.0;
      c.gridx++;
      mainPanel.add(taskLaunchPanel, c);
   }

   public static void main(String[] args)
   {
      new NewDRCDashboard();
   }

}