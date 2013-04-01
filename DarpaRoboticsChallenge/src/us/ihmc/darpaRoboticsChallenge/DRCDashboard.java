package us.ihmc.darpaRoboticsChallenge;

import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.net.InetAddress;
import java.net.UnknownHostException;

import javax.swing.BorderFactory;
import javax.swing.DefaultListModel;
import javax.swing.JButton;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JTextField;
import javax.swing.ListModel;
import javax.swing.ListSelectionModel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import javax.swing.border.BevelBorder;

import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig.LocalCloudMachines;

public class DRCDashboard
{
   private static final boolean JREBEL = false;

   private static DRCDashboard instance;
   private String ROS_HOSTNAME;
   private String ROS_IP_ADDRESS;
   private String ROS_MASTER_URI;

   private GridBagConstraints c;

   private JFrame frame = new JFrame("IHMC DRC Dashboard");

   private JPanel taskPanel;
   private JLabel taskLabel;
   private JComboBox taskCombo;

   private JPanel mainContentPanel;

   private JPanel leftPanel;
   private JPanel cloudMachineSelectionPanel;
   private JLabel cloudMachineSelectionLabel;
   private JComboBox cloudMachineSelectionCombo;
   private JPanel cloudMachineInfoPanel;
   private JLabel cloudMachineIPAddressLabel;
   private JLabel cloudMachineHostnameLabel;
   private JPanel robotModelSelectionPanel;
   private JLabel robotModelSelectionLabel;
   private JComboBox robotModelSelectionCombo;

   private JPanel rightPanel;
   private JLabel gazeboProcessListLabel;
   private JList gazeboProcessList;
   private ListModel gazeboProcessListModel;

   private JScrollPane gazeboProcessListScroller;

   private JPanel networkInfoPanel;
   private JLabel hostnameLabel;
   private JLabel ipAddressLabel;
   private JLabel rosUriLabel;
   private JTextField rosCorePortField;
   private JButton modifyRosNetworkConfigsButton;
   private JButton launchGazeboSimButton;

   public DRCDashboard()
   {
      try
      {
         ROS_HOSTNAME = InetAddress.getLocalHost().getHostName().replace(" ", "-");
         ROS_IP_ADDRESS = InetAddress.getLocalHost().getHostAddress();
//         ROS_MASTER_URI = "http://";
      }
      catch (UnknownHostException e)
      {
         e.printStackTrace();
      }

      instance = this;

      if (JREBEL)
         startRedrawSignaler();
   }

   private void setupJFrame()
   {
      initializeLayout();

      initializeSelectTaskPanel();

      initializeMainContentPanel();

      initializeLeftContentPanel();

      initializeRightContentPanel();

      setupSelectTaskPanel();

      setupLeftContentPanel();

      setupRightContentPanel();
      
      updateRosMasterURI();

//            frame.setSize(760, 510);
      //      frame.setResizable(true);

      //      System.out.println(frame.getWidth());
      //      System.out.println(frame.getHeight());
   }

   private void initializeLayout()
   {
      c = new GridBagConstraints();
   
      frame.getContentPane().setLayout(new GridBagLayout());
      c.insets = new Insets(5, 5, 5, 5);
      c.fill = GridBagConstraints.BOTH;
      c.anchor = GridBagConstraints.PAGE_START;
      c.weighty = 1.0;
      c.weightx = 1.0;
   }

   private void initializeSelectTaskPanel()
   {
      c.gridx = 0;
      c.gridy = 0;
      c.weighty = 1.0;
      taskPanel = new JPanel(new GridBagLayout());
      taskPanel.setBorder(BorderFactory.createEtchedBorder());
      frame.getContentPane().add(taskPanel, c);
   }

   private void initializeMainContentPanel()
   {
      c.gridx = 0;
      c.gridy = 1;
      mainContentPanel = new JPanel(new GridBagLayout());
      frame.getContentPane().add(mainContentPanel, c);
   }

   private void initializeLeftContentPanel()
   {
      c.gridwidth = 4;
      c.gridheight = 10;
      c.gridx = 0;
      c.gridy = 0;
      c.weightx = 0.001;
      c.ipadx = 80;
      leftPanel = new JPanel(new GridBagLayout());
      leftPanel.setBorder(BorderFactory.createEtchedBorder());
      mainContentPanel.add(leftPanel, c);
   }

   private void initializeRightContentPanel()
   {
      c.gridwidth = 1;
      c.gridx = 4;
      c.weightx = 1.0;
      c.ipadx = 30;
      rightPanel = new JPanel(new GridBagLayout());
      rightPanel.setBorder(BorderFactory.createEtchedBorder());
      mainContentPanel.add(rightPanel, c);
   }

   private void setupSelectTaskPanel()
   {
      c.fill = GridBagConstraints.HORIZONTAL;
      c.weightx = 1.0;
   
      c.gridx = 0;
      c.gridy = 0;
      c.gridwidth = 1;
      c.gridheight = 1;
      c.weighty = 0.3;
      taskLabel = new JLabel("Select DRC Task:", JLabel.CENTER);
      taskPanel.add(taskLabel, c);
   
      c.gridy = 1;
//      taskCombo = new JComboBox(new String[] { "One", "Two", "Three", "Four" });
      taskCombo = new JComboBox(DRCDashboardTypes.DRCTask.values());
      taskPanel.add(taskCombo, c);
   }

   private void setupLeftContentPanel()
   {
      setupSelectRobotModel();
   
      setupSelectCloudMachine();
   
      setupCloudMachineInfoPanel();
   }

   private void setupSelectRobotModel()
   {
      c.gridx = 0;
      c.gridy = 0;
      c.gridwidth = 1;
      c.gridheight = 2;
      c.weighty = 0.0;
   
      robotModelSelectionPanel = new JPanel(new GridLayout(2, 1));
      leftPanel.add(robotModelSelectionPanel, c);
      robotModelSelectionLabel = new JLabel("Select Robot Model: ", JLabel.LEFT);
//      robotModelSelectionCombo = new JComboBox(new String[] { "One", "Two" });
      robotModelSelectionCombo = new JComboBox(DRCDashboardTypes.RobotModel.values());
      robotModelSelectionPanel.add(robotModelSelectionLabel);
      robotModelSelectionPanel.add(robotModelSelectionCombo);
   }

   private void setupSelectCloudMachine()
   {
      c.gridy = 2;
      cloudMachineSelectionPanel = new JPanel(new GridLayout(2, 1));
      leftPanel.add(cloudMachineSelectionPanel, c);
      cloudMachineSelectionLabel = new JLabel("Select Cloud Machine: ", JLabel.LEFT);
      cloudMachineSelectionPanel.add(cloudMachineSelectionLabel);
      cloudMachineSelectionCombo = new JComboBox(LocalCloudMachines.values());
      cloudMachineSelectionPanel.add(cloudMachineSelectionCombo);
      
      cloudMachineSelectionCombo.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {            
            cloudMachineHostnameLabel.setText(updatedCloudHostnameString());
            cloudMachineIPAddressLabel.setText(updatedCloudIpAddressString());
            updateRosMasterURI();
         }
      });
   }

   private void setupCloudMachineInfoPanel()
   {
      c.gridheight = 4;
      c.gridy = 4;
      c.ipady = 70;
//      c.ipadx = 150;
      c.weighty = 1.0;
      c.insets = new Insets(30, 15, 40, 65);
      c.fill = GridBagConstraints.BOTH;
      cloudMachineInfoPanel = new JPanel(new GridLayout(2, 1));
      cloudMachineInfoPanel.setBorder(BorderFactory.createBevelBorder(BevelBorder.LOWERED));
      ((GridLayout) cloudMachineInfoPanel.getLayout()).setVgap(-50);
      leftPanel.add(cloudMachineInfoPanel, c);
      cloudMachineHostnameLabel = new JLabel(updatedCloudHostnameString());
      cloudMachineIPAddressLabel = new JLabel(updatedCloudIpAddressString());
      cloudMachineInfoPanel.add(cloudMachineHostnameLabel);
      cloudMachineInfoPanel.add(cloudMachineIPAddressLabel);
      c.insets = new Insets(5, 5, 5, 5);
   }

   private void setupRightContentPanel()
   {
      setupGazeboLauncherList();
   
      setupNetworkInfoPanel();
   
      setupLaunchButton();
      
      setupEditInfoButton();
   }

   private void setupGazeboLauncherList()
   {
      c.gridwidth = 1;
      c.gridheight = 1;
      c.gridx = 0;
      c.gridy = 0;
      c.ipady = 0;
      c.fill = GridBagConstraints.NONE;
   
      c.ipadx = rightPanel.getWidth();
      gazeboProcessListLabel = new JLabel("Gazebo Processes To Launch:", JLabel.CENTER);
      rightPanel.add(gazeboProcessListLabel, c);
   
      c.gridy = 1;
      c.gridheight = 5;
      c.weighty = 100;
      c.ipady = 200;
      c.anchor = GridBagConstraints.PAGE_START;
      c.fill = GridBagConstraints.BOTH;
      c.insets = new Insets(10, 35, 10, 35);
      gazeboProcessListModel = new DefaultListModel();
      gazeboProcessList = new JList(gazeboProcessListModel);
      gazeboProcessList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
      gazeboProcessList.setLayoutOrientation(JList.VERTICAL);
      gazeboProcessListScroller = new JScrollPane(gazeboProcessList);
      rightPanel.add(gazeboProcessListScroller, c);
   }

   private void setupNetworkInfoPanel()
   {
      c.gridy = 6;
      c.gridheight = 4;
      c.weighty = 0.1;
      c.ipady = 0;
      c.insets = new Insets(5, 5, 5, 5);
      networkInfoPanel = new JPanel(new GridBagLayout());
      rightPanel.add(networkInfoPanel, c);
   
      c.gridx = 0;
      c.gridy = 0;
      c.gridwidth = 2;
      c.gridheight = 1;
      c.ipadx = 0;
      c.ipady = 0;
   
      hostnameLabel = new JLabel("ROS Hostname: " + ROS_HOSTNAME);
      networkInfoPanel.add(hostnameLabel, c);
   
      c.gridy = 1;
      ipAddressLabel = new JLabel("ROS IP Address: " + ROS_IP_ADDRESS);
      networkInfoPanel.add(ipAddressLabel, c);
   
      c.gridy = 2;
      rosUriLabel = new JLabel();
      networkInfoPanel.add(rosUriLabel, c);
   
      c.gridx = 2;
      c.fill = GridBagConstraints.HORIZONTAL;
      c.anchor = GridBagConstraints.LINE_START;
      rosCorePortField = new JTextField("" + DRCLocalCloudConfig.DEFAULT_ROSCORE_PORT, 5);
      networkInfoPanel.add(rosCorePortField, c);
   }

   private void setupLaunchButton()
   {
	   c.gridy = 3;
	   c.gridx = 0;
	   c.gridwidth = 1;
	   c.anchor = GridBagConstraints.LAST_LINE_START;
	   launchGazeboSimButton = new JButton("Launch");
	   networkInfoPanel.add(launchGazeboSimButton, c);
	   
	   launchGazeboSimButton.addActionListener(new ActionListener()
	      {
	         public void actionPerformed(ActionEvent e)
	         {
	        	 new GazeboSimLauncher((LocalCloudMachines)cloudMachineSelectionCombo.getSelectedItem(), (DRCDashboardTypes.DRCTask)taskCombo.getSelectedItem());
	         }
	      });
   }
   
   private void setupEditInfoButton()
   {
      c.gridy = 3;
      c.gridx = 3;
      c.gridwidth = 1;
      c.anchor = GridBagConstraints.LAST_LINE_END;
      modifyRosNetworkConfigsButton = new JButton("Edit Info");
      networkInfoPanel.add(modifyRosNetworkConfigsButton, c);
   }

   private void showFrame()
   {
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setLocationRelativeTo(null);
      frame.setSize(760, 520);
      frame.setResizable(false);
      frame.setVisible(true);
   }

   public void reinitGui()
   {
      frame.getContentPane().removeAll();
      setupJFrame();
      frame.validate();
      frame.repaint();
   }
   
   private String updatedCloudHostnameString()
   {
      return "<html><body style=\"padding-left:8px;\"><br>Cloud Machine Hostname: <br><br><div style=\"padding-left:15px;font-size:1.1em;color:blue;font-weight:bold;\">" + DRCLocalCloudConfig.getHostName((LocalCloudMachines) cloudMachineSelectionCombo.getSelectedItem()) + "</div></body></html>";
   }
   
   private String updatedCloudIpAddressString()
   {
      return "<html><body style=\"padding-left:8px;\"><br>Cloud Machine IP Address: <br><br><div style=\"padding-left:15px;font-size:1.1em;color:blue;font-weight:bold\">" + DRCLocalCloudConfig.getIPAddress((LocalCloudMachines) cloudMachineSelectionCombo.getSelectedItem()) + "</div></body></html>";
   }
   
   private void updateRosMasterURI()
   {
      ROS_MASTER_URI = "http://" + DRCLocalCloudConfig.getIPAddress((LocalCloudMachines) cloudMachineSelectionCombo.getSelectedItem());
      if(rosCorePortField != null) ROS_MASTER_URI += " :" + rosCorePortField.getText();
      else ROS_MASTER_URI += DRCLocalCloudConfig.DEFAULT_ROSCORE_PORT;
      rosUriLabel.setText("ROS Master URI: " + ROS_MASTER_URI.substring(0, ROS_MASTER_URI.lastIndexOf(':') + 1));
   }

   public static DRCDashboard getInstance()
   {
      return instance;
   }

   private void startRedrawSignaler()
   {
      Timer redrawTimer = new Timer(10, new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {

         }
      });

      redrawTimer.start();
   }

   public static void main(String[] args)
   {
      final DRCDashboard dash = new DRCDashboard();

      SwingUtilities.invokeLater(new Runnable()
      {
         public void run()
         {
            dash.setupJFrame();
            dash.showFrame();
         }
      });
   }
}
