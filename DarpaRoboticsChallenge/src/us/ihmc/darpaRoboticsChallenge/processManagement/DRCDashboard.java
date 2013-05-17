package us.ihmc.darpaRoboticsChallenge.processManagement;

import java.awt.Color;
import java.awt.GridBagConstraints;
import java.awt.GridBagLayout;
import java.awt.GridLayout;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.MouseEvent;
import java.awt.event.MouseListener;

import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.DefaultListModel;
import javax.swing.ImageIcon;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JList;
import javax.swing.JOptionPane;
import javax.swing.JPanel;
import javax.swing.JRadioButton;
import javax.swing.JScrollPane;
import javax.swing.JTree;
import javax.swing.ListSelectionModel;
import javax.swing.SwingUtilities;
import javax.swing.Timer;
import javax.swing.border.BevelBorder;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;

import us.ihmc.darpaRoboticsChallenge.DRCGazeboDrivingInterface;
import us.ihmc.darpaRoboticsChallenge.ExternalCameraFeed;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig.LocalCloudMachines;
import us.ihmc.darpaRoboticsChallenge.processManagement.DRCDashboardTypes.DRCTask;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;

public class DRCDashboard
{
   private static DRCDashboard instance;
   //   private String ROS_HOSTNAME;
   //   private String ROS_IP_ADDRESS;
   private String ROS_MASTER_URI;

   private GridBagConstraints c;

   private JFrame frame = new JFrame("IHMC DRC Dashboard");

   private JPanel taskPanel;
   private JLabel taskLabel;
   private JComboBox taskCombo;
   private ButtonGroup radioGroup;
   private JRadioButton usePluginButton;
   private JRadioButton useDefaultButton;

   private JPanel mainContentPanel;

   private JPanel machineSelectionPanel;
   private JPanel gazeboMachineSelectionPanel;
   private JLabel gazeboMachineSelectionLabel;
   private JComboBox gazeboMachineSelectionCombo;
   private JPanel cloudMachineInfoPanel;
   private JLabel cloudMachineIPAddressLabel;
   private JLabel cloudMachineHostnameLabel;
   private JPanel controllerMachineSelectionPanel;
   private JLabel controllerMachineSelectionLabel;
   private JComboBox controllerMachineSelectionCombo;

   private JPanel gazeboUtilitiesPanel;
   private JLabel gazeboProcessListLabel;
   private JList gazeboProcessList;
   private DefaultListModel gazeboProcessListModel;

   private JScrollPane gazeboProcessListScroller;

   private JPanel networkInfoPanel;

   private JPanel processPanel;
   private JScrollPane networkStatusScrollPane;
   private ImageIcon goodConnectionIcon;
   private ImageIcon badConnectionIcon;
   private DefaultMutableTreeNode cloudMonsterNode;
   private DefaultMutableTreeNode cloudMonsterJuniorNode;
   private DefaultMutableTreeNode cloudMinion1Node;
   private DefaultMutableTreeNode cloudMinion2Node;
   private DefaultMutableTreeNode cloudMinion3Node;
   private DefaultMutableTreeNode cloudMinion4Node;

   private JavaProcessSpawner spawner = new JavaProcessSpawner(true);
   private GazeboSimLauncher sshSimLauncher = new GazeboSimLauncher();
   private JTree cloudMonsterTree;
   private JTree cloudMonsterJuniorTree;
   private JTree cloudMinion1Tree;
   private JTree cloudMinion2Tree;
   private JTree cloudMinion3Tree;
   private JTree cloudMinion4Tree;

   public DRCDashboard()
   {
      //      try
      //      {
      //         ROS_HOSTNAME = InetAddress.getLocalHost().getHostName().replace(" ", "-");
      //         ROS_IP_ADDRESS = InetAddress.getLocalHost().getHostAddress();
      //                  ROS_MASTER_URI = "http://";
      //      }
      //      catch (UnknownHostException e)
      //      {
      //         e.printStackTrace();
      //      }

      instance = this;

      startTimedSignalers();
   }

   private void setupJFrame()
   {
      initializeLayout();

      initializeSelectTaskPanel();

      initializeMainContentPanel();

      initializeMachineSelectionControlPanel();

      initializeGazeboUtilitiesPanel();

      setupSelectTaskPanel();

      setupLeftContentPanel();

      setupRightContentPanel();

      setupProcessStatusPanel();

      updateNetworkStatus();

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
      c.weighty = 0;
      taskPanel = new JPanel(new GridBagLayout());
      taskPanel.setBorder(BorderFactory.createEtchedBorder());
      frame.getContentPane().add(taskPanel, c);
   }

   private void initializeMainContentPanel()
   {
      c.gridx = 0;
      c.gridy = 1;
      c.weighty = 1;
      mainContentPanel = new JPanel(new GridBagLayout());
      frame.getContentPane().add(mainContentPanel, c);
   }

   private void initializeMachineSelectionControlPanel()
   {
      c.gridwidth = 4;
      c.gridheight = 10;
      c.gridx = 5;
      c.gridy = 0;
      c.weightx = 0;
      c.ipadx = 80;
      machineSelectionPanel = new JPanel(new GridBagLayout());
      machineSelectionPanel.setBorder(BorderFactory.createEtchedBorder());
      mainContentPanel.add(machineSelectionPanel, c);
   }

   private void initializeGazeboUtilitiesPanel()
   {
      c.gridwidth = 1;
      c.gridx = 0;
      c.weightx = 0.3;
      c.ipadx = 30;
      gazeboUtilitiesPanel = new JPanel(new GridBagLayout());
      gazeboUtilitiesPanel.setBorder(BorderFactory.createEtchedBorder());
      mainContentPanel.add(gazeboUtilitiesPanel, c);
   }

   private void setupProcessStatusPanel()
   {
      processPanel = new JPanel(new GridBagLayout());
      processPanel.setBorder(BorderFactory.createEtchedBorder());

      c.gridx = 4;
      c.gridy = 0;
      c.weighty = 1.0;
      c.weightx = 1.0;
      c.ipadx = 180;
      c.ipady = 388;
      mainContentPanel.add(processPanel, c);

      c.gridx = 0;
      c.gridy = 0;
      c.gridheight = 2;
      c.ipady = 30;
      c.weighty = 1;
      JLabel processPanelLabel = new JLabel("Cloud Status:", JLabel.CENTER);
      processPanel.add(processPanelLabel, c);

      c.gridy += 2;
      c.gridheight = 10;
      c.ipady = 320;
      c.weighty = 0;
      Insets oldInsets = c.insets;

      c.insets = new Insets(2, 10, 17, 10);

      networkStatusScrollPane = new JScrollPane(new JPanel());

      goodConnectionIcon = new ImageIcon(DRCDashboard.class.getResource("good_connection.png"));
      badConnectionIcon = new ImageIcon(DRCDashboard.class.getResource("bad_connection.png"));

      processPanel.add(networkStatusScrollPane, c);

      c.insets = oldInsets;

      JPanel view = (JPanel) networkStatusScrollPane.getViewport().getView();
      view.setBackground(Color.white);
      view.setLayout(new GridLayout(6, 1));
      view.setBorder(BorderFactory.createEmptyBorder(5, 5, 5, 5));

      cloudMonsterNode = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold; font-size:1.1em;\">Cloudmonster</body></html>");
      cloudMonsterNode.add(new DefaultMutableTreeNode("Running ROS/GZ Sims:"));
      cloudMonsterNode.add(new DefaultMutableTreeNode("Running SCS Controllers?"));
      cloudMonsterTree = new JTree(cloudMonsterNode);
      cloudMonsterTree.setLargeModel(true);
      cloudMonsterTree.setBorder(BorderFactory.createEmptyBorder(0, 0, 25, 0));
      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMONSTER))
         setCloudStatusItemIcon(cloudMonsterTree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMonsterTree, badConnectionIcon);
      view.add(cloudMonsterTree);

      cloudMonsterJuniorNode = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold; font-size:1.1em;\">Cloudmonster Jr.</body></html>");
      cloudMonsterJuniorNode.add(new DefaultMutableTreeNode("Running ROS/GZ Sims:"));
      cloudMonsterJuniorNode.add(new DefaultMutableTreeNode("Running SCS Controllers?"));
      cloudMonsterJuniorTree = new JTree(cloudMonsterJuniorNode);
      cloudMonsterJuniorTree.setLargeModel(true);
      cloudMonsterJuniorTree.setBorder(BorderFactory.createEmptyBorder(0, 0, 25, 0));
      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMONSTER_JR))
         setCloudStatusItemIcon(cloudMonsterJuniorTree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMonsterJuniorTree, badConnectionIcon);
      view.add(cloudMonsterJuniorTree);

      cloudMinion1Node = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold; font-size:1.1em;\">Cloudminion 1</body></html>");
      cloudMinion1Node.add(new DefaultMutableTreeNode("Running ROS/GZ Sims:"));
      cloudMinion1Node.add(new DefaultMutableTreeNode("Running SCS Controllers?"));
      cloudMinion1Tree = new JTree(cloudMinion1Node);
      cloudMinion1Tree.setLargeModel(true);
      cloudMinion1Tree.setBorder(BorderFactory.createEmptyBorder(0, 0, 25, 0));
      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMINION_1))
         setCloudStatusItemIcon(cloudMinion1Tree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMinion1Tree, badConnectionIcon);
      view.add(cloudMinion1Tree);

      cloudMinion2Node = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold; font-size:1.1em;\">Cloudminion 2</body></html>");
      cloudMinion2Node.add(new DefaultMutableTreeNode("Running ROS/GZ Sims:"));
      cloudMinion2Node.add(new DefaultMutableTreeNode("Running SCS Controllers?"));
      cloudMinion2Tree = new JTree(cloudMinion2Node);
      cloudMinion2Tree.setLargeModel(true);
      cloudMinion2Tree.setBorder(BorderFactory.createEmptyBorder(0, 0, 25, 0));
      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMINION_2))
         setCloudStatusItemIcon(cloudMinion2Tree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMinion2Tree, badConnectionIcon);
      view.add(cloudMinion2Tree);

      cloudMinion3Node = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold; font-size:1.1em;\">Cloudminion 3</body></html>");
      cloudMinion3Node.add(new DefaultMutableTreeNode("Running ROS/GZ Sims:"));
      cloudMinion3Node.add(new DefaultMutableTreeNode("Running SCS Controllers?"));
      cloudMinion3Tree = new JTree(cloudMinion3Node);
      cloudMinion3Tree.setLargeModel(true);
      cloudMinion3Tree.setBorder(BorderFactory.createEmptyBorder(0, 0, 25, 0));
      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMINION_3))
         setCloudStatusItemIcon(cloudMinion3Tree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMinion3Tree, badConnectionIcon);
      view.add(cloudMinion3Tree);

      cloudMinion4Node = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold; font-size:1.1em;\">Cloudminion 4</body></html>");
      cloudMinion4Node.add(new DefaultMutableTreeNode("Running ROS/GZ Sims:"));
      cloudMinion4Node.add(new DefaultMutableTreeNode("Running SCS Controllers?"));
      cloudMinion4Tree = new JTree(cloudMinion4Node);
      cloudMinion4Tree.setLargeModel(true);
      cloudMinion4Tree.setBorder(BorderFactory.createEmptyBorder(0, 0, 25, 0));
      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMINION_4))
         setCloudStatusItemIcon(cloudMinion4Tree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMinion4Tree, badConnectionIcon);
      view.add(cloudMinion4Tree);

      disableNodeCollapse();

      setupStatusPanelMouseListeners();

   }

   private void setupStatusPanelMouseListeners()
   {
      blockLeafSelection();
      
      cloudMonsterTree.addMouseListener(new MouseListener()
      {
         
         public void mouseReleased(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mousePressed(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseExited(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseEntered(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() > 1)
            {
               LocalCloudMachines gazeboMachine = LocalCloudMachines.CLOUDMONSTER;
               LocalCloudMachines controllerMachine = (LocalCloudMachines) controllerMachineSelectionCombo.getSelectedItem();
               DRCTask task = (DRCTask) taskCombo.getSelectedItem();                  
               String pluginOption = radioGroup.getSelection().getActionCommand();
                              
               if (!sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMONSTER))
               {
                  String[] options = new String[] { "Yes", "No" };
                  int n = JOptionPane.showOptionDialog(frame, "Do you want to launch " + task.toString() + " on " + gazeboMachine.toString() + "?",
                        "Confirm Launch ROS/Gazebo Sim", JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null, options, options[0]);

                  if (n == 0)
                     sshSimLauncher.launchSim(task, gazeboMachine, controllerMachine, pluginOption);
               }
               else
               {
                  JOptionPane.showMessageDialog(frame, "Machine is already running a ROS Sim!", "ROS/Gazebo Sim Launch Error", JOptionPane.ERROR_MESSAGE);
               }
            }            
         }
      });
      
      cloudMonsterJuniorTree.addMouseListener(new MouseListener()
      {
         
         public void mouseReleased(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mousePressed(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseExited(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseEntered(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() > 1)
            {
               LocalCloudMachines gazeboMachine = LocalCloudMachines.CLOUDMONSTER_JR;
               LocalCloudMachines controllerMachine = (LocalCloudMachines) controllerMachineSelectionCombo.getSelectedItem();
               DRCTask task = (DRCTask) taskCombo.getSelectedItem();
               String pluginOption = radioGroup.getSelection().getActionCommand();

               if (!sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMONSTER_JR))
               {
                  String[] options = new String[] { "Yes", "No" };
                  int n = JOptionPane.showOptionDialog(frame, "Do you want to launch " + task.toString() + " on " + gazeboMachine.toString() + "?",
                        "Confirm Launch ROS/Gazebo Sim", JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null, options, options[0]);

                  if (n == 0)
                     sshSimLauncher.launchSim(task, gazeboMachine, controllerMachine, pluginOption);
               }
               else
               {
                  JOptionPane.showMessageDialog(frame, "Machine is already running a ROS Sim!", "ROS/Gazebo Sim Launch Error", JOptionPane.ERROR_MESSAGE);
               }
            }            
         }
      });
      
      cloudMinion1Tree.addMouseListener(new MouseListener()
      {
         
         public void mouseReleased(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mousePressed(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseExited(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseEntered(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() > 1)
            {
               LocalCloudMachines gazeboMachine = LocalCloudMachines.CLOUDMINION_1;
               LocalCloudMachines controllerMachine = (LocalCloudMachines) controllerMachineSelectionCombo.getSelectedItem();
               DRCTask task = (DRCTask) taskCombo.getSelectedItem();
               String pluginOption = radioGroup.getSelection().getActionCommand();

               if (!sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMINION_1))
               {
                  String[] options = new String[] { "Yes", "No" };
                  int n = JOptionPane.showOptionDialog(frame, "Do you want to launch " + task.toString() + " on " + gazeboMachine.toString() + "?",
                        "Confirm Launch ROS/Gazebo Sim", JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null, options, options[0]);

                  if (n == 0)
                     sshSimLauncher.launchSim(task, gazeboMachine, controllerMachine, pluginOption);
               }
               else
               {
                  JOptionPane.showMessageDialog(frame, "Machine is already running a ROS Sim!", "ROS/Gazebo Sim Launch Error", JOptionPane.ERROR_MESSAGE);
               }
            }            
         }
      });
      
      cloudMinion2Tree.addMouseListener(new MouseListener()
      {
         
         public void mouseReleased(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mousePressed(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseExited(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseEntered(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() > 1)
            {
               LocalCloudMachines gazeboMachine = LocalCloudMachines.CLOUDMINION_2;
               LocalCloudMachines controllerMachine = (LocalCloudMachines) controllerMachineSelectionCombo.getSelectedItem();
               DRCTask task = (DRCTask) taskCombo.getSelectedItem();
               String pluginOption = radioGroup.getSelection().getActionCommand();

               if (!sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMINION_2))
               {
                  String[] options = new String[] { "Yes", "No" };
                  int n = JOptionPane.showOptionDialog(frame, "Do you want to launch " + task.toString() + " on " + gazeboMachine.toString() + "?",
                        "Confirm Launch ROS/Gazebo Sim", JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null, options, options[0]);

                  if (n == 0)
                     sshSimLauncher.launchSim(task, gazeboMachine, controllerMachine, pluginOption);
               }
               else
               {
                  JOptionPane.showMessageDialog(frame, "Machine is already running a ROS Sim!", "ROS/Gazebo Sim Launch Error", JOptionPane.ERROR_MESSAGE);
               }
            }            
         }
      });
      
      cloudMinion3Tree.addMouseListener(new MouseListener()
      {
         
         public void mouseReleased(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mousePressed(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseExited(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseEntered(MouseEvent arg0)
         {
            // TODO Auto-generated method stub
            
         }
         
         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() > 1)
            {
               LocalCloudMachines gazeboMachine = LocalCloudMachines.CLOUDMINION_3;
               LocalCloudMachines controllerMachine = (LocalCloudMachines) controllerMachineSelectionCombo.getSelectedItem();
               DRCTask task = (DRCTask) taskCombo.getSelectedItem();
               String pluginOption = radioGroup.getSelection().getActionCommand();

               if (!sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMINION_3))
               {
                  String[] options = new String[] { "Yes", "No" };
                  int n = JOptionPane.showOptionDialog(frame, "Do you want to launch " + task.toString() + " on " + gazeboMachine.toString() + "?",
                        "Confirm Launch ROS/Gazebo Sim", JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null, options, options[0]);

                  if (n == 0)
                     sshSimLauncher.launchSim(task, gazeboMachine, controllerMachine, pluginOption);
               }
               else
               {
                  JOptionPane.showMessageDialog(frame, "Machine is already running a ROS Sim!", "ROS/Gazebo Sim Launch Error", JOptionPane.ERROR_MESSAGE);
               }
            }            
         }
      });

      cloudMinion4Tree.addMouseListener(new MouseListener()
      {
         public void mouseReleased(MouseEvent e)
         {
            // TODO Auto-generated method stub

         }

         public void mousePressed(MouseEvent e)
         {
            // TODO Auto-generated method stub

         }

         public void mouseExited(MouseEvent e)
         {
            // TODO Auto-generated method stub

         }

         public void mouseEntered(MouseEvent e)
         {
            // TODO Auto-generated method stub

         }

         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() > 1)
            {
               LocalCloudMachines gazeboMachine = LocalCloudMachines.CLOUDMINION_4;
               LocalCloudMachines controllerMachine = (LocalCloudMachines) controllerMachineSelectionCombo.getSelectedItem();
               DRCTask task = (DRCTask) taskCombo.getSelectedItem();
               String pluginOption = radioGroup.getSelection().getActionCommand();

               if (!sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMINION_4))
               {
                  String[] options = new String[] { "Yes", "No" };
                  int n = JOptionPane.showOptionDialog(frame, "Do you want to launch " + task.toString() + " on " + gazeboMachine.toString() + "?",
                        "Confirm Launch ROS/Gazebo Sim", JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null, options, options[0]);

                  if (n == 0)
                     sshSimLauncher.launchSim(task, gazeboMachine, controllerMachine, pluginOption);
               }
               else
               {
                  JOptionPane.showMessageDialog(frame, "Machine is already running a ROS Sim!", "ROS/Gazebo Sim Launch Error", JOptionPane.ERROR_MESSAGE);
               }
            }
         }
      });

   }

   private void blockLeafSelection()
   {
      TreeSelectionListener blockLeafSelectionListener = new TreeSelectionListener()
      {
         public void valueChanged(TreeSelectionEvent e)
         {
            JTree tree = (JTree) e.getSource();
            DefaultMutableTreeNode node = (DefaultMutableTreeNode) tree.getLastSelectedPathComponent();

            if (node != null && node.isLeaf())
            {
               tree.getSelectionModel().clearSelection();
            }
         }
      };

      cloudMonsterTree.addTreeSelectionListener(blockLeafSelectionListener);
      cloudMonsterJuniorTree.addTreeSelectionListener(blockLeafSelectionListener);
      cloudMinion1Tree.addTreeSelectionListener(blockLeafSelectionListener);
      cloudMinion2Tree.addTreeSelectionListener(blockLeafSelectionListener);
      cloudMinion3Tree.addTreeSelectionListener(blockLeafSelectionListener);
      cloudMinion4Tree.addTreeSelectionListener(blockLeafSelectionListener);
   }

   private void disableNodeCollapse()
   {
      cloudMonsterTree.setToggleClickCount(0);
      cloudMonsterJuniorTree.setToggleClickCount(0);
      cloudMinion1Tree.setToggleClickCount(0);
      cloudMinion2Tree.setToggleClickCount(0);
      cloudMinion3Tree.setToggleClickCount(0);
      cloudMinion4Tree.setToggleClickCount(0);
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
      taskCombo = new JComboBox(DRCDashboardTypes.DRCTask.values());
      taskCombo.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            gazeboProcessListModel.clear();

            //            if (taskCombo.getSelectedItem().toString().contains("DRIVING"))
            //            {
            //               gazeboProcessListModel.addElement("DRC Driving Interface");
            //               
            //               if (taskCombo.getSelectedItem().toString().contains("_WITH_EXTERNAL_CAMS"))
            //               {
            //                  gazeboProcessListModel.addElement("Topview Camera");
            //                  gazeboProcessListModel.addElement("Rearview Camera");
            //               }
            //            }
         }
      });
      taskPanel.add(taskCombo, c);
      
      radioGroup = new ButtonGroup();
      useDefaultButton = new JRadioButton("Use ROS Synchronization Layer", true);
      useDefaultButton.setActionCommand("default");
      usePluginButton = new JRadioButton("Use Jesper Plügin Synchronization");
      usePluginButton.setActionCommand("plugin");

      radioGroup.add(useDefaultButton);
      radioGroup.add(usePluginButton);

      c.weightx = 0;
      c.gridx = 1;
      c.gridy = 0;
      taskPanel.add(useDefaultButton, c);

      c.gridy = 1;
      taskPanel.add(usePluginButton, c);
   }

   private void setupLeftContentPanel()
   {
      setupSelectControllerMachine();

      setupSelectGazeboMachine();

      setupCloudMachineInfoPanel();
   }

   private void setupSelectControllerMachine()
   {
      c.gridx = 0;
      c.gridy = 0;
      c.gridwidth = 1;
      c.gridheight = 2;
      c.weighty = 0.0;

      controllerMachineSelectionPanel = new JPanel(new GridLayout(2, 1));
      machineSelectionPanel.add(controllerMachineSelectionPanel, c);
      controllerMachineSelectionLabel = new JLabel("Select Controller Machine: ", JLabel.LEFT);
      controllerMachineSelectionCombo = new JComboBox(LocalCloudMachines.values());
      controllerMachineSelectionCombo.setEnabled(false);
      controllerMachineSelectionPanel.add(controllerMachineSelectionLabel);
      controllerMachineSelectionPanel.add(controllerMachineSelectionCombo);
   }

   private void setupSelectGazeboMachine()
   {
      c.gridy = 2;
      gazeboMachineSelectionPanel = new JPanel(new GridLayout(2, 1));
      machineSelectionPanel.add(gazeboMachineSelectionPanel, c);
      gazeboMachineSelectionLabel = new JLabel("Select Gazebo Machine: ", JLabel.LEFT);
      gazeboMachineSelectionPanel.add(gazeboMachineSelectionLabel);
      gazeboMachineSelectionCombo = new JComboBox(LocalCloudMachines.values());
      gazeboMachineSelectionCombo.setEnabled(false);
      gazeboMachineSelectionPanel.add(gazeboMachineSelectionCombo);

      gazeboMachineSelectionCombo.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            cloudMachineHostnameLabel.setText(updatedCloudHostnameString());
            cloudMachineIPAddressLabel.setText(updatedCloudIpAddressString());
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
      machineSelectionPanel.add(cloudMachineInfoPanel, c);
      cloudMachineHostnameLabel = new JLabel(updatedCloudHostnameString());
      cloudMachineIPAddressLabel = new JLabel(updatedCloudIpAddressString());
//      cloudMachineInfoPanel.add(cloudMachineHostnameLabel);
//      cloudMachineInfoPanel.add(cloudMachineIPAddressLabel);
      c.insets = new Insets(5, 5, 5, 5);
   }

   private void setupRightContentPanel()
   {
      setupGazeboLauncherList();

      setupNetworkInfoPanel();

      normalizeGridBagConstraints();      
   }

   private void setupGazeboLauncherList()
   {
      c.gridwidth = 1;
      c.gridheight = 1;
      c.gridx = 0;
      c.gridy = 0;
      c.ipady = 0;
      c.fill = GridBagConstraints.NONE;

      c.ipadx = gazeboUtilitiesPanel.getWidth();
      gazeboProcessListLabel = new JLabel("Gazebo Utilities:", JLabel.CENTER);
      gazeboUtilitiesPanel.add(gazeboProcessListLabel, c);

      c.gridy = 1;
      c.gridheight = 5;
      c.weighty = 100;
      c.ipady = 240;
      c.ipadx = 200;
      c.anchor = GridBagConstraints.PAGE_START;
      c.fill = GridBagConstraints.BOTH;
      //      c.insets = new Insets(10, 35, 10, 35);
      gazeboProcessListModel = new DefaultListModel();
      gazeboProcessList = new JList(gazeboProcessListModel);
      gazeboProcessList.addMouseListener(new MouseListener()
      {

         public void mouseClicked(MouseEvent e)
         {
            if (e.getClickCount() == 2 && !gazeboProcessListModel.isEmpty())
            {

               if (gazeboProcessList.getSelectedValue().toString() == "Topview Camera")
               {
                  String[] arguments = new String[] { ROS_MASTER_URI, "topviewCam", "/topview/compressed" };
                  spawner.spawn(ExternalCameraFeed.class, arguments);
               }

               if (gazeboProcessList.getSelectedValue().toString() == "Rearview Camera")
               {
                  String[] arguments = new String[] { ROS_MASTER_URI, "rearviewCam", "/rearview/compressed" };
                  spawner.spawn(ExternalCameraFeed.class, arguments);
               }

               if (gazeboProcessList.getSelectedValue().toString() == "DRC Driving Interface")
               {
                  String[] arguments = new String[] { ROS_MASTER_URI };
                  spawner.spawn(DRCGazeboDrivingInterface.class, arguments);
               }
            }
         }

         public void mousePressed(MouseEvent e)
         {
         }

         public void mouseReleased(MouseEvent e)
         {
         }

         public void mouseEntered(MouseEvent e)
         {
         }

         public void mouseExited(MouseEvent e)
         {
         }

      });
      gazeboProcessList.setSelectionMode(ListSelectionModel.SINGLE_SELECTION);
      gazeboProcessList.setLayoutOrientation(JList.VERTICAL);
      gazeboProcessListScroller = new JScrollPane(gazeboProcessList);
      gazeboUtilitiesPanel.add(gazeboProcessListScroller, c);
   }

   private void setupNetworkInfoPanel()
   {
      c.gridy = 6;
      c.gridheight = 4;
      c.weighty = 0.1;
      c.ipady = 0;
      c.insets = new Insets(5, 5, 5, 5);
      networkInfoPanel = new JPanel(new GridBagLayout());
      gazeboUtilitiesPanel.add(networkInfoPanel, c);

   }

   private void normalizeGridBagConstraints()
   {
      c.gridy = 3;
      c.gridx = 0;
      c.gridwidth = 1;
   }

   private void showFrame()
   {
      frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
      frame.setLocationRelativeTo(null);
      frame.setSize(1000, 520);
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
      return "<html><body style=\"padding-left:8px;\"><br>Gazebo Machine Hostname: <br><br><div style=\"padding-left:15px;font-size:1.1em;color:blue;font-weight:bold;\">"
            + DRCLocalCloudConfig.getHostName((LocalCloudMachines) gazeboMachineSelectionCombo.getSelectedItem()) + "</div></body></html>";
   }

   private String updatedCloudIpAddressString()
   {
      return "<html><body style=\"padding-left:8px;\"><br>Gazebo Machine IP Address: <br><br><div style=\"padding-left:15px;font-size:1.1em;color:blue;font-weight:bold\">"
            + DRCLocalCloudConfig.getIPAddress((LocalCloudMachines) gazeboMachineSelectionCombo.getSelectedItem()) + "</div></body></html>";
   }

   public static DRCDashboard getInstance()
   {
      return instance;
   }

   private void startTimedSignalers()
   {
      Timer redrawTimer = new Timer(10, new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
         }
      });

      Timer netStatTimer = new Timer(10000, new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            updateNetworkStatus();
         }
      });

      redrawTimer.start();
      netStatTimer.start();
   }

   private void updateNetworkStatus()
   {
      SwingUtilities.invokeLater(new Runnable()
      {
         public void run()
         {
            updateStatusIndicators();

            updateRemoteProcessListings();
            
            ((JPanel) networkStatusScrollPane.getViewport().getView()).repaint();
         }
      });
   }

   private void updateRemoteProcessListings()
   {
      updateRosSimStatuses();

      updateSCSStatuses();
   }

   private void updateRosSimStatuses()
   {
      if (sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMONSTER))
         ((DefaultMutableTreeNode) cloudMonsterNode.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:red;font-style:italic;\">"
                     + sshSimLauncher.getSimTaskname(LocalCloudMachines.CLOUDMONSTER) + "</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMonsterNode.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMONSTER_JR))
         ((DefaultMutableTreeNode) cloudMonsterJuniorNode.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:red;font-style:italic;\">"
                     + sshSimLauncher.getSimTaskname(LocalCloudMachines.CLOUDMONSTER_JR) + "</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMonsterJuniorNode.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMINION_1))
         ((DefaultMutableTreeNode) cloudMinion1Node.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:red;font-style:italic;\">"
                     + sshSimLauncher.getSimTaskname(LocalCloudMachines.CLOUDMINION_1) + "</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMinion1Node.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMINION_2))
         ((DefaultMutableTreeNode) cloudMinion2Node.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:red;font-style:italic;\">"
                     + sshSimLauncher.getSimTaskname(LocalCloudMachines.CLOUDMINION_2) + "</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMinion2Node.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMINION_3))
         ((DefaultMutableTreeNode) cloudMinion3Node.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:red;font-style:italic;\">"
                     + sshSimLauncher.getSimTaskname(LocalCloudMachines.CLOUDMINION_3) + "</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMinion3Node.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningSim(LocalCloudMachines.CLOUDMINION_4))
         ((DefaultMutableTreeNode) cloudMinion4Node.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:red;font-style:italic;\">"
                     + sshSimLauncher.getSimTaskname(LocalCloudMachines.CLOUDMINION_4) + "</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMinion4Node.getChildAt(0))
               .setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:green;font-style:italic;\">No</span></body></html>");
   }

   private void updateSCSStatuses()
   {
      if (sshSimLauncher.isMachineRunningController(LocalCloudMachines.CLOUDMONSTER))
         ((DefaultMutableTreeNode) cloudMonsterNode.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:red;font-style:italic;\">Yes</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMonsterNode.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningController(LocalCloudMachines.CLOUDMONSTER_JR))
         ((DefaultMutableTreeNode) cloudMonsterJuniorNode.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:red;font-style:italic;\">Yes</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMonsterJuniorNode.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningController(LocalCloudMachines.CLOUDMINION_1))
         ((DefaultMutableTreeNode) cloudMinion1Node.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:red;font-style:italic;\">Yes</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMinion1Node.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningController(LocalCloudMachines.CLOUDMINION_2))
         ((DefaultMutableTreeNode) cloudMinion2Node.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:red;font-style:italic;\">Yes</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMinion2Node.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningController(LocalCloudMachines.CLOUDMINION_3))
         ((DefaultMutableTreeNode) cloudMinion3Node.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:red;font-style:italic;\">Yes</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMinion3Node.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:green;font-style:italic;\">No</span></body></html>");

      if (sshSimLauncher.isMachineRunningController(LocalCloudMachines.CLOUDMINION_4))
         ((DefaultMutableTreeNode) cloudMinion4Node.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:red;font-style:italic;\">Yes</span></body></html>");
      else
         ((DefaultMutableTreeNode) cloudMinion4Node.getChildAt(1))
               .setUserObject("<html><body>Running SCS Controller? <span style=\"color:green;font-style:italic;\">No</span></body></html>");
   }

   private void updateStatusIndicators()
   {
      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMONSTER))
         setCloudStatusItemIcon(cloudMonsterTree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMonsterTree, badConnectionIcon);

      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMONSTER_JR))
         setCloudStatusItemIcon(cloudMonsterJuniorTree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMonsterJuniorTree, badConnectionIcon);

      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMINION_1))
         setCloudStatusItemIcon(cloudMinion1Tree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMinion1Tree, badConnectionIcon);

      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMINION_2))
         setCloudStatusItemIcon(cloudMinion2Tree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMinion2Tree, badConnectionIcon);

      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMINION_3))
         setCloudStatusItemIcon(cloudMinion3Tree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMinion3Tree, badConnectionIcon);

      if (sshSimLauncher.isMachineReachable(LocalCloudMachines.CLOUDMINION_4))
         setCloudStatusItemIcon(cloudMinion4Tree, goodConnectionIcon);
      else
         setCloudStatusItemIcon(cloudMinion4Tree, badConnectionIcon);
   }

   private void setCloudStatusItemIcon(JTree cloudStatusSubtree, ImageIcon icon)
   {
      DefaultTreeCellRenderer renderer = (DefaultTreeCellRenderer) cloudStatusSubtree.getCellRenderer();

      renderer.setOpenIcon(icon);
      renderer.setClosedIcon(icon);
      renderer.setLeafIcon(null);
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
