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
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.TimerTask;

import javax.swing.BorderFactory;
import javax.swing.ButtonGroup;
import javax.swing.DefaultComboBoxModel;
import javax.swing.DefaultListModel;
import javax.swing.ImageIcon;
import javax.swing.JCheckBox;
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
import javax.swing.UIManager;
import javax.swing.UnsupportedLookAndFeelException;
import javax.swing.border.BevelBorder;
import javax.swing.event.TreeSelectionEvent;
import javax.swing.event.TreeSelectionListener;
import javax.swing.tree.DefaultMutableTreeNode;
import javax.swing.tree.DefaultTreeCellRenderer;
import javax.swing.tree.DefaultTreeModel;

import org.apache.commons.lang.WordUtils;

import us.ihmc.darpaRoboticsChallenge.DRCConfigParameters;
import us.ihmc.darpaRoboticsChallenge.DRCDemo01;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig;
import us.ihmc.darpaRoboticsChallenge.configuration.DRCLocalCloudConfig.LocalCloudMachines;
import us.ihmc.darpaRoboticsChallenge.processManagement.DRCDashboardTypes.DRCPluginTasks;
import us.ihmc.darpaRoboticsChallenge.processManagement.DRCDashboardTypes.DRCROSTasks;
import us.ihmc.darpaRoboticsChallenge.userInterface.DRCOperatorUserInterface;
import us.ihmc.utilities.Pair;
import us.ihmc.utilities.ThreadTools;
import us.ihmc.utilities.processManagement.JavaProcessSpawner;

public class DRCDashboard
{
   private static DRCDashboard instance;

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

   JCheckBox operatorUICheckBox;
   JCheckBox scsCheckBox;

   private JPanel processPanel;
   private JScrollPane networkStatusScrollPane;
   private ImageIcon goodConnectionIcon;
   private ImageIcon badConnectionIcon;

   private JavaProcessSpawner uiSpawner = new JavaProcessSpawner(true);
   private JavaProcessSpawner scsSpawner = new JavaProcessSpawner(true);
   private GazeboSimLauncher sshSimLauncher = new GazeboSimLauncher();

   private HashMap<LocalCloudMachines, Pair<JTree, DefaultMutableTreeNode>> cloudMachineTrees = new HashMap<LocalCloudMachines, Pair<JTree, DefaultMutableTreeNode>>();
   JTree currentSelection = null;

   private ArrayList<LocalCloudMachines> userOwnedSims = new ArrayList<DRCLocalCloudConfig.LocalCloudMachines>();

   private File configFileHandle;
   private boolean shouldLoadConfig = false;

   public DRCDashboard()
   {
      setNativeLookAndFeel();

      instance = this;

      initConfig();
   }

   private void setNativeLookAndFeel()
   {
      try
      {
         UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
      }
      catch (ClassNotFoundException e)
      {

         e.printStackTrace();
      }
      catch (InstantiationException e)
      {

         e.printStackTrace();
      }
      catch (IllegalAccessException e)
      {

         e.printStackTrace();
      }
      catch (UnsupportedLookAndFeelException e)
      {

         e.printStackTrace();
      }
   }

   private void initConfig()
   {
      String fileName = "dashboard.prefs";

      File configFile = new File(fileName);

      if (configFile.exists() && configFile.length() > 0)
      {
         configFileHandle = configFile;
         shouldLoadConfig = true;
      }
      else
      {
         try
         {
            configFile.createNewFile();
            configFileHandle = configFile;
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }
      }
   }

   private void loadConfig()
   {
      try
      {
         BufferedReader reader = new BufferedReader(new FileReader(configFileHandle));
         String line, pluginOption = "default";
         boolean done = false;
         while (!done)
         {
            line = reader.readLine();
            if (line.contains("END") || line == null)
            {
               done = true;
            }
            else
            {
               if (line != null && line.startsWith("PLUGIN:"))
               {
                  pluginOption = line.substring(line.indexOf(":") + 1, line.length());
                  if (pluginOption.contains("plugin"))
                  {
                     radioGroup.setSelected(usePluginButton.getModel(), true);
                  }
                  else
                  {
                     radioGroup.setSelected(useDefaultButton.getModel(), true);
                  }
               }
               else if (line != null && line.startsWith("TASK:"))
               {
                  String taskOption = line.substring(line.indexOf(":") + 1, line.length());

                  forceTaskComboUpdate();

                  if (pluginOption.contains("plugin"))
                     taskCombo.setSelectedItem(DRCPluginTasks.valueOf(taskOption));
                  else
                     taskCombo.setSelectedItem(DRCROSTasks.valueOf(taskOption));
               }
               else if (line != null && line.startsWith("UI:"))
               {
                  String uiOption = line.substring(line.indexOf(":") + 1, line.length());

                  if (uiOption.contains("true"))
                     operatorUICheckBox.setSelected(true);
                  else
                     operatorUICheckBox.setSelected(false);
               }
               else if (line != null && line.startsWith("SCS:"))
               {
                  String scsOption = line.substring(line.indexOf(":") + 1, line.length());

                  if (scsOption.contains("true"))
                     scsCheckBox.setSelected(true);
                  else
                     scsCheckBox.setSelected(false);
               }
            }
         }
         reader.close();
      }
      catch (FileNotFoundException e)
      {
         e.printStackTrace();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   private void writeConfigFile()
   {
      System.out.println("Writing config...");
      String taskOption = taskCombo.getSelectedItem().toString();
      String pluginOption = radioGroup.getSelection().getActionCommand();

      try
      {
         BufferedWriter fileWriter = new BufferedWriter(new FileWriter(configFileHandle));
         fileWriter.write("PLUGIN:" + pluginOption);
         fileWriter.newLine();
         fileWriter.write("TASK:" + taskOption);
         fileWriter.newLine();
         fileWriter.write("UI:" + (operatorUICheckBox.isSelected() ? "true" : "false"));
         fileWriter.newLine();
         fileWriter.write("SCS:" + (scsCheckBox.isSelected() ? "true" : "false"));
         fileWriter.newLine();
         fileWriter.write("END");
         fileWriter.flush();
         fileWriter.close();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
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

      setupFrameCloseListener();

      if (shouldLoadConfig)
         loadConfig();

      startTimers();
   }

   private void forceTaskComboUpdate()
   {
      DefaultComboBoxModel model = (DefaultComboBoxModel) taskCombo.getModel();

      model.removeAllElements();

      if (radioGroup.getSelection().getActionCommand().contains("plugin"))
      {
         for (DRCPluginTasks task : DRCPluginTasks.values())
         {
            model.addElement(task);
         }
      }
      else
      {
         for (DRCROSTasks task : DRCROSTasks.values())
         {
            model.addElement(task);
         }
      }
   }

   private void setupFrameCloseListener()
   {
      frame.addWindowListener(new WindowListener()
      {

         public void windowOpened(WindowEvent arg0)
         {

         }

         public void windowIconified(WindowEvent arg0)
         {

         }

         public void windowDeiconified(WindowEvent arg0)
         {

         }

         public void windowDeactivated(WindowEvent arg0)
         {

         }

         public void windowClosing(WindowEvent arg0)
         {
            writeConfigFile();
         }

         public void windowClosed(WindowEvent arg0)
         {

         }

         public void windowActivated(WindowEvent arg0)
         {

         }
      });
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

      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
         {
            DefaultMutableTreeNode rootNode = new DefaultMutableTreeNode("<html><body style=\"font-weight:bold;font-size:1.1em;\">"
                  + WordUtils.capitalize(machine.toString().toLowerCase().replace("_", " ")) + "</body></html>");
            rootNode.add(new DefaultMutableTreeNode("Running ROS/GZ Sims:"));
            rootNode.add(new DefaultMutableTreeNode("Running SCS Controllers?"));

            JTree tree = new JTree(rootNode);
            tree.setLargeModel(true);
            tree.setBorder(BorderFactory.createEmptyBorder(15, 10, 15, 0));
            if (sshSimLauncher.isMachineReachable(machine))
               setCloudStatusItemIcon(tree, goodConnectionIcon);
            else
               setCloudStatusItemIcon(tree, badConnectionIcon);

            view.add(tree);
            cloudMachineTrees.put(machine, new Pair<JTree, DefaultMutableTreeNode>(tree, rootNode));
         }
      }

      disableNodeCollapse();

      setupStatusPanelMouseListeners();
   }

   private void setupStatusPanelMouseListeners()
   {
      setupTreeSelection();

      setupDoubleClickListener();
   }

   private void setupDoubleClickListener()
   {
      for (final LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
            cloudMachineTrees.get(machine).first().addMouseListener(new MouseListener()
            {

               public void mouseReleased(MouseEvent e)
               {

               }

               public void mousePressed(MouseEvent e)
               {

               }

               public void mouseExited(MouseEvent e)
               {

               }

               public void mouseEntered(MouseEvent e)
               {

               }

               public void mouseClicked(MouseEvent e)
               {
                  ((JTree) e.getSource()).setSelectionRow(0);
                  if (e.getClickCount() > 1)
                  {
                     final LocalCloudMachines gazeboMachine = machine;
                     final LocalCloudMachines controllerMachine = (LocalCloudMachines) controllerMachineSelectionCombo.getSelectedItem();
                     final String task = taskCombo.getSelectedItem().toString();
                     final String pluginOption = radioGroup.getSelection().getActionCommand();
                     if (sshSimLauncher.isMachineReachable(machine))
                     {
                        if (!sshSimLauncher.isMachineRunningSim(gazeboMachine))
                        {
                           String[] options = new String[] { "Yes", "No" };
                           int n = JOptionPane.showOptionDialog(frame, "Do you want to launch " + task.toString() + " on " + gazeboMachine.toString() + "?",
                                 "Confirm Launch ROS/Gazebo Sim", JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null, options, options[0]);

                           if (n == 0)
                           {
                              sshSimLauncher.launchSim(task, gazeboMachine, controllerMachine, pluginOption);
                              userOwnedSims.add(gazeboMachine);
                              
                              if (operatorUICheckBox.isSelected() && !uiSpawner.hasRunningProcesses())
                              {
                                 uiSpawner.spawn(DRCOperatorUserInterface.class, new String[] { "-Xms1024m", "-Xmx2048m" }, null);
                              }

                              new Thread(new Runnable()
                              {
                                 public void run()
                                 {                                    
                                    if (scsCheckBox.isSelected() && !scsSpawner.hasRunningProcesses())
                                    {
                                       if (pluginOption.contains("plugin"))
                                       {
                                          String newTask = "";

                                          if (task.toLowerCase().contains("vehicle"))
                                             newTask = "ONLY_VEHICLE";
                                          else
                                             newTask = task;

                                          if (newTask.toLowerCase().contains("parking") || newTask.toLowerCase().contains("hand")
                                                || newTask.toLowerCase().equals("atlas"))
                                          {
                                             System.err.println("Can't launch SCS; no environment for selected task");
                                             return;
                                          }
                                          ThreadTools.sleep(5000);
                                          scsSpawner.spawn(DRCDemo01.class, new String[] { "-Xms1024m", "-Xmx2048m" }, new String[] { "--sim", "--env",
                                                newTask, "--gazebo", "--gazeboHost", DRCConfigParameters.GAZEBO_HOST });

                                       }
                                       else
                                       {
                                          System.err.println("Launching SCS without Jesper plugin not yet implemented");
                                       }
                                    }
                                 }
                              }).start();       
                           }
                        }
                        else if (userOwnedSims.contains(gazeboMachine))
                        {
                           String[] options = new String[] { "Yes", "No" };
                           int n = JOptionPane.showOptionDialog(frame, "Do you want to kill your sim running on " + gazeboMachine.toString() + "?",
                                 "Confirm Kill ROS/Gazebo Sim", JOptionPane.YES_NO_CANCEL_OPTION, JOptionPane.WARNING_MESSAGE, null, options, options[0]);

                           if (n == 0)
                           {
                              sshSimLauncher.killSim(gazeboMachine, controllerMachine);
                              userOwnedSims.remove(gazeboMachine);
                           }
                        }
                        else
                        {
                           JOptionPane.showMessageDialog(frame, "Machine is running somebody else's sim!", "ROS/Gazebo Sim Launch Error",
                                 JOptionPane.ERROR_MESSAGE);
                        }
                     }
                     else
                     {
                        JOptionPane.showMessageDialog(frame, "Machine is offline!", "ROS/Gazebo Sim Launch Error", JOptionPane.ERROR_MESSAGE);
                     }
                  }

               }
            });
      }
   }

   private void setupTreeSelection()
   {
      TreeSelectionListener customTreeSelectionListener = new TreeSelectionListener()
      {
         private Color white = Color.white;
         private Color selectionColor = new Color(232, 236, 241);

         public void valueChanged(TreeSelectionEvent e)
         {
            JTree tree = (JTree) e.getSource();
            tree.getSelectionModel().clearSelection();

            if (currentSelection != null)
            {
               currentSelection.setBackground(white);
               ((DefaultTreeCellRenderer) currentSelection.getCellRenderer()).setBackgroundNonSelectionColor(white);
            }

            currentSelection = tree;
            currentSelection.setBackground(selectionColor);
            ((DefaultTreeCellRenderer) currentSelection.getCellRenderer()).setBackgroundNonSelectionColor(selectionColor);
         }
      };

      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
            cloudMachineTrees.get(machine).first().addTreeSelectionListener(customTreeSelectionListener);
      }
   }

   private void disableNodeCollapse()
   {

      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
            cloudMachineTrees.get(machine).first().setToggleClickCount(0);
      }
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

      DefaultComboBoxModel model = new DefaultComboBoxModel();

      radioGroup = new ButtonGroup();
      useDefaultButton = new JRadioButton("Use ROS Synchronization Layer", true);
      useDefaultButton.setActionCommand("default");
      usePluginButton = new JRadioButton("Use Jesper Pl\u00FCgin Synchronization");
      usePluginButton.setActionCommand("plugin");

      radioGroup.add(useDefaultButton);
      radioGroup.add(usePluginButton);

      useDefaultButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            ((DefaultComboBoxModel) taskCombo.getModel()).removeAllElements();
            for (DRCROSTasks task : DRCROSTasks.values())
            {
               ((DefaultComboBoxModel) taskCombo.getModel()).addElement(task);
            }
         }
      });

      usePluginButton.addActionListener(new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {
            ((DefaultComboBoxModel) taskCombo.getModel()).removeAllElements();
            for (DRCPluginTasks task : DRCPluginTasks.values())
            {
               ((DefaultComboBoxModel) taskCombo.getModel()).addElement(task);
            }
         }
      });

      if (radioGroup.getSelection().getActionCommand().contains("plugin"))
      {
         for (DRCPluginTasks task : DRCPluginTasks.values())
         {
            model.addElement(task);
         }
      }
      else
      {
         for (DRCROSTasks task : DRCROSTasks.values())
         {
            model.addElement(task);
         }
      }

      taskCombo = new JComboBox(model);
      taskPanel.add(taskCombo, c);

      c.weightx = 0;
      c.gridx = 1;
      c.gridy = 0;
      taskPanel.add(useDefaultButton, c);

      c.gridy = 1;
      taskPanel.add(usePluginButton, c);
   }

   private void setupLeftContentPanel()
   {
      operatorUICheckBox = new JCheckBox("Launch Operator UI With Gazebo");
      scsCheckBox = new JCheckBox("Launch SCS (Demo01) With Gazebo");
      c.gridx = 0;
      c.gridy = 0;
      machineSelectionPanel.add(operatorUICheckBox, c);
      c.gridy++;
      machineSelectionPanel.add(scsCheckBox, c);

      setupSelectControllerMachine();

      setupSelectGazeboMachine();

      setupCloudMachineInfoPanel();
   }

   private void setupSelectControllerMachine()
   {
      c.gridx = 0;
      c.gridy = 2;
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
      c.gridy = 4;
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
      c.gridy = 6;
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

      frame.toFront();
      taskCombo.requestFocus();
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

   private void startTimers()
   {
      Timer redrawTimer = new Timer(10, new ActionListener()
      {
         public void actionPerformed(ActionEvent e)
         {

         }
      });

      java.util.Timer updateNetStatusTimer = new java.util.Timer();

      redrawTimer.start();
      updateNetStatusTimer.schedule(new TimerTask()
      {
         @Override
         public void run()
         {
            updateNetworkStatus();
         }
      }, 0l, 5000l);
   }

   private void updateNetworkStatus()
   {
      updateStatusIndicators();
      updateRemoteProcessListings();

      SwingUtilities.invokeLater(new Runnable()
      {
         public void run()
         {
            ((JPanel) networkStatusScrollPane.getViewport().getView()).repaint();
         }
      });
   }

   private void updateRemoteProcessListings()
   {
      updateRosSimStatuses();

      updateSCSStatuses();

      recalculateNodeSizes();
   }

   private void recalculateNodeSizes()
   {
      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
         {
            DefaultMutableTreeNode root = cloudMachineTrees.get(machine).second();
            ((DefaultTreeModel) cloudMachineTrees.get(machine).first().getModel()).nodeChanged(root);
            ((DefaultTreeModel) cloudMachineTrees.get(machine).first().getModel()).nodeChanged(root.getChildAt(0));
            ((DefaultTreeModel) cloudMachineTrees.get(machine).first().getModel()).nodeChanged(root.getChildAt(1));
         }
      }
   }

   private void updateRosSimStatuses()
   {
      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
         {
            DefaultMutableTreeNode node = (DefaultMutableTreeNode) cloudMachineTrees.get(machine).second().getChildAt(0);

            if (sshSimLauncher.isMachineRunningSim(machine))
            {
               node.setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:red;font-style:italic;\">" + sshSimLauncher.getSimTaskname(machine)
                     + "</span></body></html>");
            }
            else
            {
               node.setUserObject("<html><body>Running ROS/GZ Sim: <span style=\"color:green;font-style:italic;\">No</span></body></html>");
            }
         }
      }
   }

   private void updateSCSStatuses()
   {
      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
         {
            DefaultMutableTreeNode node = (DefaultMutableTreeNode) cloudMachineTrees.get(machine).second().getChildAt(1);

            if (sshSimLauncher.isMachineRunningController(machine))
            {
               node.setUserObject("<html><body>Running SCS Controller? <span style=\"color:red;font-style:italic;\">Yes</span></body></html>");
            }
            else
            {
               node.setUserObject("<html><body>Running SCS Controller? <span style=\"color:green;font-style:italic;\">No</span></body></html>");
            }
         }
      }
   }

   private void updateStatusIndicators()
   {
      for (LocalCloudMachines machine : LocalCloudMachines.values())
      {
         if (!machine.equals(LocalCloudMachines.LOCALHOST))
         {
            if (sshSimLauncher.isMachineReachable(machine))
            {
               setCloudStatusItemIcon(cloudMachineTrees.get(machine).first(), goodConnectionIcon);
               cloudMachineTrees.get(machine).first().expandRow(0);
            }
            else
            {
               setCloudStatusItemIcon(cloudMachineTrees.get(machine).first(), badConnectionIcon);
               cloudMachineTrees.get(machine).first().collapseRow(0);
            }
         }
      }
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
