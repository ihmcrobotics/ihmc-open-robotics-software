package us.ihmc.avatar.simulationStarter;

import java.awt.BorderLayout;
import java.awt.Dialog.ModalExclusionType;
import java.awt.GridLayout;
import java.awt.KeyEventDispatcher;
import java.awt.KeyboardFocusManager;
import java.awt.event.ActionEvent;
import java.awt.event.KeyEvent;
import java.io.FileInputStream;
import java.io.FileOutputStream;
import java.io.IOException;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Properties;

import javax.swing.AbstractAction;
import javax.swing.BoxLayout;
import javax.swing.ImageIcon;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.SwingUtilities;
import javax.swing.event.ChangeEvent;
import javax.swing.event.ChangeListener;

import us.ihmc.avatar.DRCStartingLocation;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.robotEnvironmentAwareness.LidarBasedREAStandaloneLauncher;
import us.ihmc.robotEnvironmentAwareness.RemoteLidarBasedREAUILauncher;
import us.ihmc.tools.processManagement.JavaProcessSpawner;

public abstract class DRCSimulationTools
{
   private static final String STARTING_LOCATION_PROPERTY_NAME = "startingLocation";

   @SuppressWarnings({"hiding", "unchecked"})
   public static <T extends DRCStartingLocation, Enum> void startSimulationWithGraphicSelector(SimulationStarterInterface simulationStarter,
                                                                                               Class<?> operatorInterfaceClass, String[] operatorInterfaceArgs,
                                                                                               T... possibleStartingLocations)
   {
      List<Modules> modulesToStart = new ArrayList<Modules>();
      DRCStartingLocation startingLocation = null;

      startingLocation = showSelectorWithStartingLocation(modulesToStart, possibleStartingLocations);

      if (startingLocation != null)
         simulationStarter.setStartingLocation(startingLocation);

      if (modulesToStart.isEmpty())
         return;

      boolean automaticallyStartSimulation = true;
      DRCNetworkModuleParameters networkProcessorParameters;
      if (modulesToStart.contains(Modules.NETWORK_PROCESSOR))
      {
         networkProcessorParameters = new DRCNetworkModuleParameters();
         networkProcessorParameters.enableUiModule(true);
         networkProcessorParameters.enableBehaviorModule(modulesToStart.contains(Modules.BEHAVIOR_MODULE));
         networkProcessorParameters.enableBehaviorVisualizer(modulesToStart.contains(Modules.BEHAVIOR_MODULE));
         networkProcessorParameters.enableSensorModule(modulesToStart.contains(Modules.SENSOR_MODULE));
         networkProcessorParameters.enableZeroPoseRobotConfigurationPublisherModule(modulesToStart.contains(Modules.ZERO_POSE_PRODUCER));
         networkProcessorParameters.enablePerceptionModule(true);
         networkProcessorParameters.setEnableJoystickBasedStepping(true);
         networkProcessorParameters.enableRosModule(modulesToStart.contains(Modules.ROS_MODULE));
         networkProcessorParameters.enableLocalControllerCommunicator(false);
         networkProcessorParameters.enableKinematicsToolbox(modulesToStart.contains(Modules.KINEMATICS_TOOLBOX));
         networkProcessorParameters.enableKinematicsStreamingToolbox(modulesToStart.contains(Modules.KINEMATICS_TOOLBOX));
         networkProcessorParameters.enableFootstepPlanningToolbox(modulesToStart.contains(Modules.FOOTSTEP_PLANNING_TOOLBOX));
         networkProcessorParameters.enableWholeBodyTrajectoryToolbox(modulesToStart.contains(Modules.WHOLE_BODY_TRAJECTORY_TOOLBOX));
         networkProcessorParameters.enableKinematicsPlanningToolbox(true);
         boolean startREAModule = modulesToStart.contains(Modules.REA_MODULE) && !modulesToStart.contains(Modules.REA_UI);
         networkProcessorParameters.enableRobotEnvironmentAwerenessModule(startREAModule);
         networkProcessorParameters.enableBipedalSupportPlanarRegionPublisher(modulesToStart.contains(Modules.SENSOR_MODULE));
         networkProcessorParameters.enableMocapModule(modulesToStart.contains(Modules.MOCAP_MODULE));
      }
      else
      {
         networkProcessorParameters = null;
      }

      if (modulesToStart.contains(Modules.SIMULATION))
         simulationStarter.startSimulation(networkProcessorParameters, automaticallyStartSimulation);

      if (modulesToStart.contains(Modules.OPERATOR_INTERFACE))
      {
         if (modulesToStart.contains(Modules.SIMULATION))
            startOpertorInterfaceUsingProcessSpawner(operatorInterfaceClass, operatorInterfaceArgs);
         else
            startOpertorInterface(operatorInterfaceClass, operatorInterfaceArgs);
      }

      if (modulesToStart.contains(Modules.BEHAVIOR_VISUALIZER))
         simulationStarter.startBehaviorVisualizer();

      boolean startREAModule = modulesToStart.contains(Modules.REA_MODULE);
      boolean startREAUI = modulesToStart.contains(Modules.REA_UI);

      if (startREAModule && startREAUI)
         new JavaProcessSpawner(true, true).spawn(LidarBasedREAStandaloneLauncher.class);
      else if (startREAUI)
         new JavaProcessSpawner(true, true).spawn(RemoteLidarBasedREAUILauncher.class);
   }

   @SuppressWarnings({"hiding", "unchecked", "rawtypes", "serial"})
   private static <T extends DRCStartingLocation, Enum> DRCStartingLocation showSelectorWithStartingLocation(List<Modules> modulesToStartListToPack,
                                                                                                             T... possibleStartingLocations)
   {
      JPanel userPromptPanel = new JPanel(new BorderLayout());
      JPanel checkBoxesPanel = new JPanel(new GridLayout(4, 3));

      String configFile = System.getProperty("user.home") + "/.ihmc/drcSimulationDefaultOptions.config";
      Properties properties = new Properties();
      try
      {
         FileInputStream lastConfigInputStream = new FileInputStream(configFile);
         properties.load(lastConfigInputStream);
         lastConfigInputStream.close();
      }
      catch (IOException e)
      {
         // No config file, whatever.
      }

      JLabel userMessageLabel = new JLabel("Select which modules to start:");
      final EnumMap<Modules, JCheckBox> moduleCheckBoxes = new EnumMap<>(Modules.class);

      for (Modules module : Modules.values())
      {
         boolean enabled;
         if (module.isAlwaysEnabled()) // So Simulation, operator and spectator interfaces, and behavior visualizer can never get disabled
            enabled = true;
         else
            enabled = Boolean.parseBoolean(properties.getProperty(module.getPropertyNameForEnable(), Boolean.toString(module.getDefaultValueForEnable())));
         boolean selected = Boolean.parseBoolean(properties.getProperty(module.getPropertyNameForSelected(),
                                                                        Boolean.toString(module.getDefaultValueForSelected())));
         JCheckBox checkBox = new JCheckBox(module.getName());
         checkBox.setSelected(selected);
         checkBox.setEnabled(enabled);
         checkBoxesPanel.add(checkBox);
         moduleCheckBoxes.put(module, checkBox);
      }

      ChangeListener networkProcessorCheckBoxChangeListener = new ChangeListener()
      {
         @Override
         public void stateChanged(ChangeEvent e)
         {
            boolean isNetworkProcessorSelected = moduleCheckBoxes.get(Modules.NETWORK_PROCESSOR).isSelected();
            boolean isNetworkProcessorEnabled = moduleCheckBoxes.get(Modules.NETWORK_PROCESSOR).isEnabled();
            moduleCheckBoxes.get(Modules.BEHAVIOR_MODULE).setEnabled(isNetworkProcessorSelected && isNetworkProcessorEnabled);
            moduleCheckBoxes.get(Modules.SENSOR_MODULE).setEnabled(isNetworkProcessorSelected && isNetworkProcessorEnabled);
            moduleCheckBoxes.get(Modules.ZERO_POSE_PRODUCER).setEnabled(isNetworkProcessorSelected && isNetworkProcessorEnabled);
            moduleCheckBoxes.get(Modules.ROS_MODULE).setEnabled(isNetworkProcessorSelected && isNetworkProcessorEnabled);
            moduleCheckBoxes.get(Modules.REA_MODULE).setEnabled(isNetworkProcessorSelected && isNetworkProcessorEnabled);
            moduleCheckBoxes.get(Modules.REA_UI).setEnabled(isNetworkProcessorSelected && isNetworkProcessorEnabled);
         }
      };

      ChangeListener simulationCheckBoxChangeListener = new ChangeListener()
      {
         @Override
         public void stateChanged(ChangeEvent e)
         {
            boolean isSimulationSelected = moduleCheckBoxes.get(Modules.SIMULATION).isSelected();
            moduleCheckBoxes.get(Modules.NETWORK_PROCESSOR).setEnabled(isSimulationSelected);
         }
      };
      moduleCheckBoxes.get(Modules.NETWORK_PROCESSOR).addChangeListener(networkProcessorCheckBoxChangeListener);
      moduleCheckBoxes.get(Modules.SIMULATION).addChangeListener(simulationCheckBoxChangeListener);

      // Call the listeners
      networkProcessorCheckBoxChangeListener.stateChanged(null);
      simulationCheckBoxChangeListener.stateChanged(null);

      JComboBox obstacleCourseStartingLocationComboBox = null;

      if (possibleStartingLocations != null && possibleStartingLocations.length > 0)
      {
         JPanel comboBoxPanel = new JPanel(new BorderLayout());
         HashMap<JPanel, JComboBox> comboBoxPanelsMap = new HashMap<JPanel, JComboBox>();
         JLabel selectObstacleCourseLocationLabel = new JLabel("Select a Starting Location: ");
         JPanel locationPanel = new JPanel();
         locationPanel.setLayout(new BoxLayout(locationPanel, BoxLayout.PAGE_AXIS));

         final JPanel obstacleCourseLocationPanel = new JPanel(new BorderLayout());
         obstacleCourseLocationPanel.setVisible(true);
         obstacleCourseStartingLocationComboBox = new JComboBox(possibleStartingLocations);

         Map<String, T> possibleStartingLocationMap = new HashMap<>();
         for (T possibleStartingLocation : possibleStartingLocations)
         {
            possibleStartingLocationMap.put(possibleStartingLocation.toString(), possibleStartingLocation);
         }

         T selectedStartingLocation = possibleStartingLocationMap.get(properties.getProperty(STARTING_LOCATION_PROPERTY_NAME,
                                                                                             possibleStartingLocations[0].toString()));

         obstacleCourseStartingLocationComboBox.setSelectedItem(selectedStartingLocation == null ? possibleStartingLocations[0] : selectedStartingLocation);
         comboBoxPanelsMap.put(obstacleCourseLocationPanel, obstacleCourseStartingLocationComboBox);

         obstacleCourseLocationPanel.add(selectObstacleCourseLocationLabel, BorderLayout.WEST);
         obstacleCourseLocationPanel.add(obstacleCourseStartingLocationComboBox, BorderLayout.EAST);
         obstacleCourseLocationPanel.setVisible(true);
         locationPanel.add(obstacleCourseLocationPanel);
         comboBoxPanel.add(locationPanel, BorderLayout.CENTER);
         userPromptPanel.add(comboBoxPanel, BorderLayout.NORTH);
      }

      userPromptPanel.add(userMessageLabel, BorderLayout.CENTER);
      userPromptPanel.add(checkBoxesPanel, BorderLayout.SOUTH);

      final JFrame frame = new JFrame("Launch");
      frame.setIconImage(new ImageIcon(DRCSimulationTools.class.getClassLoader().getResource("running-man-32x32-Launch.png")).getImage());
      frame.setLayout(new BorderLayout());
      frame.add(userPromptPanel, BorderLayout.CENTER);
      JPanel optionPanel = new JPanel();
      final JButton okayButton = new JButton(new AbstractAction("Okay")
      {
         @Override
         public void actionPerformed(ActionEvent arg0)
         {
            frame.dispose();
            frame.setEnabled(false);
         }
      });
      optionPanel.add(okayButton);
      final JButton cancelButton = new JButton(new AbstractAction("Cancel")
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            System.exit(-1);
         }
      });
      optionPanel.add(cancelButton);

      KeyboardFocusManager.getCurrentKeyboardFocusManager().addKeyEventDispatcher(new KeyEventDispatcher()
      {
         @Override
         public boolean dispatchKeyEvent(KeyEvent e)
         {
            if (frame.isVisible() && e.getID() == KeyEvent.KEY_TYPED)
            {
               if (e.getKeyChar() == KeyEvent.VK_ENTER)
                  okayButton.doClick();
               else if (e.getKeyChar() == KeyEvent.VK_ESCAPE)
                  cancelButton.doClick();
               return true;
            }
            return false;
         }
      });

      frame.add(optionPanel, BorderLayout.SOUTH);
      frame.pack();
      frame.setModalExclusionType(ModalExclusionType.APPLICATION_EXCLUDE);
      frame.setLocationRelativeTo(null);
      SwingUtilities.invokeLater(new Runnable()
      {
         @Override
         public void run()
         {
            frame.setVisible(true);
         }
      });

      while (frame.isEnabled())
         ThreadTools.sleep(50);

      //      int selectedOption = JOptionPane.showOptionDialog(null, userPromptPanel, "Select", JOptionPane.OK_CANCEL_OPTION, JOptionPane.QUESTION_MESSAGE, null,
      //            null, null);
      //      if (selectedOption != JOptionPane.OK_OPTION)
      //      {
      //         System.exit(-1);
      //      }

      properties = new Properties();
      for (Modules module : Modules.values())
      {
         boolean selected = moduleCheckBoxes.get(module).isSelected();
         boolean enabled = moduleCheckBoxes.get(module).isEnabled();
         if (selected && enabled)
            modulesToStartListToPack.add(module);

         properties.setProperty(module.getPropertyNameForEnable(), String.valueOf(enabled));
         properties.setProperty(module.getPropertyNameForSelected(), String.valueOf(selected));
      }

      if (obstacleCourseStartingLocationComboBox != null && obstacleCourseStartingLocationComboBox.getSelectedItem() != null)
      {
         properties.setProperty(STARTING_LOCATION_PROPERTY_NAME, obstacleCourseStartingLocationComboBox.getSelectedItem().toString());
      }

      FileOutputStream newConfigOutputStream;
      try
      {
         newConfigOutputStream = new FileOutputStream(configFile);
         properties.store(newConfigOutputStream, "Default configuration for the graphic selector of the DRCSimulationTools");
         newConfigOutputStream.close();
      }
      catch (IOException e)
      {
      }

      if (obstacleCourseStartingLocationComboBox == null)
         return null;
      else
         return (DRCStartingLocation) obstacleCourseStartingLocationComboBox.getSelectedItem();
   }

   /**
    * Creates and starts the operator interface. The operator interface needs the simulation and
    * network processor to work properly, if started before any of these it will simply hang and
    * wait for these two to start. Use {@link #spawnOperatorInterfaceInDifferentProcess} to either
    * start the operator interface in the same process or a different one. Note that if started in a
    * different process the debug mode will not work.
    */
   public static void startOpertorInterfaceUsingProcessSpawner(Class<?> operatorInterfaceClass, String[] operatorInterfaceArgs)
   {
      JavaProcessSpawner spawner = new JavaProcessSpawner(true, true);
      if (operatorInterfaceClass == null)
         return;
      spawner.spawn(operatorInterfaceClass, operatorInterfaceArgs);

   }

   public static void startOpertorInterface(Class<?> operatorInterfaceClass, String[] operatorInterfaceArgs)
   {
      if (operatorInterfaceClass == null)
         return;

      try
      {
         Method mainMethod = operatorInterfaceClass.getDeclaredMethod("main", String[].class);
         Object args[] = {operatorInterfaceArgs};
         mainMethod.invoke(null, args);
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         e.printStackTrace();
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         e.printStackTrace();
      }
   }

   public enum Modules
   {
      SIMULATION,
      OPERATOR_INTERFACE,
      BEHAVIOR_VISUALIZER,
      NETWORK_PROCESSOR,
      SENSOR_MODULE,
      ROS_MODULE,
      BEHAVIOR_MODULE,
      ZERO_POSE_PRODUCER,
      REA_MODULE,
      REA_UI,
      MOCAP_MODULE,
      KINEMATICS_TOOLBOX,
      FOOTSTEP_PLANNING_TOOLBOX,
      WHOLE_BODY_TRAJECTORY_TOOLBOX;

      public String getPropertyNameForEnable()
      {
         return "enable" + FormattingTools.underscoredToCamelCase(toString(), true);
      }

      public String getPropertyNameForSelected()
      {
         return "select" + FormattingTools.underscoredToCamelCase(toString(), true);
      }

      public boolean isAlwaysEnabled()
      {
         if (this == SIMULATION || this == OPERATOR_INTERFACE || this == BEHAVIOR_VISUALIZER)
            return true;
         else
            return false;
      }

      public boolean getDefaultValueForEnable()
      {
         return true;
      }

      public boolean getDefaultValueForSelected()
      {
         if (this == SIMULATION || this == OPERATOR_INTERFACE || this == NETWORK_PROCESSOR || this == SENSOR_MODULE)
            return true;
         else
            return false;
      }

      public String getName()
      {
         return FormattingTools.underscoredToCamelCase(toString(), true);
      }
   }
}
