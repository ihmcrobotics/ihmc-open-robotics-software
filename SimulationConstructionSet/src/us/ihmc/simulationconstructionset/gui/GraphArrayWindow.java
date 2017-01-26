package us.ihmc.simulationconstructionset.gui;

import java.awt.Dimension;
import java.awt.Frame;
import java.awt.GraphicsConfiguration;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.Point;
import java.util.ArrayList;

import javax.swing.JFrame;
import javax.swing.JMenuBar;
import javax.swing.JPanel;

import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.commands.AllCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.SelectGraphConfigurationCommandExecutor;
import us.ihmc.simulationconstructionset.gui.config.Configuration;
import us.ihmc.simulationconstructionset.gui.config.ConfigurationList;
import us.ihmc.simulationconstructionset.gui.config.GraphConfigurationList;
import us.ihmc.simulationconstructionset.gui.config.GraphGroup;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupList;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupSelector;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportGraphsToFileConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.ExportGraphsToFileGenerator;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadGraphGroupDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.LoadGraphGroupDialogGenerator;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PrintGraphsDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.PrintGraphsDialogGenerator;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveGraphConfigurationDialogConstructor;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.SaveGraphConfigurationDialogGenerator;

public class GraphArrayWindow implements SelectGraphConfigurationCommandExecutor, GraphGroupSelector
{
   protected final GraphArrayPanel myGraphArrayPanel;
   private final JFrame frame;
   private final String name;

   private final ConfigurationList configurationList;
   private final GraphGroupList graphGroupList;
   private final GraphConfigurationList graphConfigurationList;

   private StandardGUIActions windowGUIActions;

   public GraphArrayWindow(AllCommandsExecutor allCommandsExecutor, SimulationConstructionSet sim, GUIEnablerAndDisabler guiEnablerAndDisabler, ConfigurationList configurationList,
                           GraphGroupList graphGroupList, String graphGroupName, GraphConfigurationList graphConfigurationList,
                           SelectedVariableHolder selectedVariableHolder, DataBuffer dataBuffer, StandardGUIActions mainGUIActions, int screenID, Point windowLocation,
                           Dimension windowSize,
                           boolean maximizeWindow)
   {
      this.configurationList = configurationList;
      this.graphGroupList = graphGroupList;
      this.graphConfigurationList = graphConfigurationList;

      GraphicsConfiguration configurationToUse = null;
      GraphicsEnvironment graphicsEnvironment = GraphicsEnvironment.getLocalGraphicsEnvironment();
      GraphicsDevice[] devices = graphicsEnvironment.getScreenDevices();
      for (int j = 0; j < devices.length; j++)
      {
         // GraphicsDevice graphicsDevice = devices[j];
         if (devices[j].toString().indexOf("screen=" + screenID) >= 0)
            configurationToUse = devices[j].getDefaultConfiguration();
      }

      String windowName;
      
      if (graphGroupName != null) 
      {
         windowName = "Graph Window: " + graphGroupName;
         this.name = graphGroupName;
      }
      else 
      {
         windowName = "Graph Window";
         this.name = "Unnamed";
      }
      
      frame = new JFrame(windowName, configurationToUse);
      frame.setName(windowName);
      myGraphArrayPanel = new GraphArrayPanel(selectedVariableHolder, dataBuffer, frame);

      windowGUIActions = new StandardGUIActions();

      SaveGraphConfigurationDialogConstructor saveGraphConfigurationDialogConstructor = new SaveGraphConfigurationDialogGenerator(guiEnablerAndDisabler, frame,
                                                                                           myGraphArrayPanel);
      
     LoadGraphGroupDialogConstructor loadGraphGroupDialogConstructor = new LoadGraphGroupDialogGenerator(sim, sim.getStandardSimulationGUI(), this, frame, myGraphArrayPanel);
      
      PrintGraphsDialogConstructor printGraphsDialogConstructor = new PrintGraphsDialogGenerator(myGraphArrayPanel);

      ExportGraphsToFileConstructor exportGraphsToFileConstructor = new ExportGraphsToFileGenerator(sim, frame, myGraphArrayPanel, sim.getStandardSimulationGUI());
      
      windowGUIActions.createGraphWindowActions(mainGUIActions, myGraphArrayPanel, saveGraphConfigurationDialogConstructor,loadGraphGroupDialogConstructor, printGraphsDialogConstructor, exportGraphsToFileConstructor);
      JPanel buttonPanel = windowGUIActions.createGraphWindowButtons();
      JMenuBar menuBar = windowGUIActions.createGraphWindowMenus();

      frame.setJMenuBar(menuBar);

      frame.getContentPane().add(buttonPanel, "South");

      JPanel graphArrayAndButtonPanel = new JPanel(new java.awt.BorderLayout());
      graphArrayAndButtonPanel.add("Center", myGraphArrayPanel);

      JPanel graphButtonPanel = myGraphArrayPanel.createGraphButtonPanel();
      graphArrayAndButtonPanel.add("South", graphButtonPanel);

      frame.getContentPane().add(graphArrayAndButtonPanel);

      this.selectGraphGroup(graphGroupName);
      
      if (windowLocation != null) frame.setLocation(windowLocation);
      
      frame.pack();

      if (maximizeWindow)
      {
         frame.setExtendedState(Frame.MAXIMIZED_BOTH);
      }
      else if (windowSize != null) frame.setSize(windowSize);
         

      frame.setVisible(true);
   }
   
   public int getScreenID()
   {
      String idString = frame.getGraphicsConfiguration().getDevice().getIDstring();
      return Integer.parseInt(idString.substring(idString.length()-1, idString.length()));
   }
   
   public Dimension getWindowSize()
   {
      return frame.getSize();
   }
   
   public Point getWindowLocationOnScreen()
   {
      Point locationOnScreen = frame.getLocationOnScreen();
      return locationOnScreen;
   }


   public String getName()
   {
      return name;
   }
   
   public JFrame getJFrame()
   {
      return frame;
   }
   
   public StandardGUIActions getGUIActions()
   {
      return this.windowGUIActions;
   }

   public void updateGraphs()
   {
      myGraphArrayPanel.repaintGraphs();
   }

   public boolean isVisable()
   {
      return frame.isVisible();
   }

   public void closeWindow()
   {
      frame.dispose();
   }

   public void repaint()
   {
      myGraphArrayPanel.repaint();
   }

   public GraphArrayPanel getGraphArrayPanel()
   {
      return myGraphArrayPanel;
   }

   public boolean isPainting()
   {
      return myGraphArrayPanel.isPaintingPanel();
   }


   public void updateGUI()
   {
      myGraphArrayPanel.repaint(); //updateUI();

   }


   /*
    * private void setupConfigurationMenu()
    * {
    * configurationMenu.removeAll();
    * String[] names = configurationList.getConfigurationNames();
    *
    * for (int i=0; i<names.length; i++)
    * {
    *   configurationMenu.add(new SelectConfigurationAction(this, i, names[i]));
    * }
    *
    * //configurationMenu.addSeparator();
    * //configurationMenu.add(varGroupsMenu);
    * //configurationMenu.add(graphGroupsMenu);
    * //configurationMenu.add(entryBoxGroupsMenu);
    * }
    */



   @SuppressWarnings("unused")
   private String selectedConfigurationName, selectedGraphGroupName;

   @Override
   public void selectGraphConfiguration(String name)
   {
      // if (rob == null) return;

      selectedConfigurationName = name;

      Configuration config = configurationList.getConfiguration(name);
      if (config == null)
         return;

      // selectVarGroup(config.getVarGroupName());
      selectGraphGroup(config.getGraphGroupName());

      // selectEntryBoxGroup(config.getEntryBoxGroupName());
   }

   @Override
   public void selectGraphGroup(String name)
   {
      selectedGraphGroupName = name;
      if (myGraphArrayPanel == null)
         return;

      GraphGroup group = graphGroupList.getGraphGroup(name);
      if (group == null)
         return;

      myGraphArrayPanel.removeAllGraphs();
      myGraphArrayPanel.setNumColumns(group.getNumColumns());

      ArrayList<String[][]> graphVars = group.getGraphVars();
      for (int i = 0; i < graphVars.size(); i++)
      {
         this.setupGraph(graphVars.get(i));
      }
   }

   public void setupGraph(String[] varnames)
   {
      myGraphArrayPanel.setupGraph(varnames);
   }

   public void setupGraph(String[][] varnames)
   {
      if ((varnames.length > 1) && (varnames[1].length > 0))
         myGraphArrayPanel.setupGraph(varnames[0], graphConfigurationList.getGraphConfiguration(varnames[1][0]));
      else
         myGraphArrayPanel.setupGraph(varnames[0]);
   }


   public void zoomFullView()
   {
      this.myGraphArrayPanel.zoomFullView();
   }

   public boolean allowTickUpdatesNow()
   {
      return myGraphArrayPanel.allowTickUpdatesNow();
   }

   public void closeAndDispose()
   {
      if (windowGUIActions != null)
      {
         windowGUIActions.closeAndDispose();
         windowGUIActions = null;
      }

      frame.setMenuBar(null);
      frame.removeAll();

      frame.dispose();
   }

  
}
