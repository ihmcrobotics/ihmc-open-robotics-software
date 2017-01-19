package us.ihmc.simulationconstructionset.simulationDispatcher.client.gui;

import java.awt.BorderLayout;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.FlowLayout;
import java.awt.GraphicsDevice;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.Action;
import javax.swing.BoxLayout;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JPanel;
import javax.swing.JScrollPane;
import javax.swing.JToolBar;
import javax.swing.border.TitledBorder;

import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.Simulation;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.simulationconstructionset.gui.SelectedVariableHolder;
import us.ihmc.simulationconstructionset.gui.ViewportPanel;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariableListPanel;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelJPopupMenu;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.DispatchHostList;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationDispatcher;
import us.ihmc.simulationconstructionset.simulationDispatcher.client.SimulationToDispatch;

public class SimulationDispatcherGUI
{
   private static final boolean DEBUG = true;

   // private ArrayList exitActionListeners = new ArrayList();
   private Simulation simulation;

   private JFrame frame;
   private Container contentPane;

   private JPanel simContentPane, buttonPanel, hostContentPane;

   private YoVariableListPanel varPanel;
   private SelectedVariableHolder selectedVariableHolder = new SelectedVariableHolder();

   private DoneSimsPanel doneSimsPanel;
   private WaitingSimsPanel waitingSimsPanel;
   private DispatchHostPanel dispatchHostPanel, deadDispatchHostPanel;

   private final PanelUpdater panelUpdater = new PanelUpdater();

   private ViewportPanel viewportPanel;

   private int SLIDER_WIDTH = 26;

   // protected JPanel buttonPanel;
   // protected EntryBoxArrayPanel myEntryBoxArrayPanel;
   // protected GraphArrayPanel myGraphArrayPanel;

   private DispatchHostList dispatchHostList;
   private SimulationDispatcher dispatcher;

   private final ArrayList<GraphicsRobot> graphicsRobotsToUpdate = new ArrayList<GraphicsRobot>();

   private String password;

   public SimulationDispatcherGUI(SimulationDispatcher dispatcher, DispatchHostList dispatchHostList, String password)
   {
      this.password = password;
      this.dispatchHostList = dispatchHostList;
      this.dispatcher = dispatcher;

      // System.out.println("Starting GUI");
      frame = new JFrame("Simulation Dispatcher");


      frame.addWindowListener(new WindowAdapter()
      {
         @Override
         public void windowClosing(WindowEvent e)
         {
            fileExit_actionPerformed();

            // System.exit(0);
         }
      });


      contentPane = frame.getContentPane();
      contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.Y_AXIS));    // new GridLayout(3,1));

      simContentPane = new JPanel(new BorderLayout());

      varPanel = new YoVariableListPanel("new panel", selectedVariableHolder, new YoVariablePanelJPopupMenu(selectedVariableHolder));



      // simContentPane.add("West", varPanel); //+++JEP
      // simContentPane.add("Center",canvas3D);

      JScrollPane varPanelScrollPanel = new JScrollPane(varPanel, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      varPanelScrollPanel.getVerticalScrollBar().setUnitIncrement(12);
      varPanelScrollPanel.setBorder(new TitledBorder("Variables"));
      varPanelScrollPanel.setPreferredSize(new Dimension(varPanel.getPreferredSize().width + SLIDER_WIDTH, 200));
      varPanelScrollPanel.setMaximumSize(new Dimension(varPanel.getPreferredSize().width + SLIDER_WIDTH, 200));

      // lowerHostContentPane.add(dispatchHostScrollPanel);
      // hostContentPane.add("West", deadDispatchHostScrollPanel);
      simContentPane.add("West", varPanelScrollPanel);


      hostContentPane = new JPanel();    // new BorderLayout());
      hostContentPane.setLayout(new BoxLayout(hostContentPane, BoxLayout.X_AXIS));

      deadDispatchHostPanel = new DeadDispatchHostPanel(dispatchHostList);
      dispatchHostList.addHostsChangedListener(deadDispatchHostPanel);
      dispatchHostPanel = new AliveDispatchHostPanel(dispatchHostList);
      dispatchHostList.addHostsChangedListener(dispatchHostPanel);
      waitingSimsPanel = new WaitingSimsPanel(dispatcher);
      dispatcher.addSimulationsChangedListener(waitingSimsPanel);
      doneSimsPanel = new DoneSimsPanel(dispatcher);
      dispatcher.addSimulationsChangedListener(doneSimsPanel);


      deadDispatchHostPanel.hostsChanged();
      dispatchHostPanel.hostsChanged();

      buttonPanel = new JPanel(new FlowLayout());
      createActions();

      // hostContentPane.add("North", buttonPanel);



      // JPanel lowerHostContentPane = new JPanel(new FlowLayout());
      // hostContentPane.add("Center", lowerHostContentPane);



      JScrollPane deadDispatchHostScrollPanel = new JScrollPane(deadDispatchHostPanel, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                                   JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      deadDispatchHostScrollPanel.getVerticalScrollBar().setUnitIncrement(12);
      deadDispatchHostScrollPanel.setBorder(new TitledBorder("Dead Hosts"));
      deadDispatchHostScrollPanel.setPreferredSize(new Dimension(deadDispatchHostPanel.getPreferredSize().width + SLIDER_WIDTH, 400));
      deadDispatchHostScrollPanel.setMaximumSize(new Dimension(deadDispatchHostPanel.getPreferredSize().width + SLIDER_WIDTH, 600));

      // lowerHostContentPane.add(dispatchHostScrollPanel);
      // hostContentPane.add("West", deadDispatchHostScrollPanel);
      hostContentPane.add(deadDispatchHostScrollPanel);

      JScrollPane dispatchHostScrollPanel = new JScrollPane(dispatchHostPanel, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS,
                                               JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      dispatchHostScrollPanel.getVerticalScrollBar().setUnitIncrement(12);
      dispatchHostScrollPanel.setBorder(new TitledBorder("Dispatch Hosts"));
      dispatchHostScrollPanel.setPreferredSize(new Dimension(dispatchHostPanel.getPreferredSize().width + SLIDER_WIDTH, 400));
      dispatchHostScrollPanel.setMaximumSize(new Dimension(dispatchHostPanel.getPreferredSize().width + SLIDER_WIDTH, 600));

      // lowerHostContentPane.add(dispatchHostScrollPanel);
      // hostContentPane.add("Center", dispatchHostScrollPanel);
      hostContentPane.add(dispatchHostScrollPanel);

      JScrollPane waitingSimsScrollPanel = new JScrollPane(waitingSimsPanel, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      waitingSimsScrollPanel.getVerticalScrollBar().setUnitIncrement(12);
      waitingSimsScrollPanel.setBorder(new TitledBorder("Waiting Sims"));
      waitingSimsScrollPanel.setPreferredSize(new Dimension(waitingSimsPanel.getPreferredSize().width + SLIDER_WIDTH, 400));
      waitingSimsScrollPanel.setMaximumSize(new Dimension(waitingSimsPanel.getMaximumSize().width + SLIDER_WIDTH, 600));

      // lowerHostContentPane.add(doneSimsScrollPanel);
      hostContentPane.add(waitingSimsScrollPanel);

      // hostContentPane.add("East", doneSimsScrollPanel);

      JScrollPane doneSimsScrollPanel = new JScrollPane(doneSimsPanel, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      doneSimsScrollPanel.getVerticalScrollBar().setUnitIncrement(12);
      doneSimsScrollPanel.setBorder(new TitledBorder("Done Sims"));
      doneSimsScrollPanel.setPreferredSize(new Dimension(doneSimsPanel.getPreferredSize().width + SLIDER_WIDTH, 400));
      doneSimsScrollPanel.setMaximumSize(new Dimension(doneSimsPanel.getMaximumSize().width + SLIDER_WIDTH, 600));

      // lowerHostContentPane.add(doneSimsScrollPanel);
      hostContentPane.add(doneSimsScrollPanel);

      // hostContentPane.add("East", doneSimsScrollPanel);


      contentPane.add(simContentPane);
      contentPane.add(hostContentPane);

      /*
       * JPanel boxPanel = new JPanel();
       * boxPanel.setLayout(new GridLayout(2,1));
       * boxPanel.add(simContentPane);
       * boxPanel.add(hostContentPane);
       *
       * contentPane.add(boxPanel);
       */


      /*
       * JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT);
       * splitPane.setContinuousLayout(true);
       *
       * splitPane.setTopComponent(simContentPane);
       * splitPane.setBottomComponent(hostContentPane);
       *
       * splitPane.setOneTouchExpandable(true);
       *
       * contentPane.add(splitPane);
       */


      contentPane.add(buttonPanel);

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
      frame.setSize(screenSize.width * 7 / 8, screenSize.height * 6 / 8);

      // frame.setSize(screenSize.width*1/8, screenSize.height*1/8);
      frame.setLocation(screenSize.width / 16, screenSize.height / 16);

      // frame.setResizable(false);

      frame.validate();

      // frame.pack();
      frame.setVisible(true);

      // frame.show();
   }

   private ArrayList<ShowSelectedAction> guiButtons, guiActions;

   private void createActions()
   {
      guiButtons = new ArrayList<ShowSelectedAction>();
      guiActions = new ArrayList<ShowSelectedAction>();

      ShowSelectedAction showSelectedAction = new ShowSelectedAction(dispatchHostPanel, this, this.password);
      guiActions.add(showSelectedAction);

      JMenuItem exitMenuItem = new JMenuItem("Exit");
      exitMenuItem.addActionListener(new ActionListener()
      {
         @Override
         public void actionPerformed(ActionEvent e)
         {
            fileExit_actionPerformed();
         }
      });

      Action[] showActions = new Action[] {showSelectedAction};

      Action[][] allActions = new Action[][]
      {
         showActions
      };
      JToolBar[] toolBars = new JToolBar[allActions.length];

      // System.out.println(allActions.length);

      for (int j = 0; j < allActions.length; j++)
      {
         Action[] acts = allActions[j];

         toolBars[j] = new JToolBar();
         toolBars[j].setBorderPainted(true);    // false);
         toolBars[j].setFloatable(false);

         for (int i = 0; i < acts.length; i++)
         {
            Action act = acts[i];
            if (act.getValue(Action.SMALL_ICON) != null)
            {
               JButton button = toolBars[j].add(act);
               button.setToolTipText((String) act.getValue(Action.NAME));
            }
         }
      }



      for (int i = 0; i < toolBars.length; i++)
      {
         buttonPanel.add(toolBars[i]);
      }

      // numericContentPane.add("North",buttonPanel);

      // Menu:
      // URL iconURL;

      // Need to do the following since menus are lightweight and the Canvas3D is heavyweight:
      // JPopupMenu.setDefaultLightWeightPopupEnabled(false);
      JMenuBar menuBar = new JMenuBar();
      frame.setJMenuBar(menuBar);


//    File Menu:

      JMenu fileMenu = new JMenu("File");
      fileMenu.setMnemonic('f');
      menuBar.add(fileMenu);

      fileMenu.add(exitMenuItem);

      // Show Menu:

      JMenu showMenu = new JMenu("Show");
      showMenu.setMnemonic('s');
      menuBar.add(showMenu);

      showMenu.add(showSelectedAction);

   }

   protected void fileExit_actionPerformed()
   {
      /*
       * ListIterator iter = exitActionListeners.listIterator();
       * while(iter.hasNext())
       * {
       * ExitActionListener listener = (ExitActionListener) iter.next();
       * listener.exitActionPerformed();
       * }
       */

      System.exit(0);
   }

   public synchronized void setSimulation(SimulationToDispatch dispatchSim)
   {
      if (simulation == dispatchSim.getSimulation())
      {
//         printIfDebug("Sim didn't change");
         return;
      }

      // System.out.println("Sim changed!!!");

      simulation = dispatchSim.getSimulation();
      if (simulation == null)
         return;

      simContentPane.removeAll();
      Graphics3DAdapter graphics = simulation.getSimulationGraphics();


      if (graphics == null)
      {
         printIfDebug("Creating new simulation graphics");

         for (Robot robot : simulation.getRobots())
         {
            graphicsRobotsToUpdate.add(new GraphicsRobot(robot));
         }

         simulation.setupSimulationGraphics(graphicsRobotsToUpdate);
         graphics = simulation.getSimulationGraphics();
      }


      // Viewport:
      GraphicsDevice device = null;
      if (frame != null)
         device = frame.getGraphicsConfiguration().getDevice();

      /*
       * ViewportPanel viewportPanel = new ViewportPanel(sim.getRobot(), standardGUIActions,
       *                                               sim.getCombinedVarList(),
       *                                               cameraConfigurationList,
       *                                               cameraMountList,
       *                                               navigatingCameraHolder);
       */


      //TODO: This seems to be broken. If we switch sims, the graphics show for one frame but don't update.
      printIfDebug("Creating new viewportPanel");
      viewportPanel = new ViewportPanel(null, null, null, null, null, graphics);

      viewportPanel.setupViews(device, null);

      CameraConfiguration view1 = new CameraConfiguration("view1");

      // view1.setCameraDolly(true, true, true, false);
      view1.setCameraTracking(true, true, true, false);
      view1.setCameraPosition(0.0, -9.6, 1.2);
      view1.setCameraFix(0.0, 0.0741, 1.0976);


      viewportPanel.setCameraConfiguration(view1, simulation);

      // viewportPanel.setCameraTracking(true, true, true, false);

      simContentPane.add("Center", viewportPanel);

      ////////////////////////

      varPanel = new YoVariableListPanel(dispatchSim.getDescription(), selectedVariableHolder, new YoVariablePanelJPopupMenu(selectedVariableHolder));

      String[] vars = dispatchSim.getOutputStateVariableNames();
      for (int i = 0; i < vars.length; i++)
      {
         YoVariable<?> var = simulation.getVariable(vars[i]);
         if (var != null)
            varPanel.addVariable(var);
      }

      // simContentPane.add("West", varPanel); //+++JEP
      // simContentPane.add("Center",canvas3D);

      JScrollPane varPanelScrollPanel = new JScrollPane(varPanel, JScrollPane.VERTICAL_SCROLLBAR_ALWAYS, JScrollPane.HORIZONTAL_SCROLLBAR_AS_NEEDED);
      varPanelScrollPanel.getVerticalScrollBar().setUnitIncrement(12);
      varPanelScrollPanel.setBorder(new TitledBorder(dispatchSim.getDescription()));
      varPanelScrollPanel.setPreferredSize(new Dimension(varPanel.getPreferredSize().width + SLIDER_WIDTH, 400));

      panelUpdater.setPanelToUpdate(varPanel);

      // varPanelScrollPanel.setMaximumSize(new Dimension(varPanel.getPreferredSize().width+SLIDER_WIDTH, 200));

      // lowerHostContentPane.add(dispatchHostScrollPanel);
      // hostContentPane.add("West", deadDispatchHostScrollPanel);
      simContentPane.add("West", varPanelScrollPanel);

      simContentPane.setPreferredSize(new Dimension(600, 400));

      simContentPane.updateUI();
      simContentPane.repaint();
   }

   private void printIfDebug(String message)
   {
      if (DEBUG)
         System.out.println(message);
   }

   public synchronized void updateRobot()
   {
      if (viewportPanel != null)
      {
         for (Robot robot : simulation.getRobots())
         {
            robot.update();
         }

         for (GraphicsRobot graphicsRobot : graphicsRobotsToUpdate)
         {
            graphicsRobot.update();
         }

      }
   }

   private static class PanelUpdater
   {
      private Timer alertChangeListenersTimer;
      private TimerTask alertChangeListenersTask;

      private JPanel panelToUpdate;

      public PanelUpdater()
      {
         createAndStartPeriodicUIUpdateThread();
      }

      public void setPanelToUpdate(JPanel panelToUpdate)
      {
         this.panelToUpdate = panelToUpdate;
      }

      private void createAndStartPeriodicUIUpdateThread()
      {
         alertChangeListenersTimer = new Timer("CombinedVarPanelTimer");
         alertChangeListenersTask = new TimerTask()
         {
            @Override
            public void run()
            {
               if (panelToUpdate != null)
               {
                  panelToUpdate.repaint();
               }
            }
         };
         alertChangeListenersTimer.schedule(alertChangeListenersTask, 1000, 250);
      }
   }
}
