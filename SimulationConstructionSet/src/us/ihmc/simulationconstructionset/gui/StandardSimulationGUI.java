package us.ihmc.simulationconstructionset.gui;

import java.awt.BorderLayout;
import java.awt.Component;
import java.awt.Container;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.GraphicsDevice;
import java.awt.GraphicsEnvironment;
import java.awt.GridLayout;
import java.awt.Point;
import java.awt.Toolkit;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.ComponentAdapter;
import java.awt.event.ComponentEvent;
import java.awt.event.ComponentListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.awt.event.WindowListener;
import java.beans.PropertyChangeEvent;
import java.beans.PropertyChangeListener;
import java.io.File;
import java.net.URL;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.StringTokenizer;
import java.util.concurrent.ConcurrentLinkedQueue;

import javax.swing.AbstractButton;
import javax.swing.JApplet;
import javax.swing.JButton;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JComponent;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenuBar;
import javax.swing.JPanel;
import javax.swing.JPopupMenu;
import javax.swing.JRadioButton;
import javax.swing.JSplitPane;
import javax.swing.JTextField;
import javax.swing.JWindow;
import javax.swing.SwingUtilities;
import javax.vecmath.Color3f;
import javax.vecmath.Tuple3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.graphInterfaces.SelectedVariableHolder;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DBackgroundScaleMode;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfigurationList;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraMountList;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraTrackingAndDollyPositionHolder;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.ClassicCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.OffscreenBufferVideoServer;
import us.ihmc.jMonkeyEngineToolkit.camera.RenderedSceneHandler;
import us.ihmc.jMonkeyEngineToolkit.camera.TrackingDollyCameraController;
import us.ihmc.jMonkeyEngineToolkit.camera.ViewportAdapter;
import us.ihmc.javaFXToolkit.graphing.JavaFX3DGraph;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.simulationconstructionset.DataBuffer;
import us.ihmc.simulationconstructionset.ExitActionListener;
import us.ihmc.simulationconstructionset.ExtraPanelConfiguration;
import us.ihmc.simulationconstructionset.GraphConfiguration;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.HeightMapFromGroundContactModel;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.TimeHolder;
import us.ihmc.simulationconstructionset.ViewportConfiguration;
import us.ihmc.simulationconstructionset.commands.AllCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.SelectGraphConfigurationCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ViewportSelectorCommandListener;
import us.ihmc.simulationconstructionset.graphics.GraphicsDynamicGraphicsObject;
import us.ihmc.simulationconstructionset.graphics.GraphicsRobot;
import us.ihmc.simulationconstructionset.gui.camera.CameraTrackAndDollyYoVariablesHolder;
import us.ihmc.simulationconstructionset.gui.config.CameraSelector;
import us.ihmc.simulationconstructionset.gui.config.Configuration;
import us.ihmc.simulationconstructionset.gui.config.ConfigurationList;
import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroup;
import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroupList;
import us.ihmc.simulationconstructionset.gui.config.EntryBoxGroupSelector;
import us.ihmc.simulationconstructionset.gui.config.ExtraPanelConfigurationList;
import us.ihmc.simulationconstructionset.gui.config.ExtraPanelSelector;
import us.ihmc.simulationconstructionset.gui.config.GraphConfigurationList;
import us.ihmc.simulationconstructionset.gui.config.GraphGroup;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupList;
import us.ihmc.simulationconstructionset.gui.config.GraphGroupSelector;
import us.ihmc.simulationconstructionset.gui.config.VarGroup;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.simulationconstructionset.gui.config.VarGroupSelector;
import us.ihmc.simulationconstructionset.gui.config.ViewportConfigurationList;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.AllDialogConstructorsHolder;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.StandardAllDialogConstructorsGenerator;
import us.ihmc.simulationconstructionset.gui.hierarchyTree.NameSpaceHierarchyTree;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariableListPanel;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariablePanelJPopupMenu;
import us.ihmc.simulationconstructionset.gui.yoVariableSearch.YoVariableSearchPanel;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;
import us.ihmc.simulationconstructionset.util.SimpleFileReader;
import us.ihmc.simulationconstructionset.util.SimpleFileWriter;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.gui.GraphicsUpdatable;
import us.ihmc.tools.io.xml.XMLReaderUtility;
import us.ihmc.tools.thread.CloseableAndDisposableRegistry;

public class StandardSimulationGUI implements SelectGraphConfigurationCommandExecutor, GraphGroupSelector, EntryBoxGroupSelector, CameraSelector,
      ViewportSelectorCommandExecutor, CameraHolder, ActiveCameraHolder, ActiveCanvas3DHolder, ExtraPanelSelector, VarGroupSelector, ExitActionListenerNotifier
{
   private static final boolean UPDATE_UI = false;
   private static final boolean DEBUG_CLOSE_AND_DISPOSE = false;
   private static JWindow splashWindow;
   private Graphics3DAdapter graphics3dAdapter;
   private ConcurrentLinkedQueue<GraphicsUpdatable> graphicsUpdatables = new ConcurrentLinkedQueue<GraphicsUpdatable>();
   private LinkedHashMap<Robot, GraphicsRobot> graphicsRobots = new LinkedHashMap<Robot, GraphicsRobot>();

   private ArrayList<ExitActionListener> exitActionListeners = new ArrayList<ExitActionListener>();
   private ConfigurationList configurationList = new ConfigurationList();
   private CameraMountList cameraMountList = new CameraMountList();
   private GraphGroupList graphGroupList = new GraphGroupList();

   private GraphConfigurationList graphConfigurationList = new GraphConfigurationList();
   private EntryBoxGroupList entryBoxGroupList = new EntryBoxGroupList();
   private CameraConfigurationList cameraConfigurationList = new CameraConfigurationList();
   private ExtraPanelConfigurationList extraPanelConfigurationList = new ExtraPanelConfigurationList();
   private int updateCounts = 0;
   private ViewportConfigurationList viewportConfigurationList = new ViewportConfigurationList();
   private ArrayList<GraphArrayWindow> graphArrayWindows = new ArrayList<GraphArrayWindow>();
   private ArrayList<ViewportWindow> viewportWindows = new ArrayList<ViewportWindow>();
   private boolean isViewportHidden = false;

   protected JPanel buttonPanel;
   private Container contentPane;
   private JSplitPane splitPane;
   private int dividerLocation;
   private ViewportSelectorCommandListener viewportSelectorCommandListener;

   private JFrame jFrame;
   private JApplet jApplet;
   private Container parentContainer;

   private Thread shutdownHook;

   // Menu Items we need here:
   private JMenuBar menuBar;
   private YoVariableExplorerTabbedPane yoVariableExplorerTabbedPane;
   private DataBuffer myDataBuffer;
   protected EntryBoxArrayTabbedPanel myEntryBoxArrayPanel;
   protected GraphArrayPanel myGraphArrayPanel;
   private JPanel numericContentPane;

   private Robot[] robots;

   private SelectedVariableHolder selectedVariableHolder;
   private SimulationConstructionSet sim;

   private final YoVariableHolder yoVariableHolder;

   private final GUIEnablerAndDisabler guiEnablerAndDisabler;

   private AllCommandsExecutor allCommandsExecutor;

   private ViewportWindow viewportWindow;
   private GraphArrayWindow graphArrayWindow;
   private final SimulationSynchronizer simulationSynchronizer;

   // private ArrayList guiActions;
   private StandardGUIActions standardGUIActions;
   private AllDialogConstructorsHolder allDialogConstructorsHolder;
   
   private VarGroupList varGroupList;
   private ViewportPanel viewportPanel;
   private TimeStepMouseWheelListener timeStepMouseWheelListener;
   private JPanel mainPanel;
   private JPanel mainPanelHolder;

   private String currentView = "Normal View";
   public Canvas3DPanel canvas; // TODO: This appears to never be used
   private JSplitPane jSplitPane;
   private boolean rePaintOnSetPoint = true;
   private int multiViewCanvas = 0;
   private ArrayList<Component> tempPanelsHolder = new ArrayList<Component>();
   private YoVariableRegistry rootRegistry;
   private String configFileName = "defaultRegistry.conf";

   private String registryFileEnding = "_RegistryConfiguration";

   private String guiConfigFileEnding = "GuiConfiguration.dat";

   private BookmarkedVariablesHolder bookmarkedVariablesHolder;

   private ArrayList<TickUpdateListener> tickUpdateListeners = new ArrayList<TickUpdateListener>();
   
   private CloseableAndDisposableRegistry closeableAndDisposableRegistry = new CloseableAndDisposableRegistry();
   
   private List<String> panelsSelectedEarly = new ArrayList<>();
   private boolean scsWindowOpened = false;
   
   public StandardSimulationGUI(Graphics3DAdapter graphics3dAdapter, SimulationSynchronizer simulationSynchronizer, AllCommandsExecutor allCommandsExecutor,
         AllDialogConstructorsHolder allDialogConstructorsHolder, SimulationConstructionSet sim, YoVariableHolder yoVariableHolder, Robot[] robots,
         DataBuffer buffer, VarGroupList varGroupList, JApplet jApplet, YoVariableRegistry rootRegistry)
   {
      this(graphics3dAdapter, simulationSynchronizer, allCommandsExecutor, allDialogConstructorsHolder, sim, yoVariableHolder, robots, buffer, varGroupList,
            null, jApplet, rootRegistry);
   }

   public StandardSimulationGUI(Graphics3DAdapter graphics3dAdapter, SimulationSynchronizer simulationSynchronizer, AllCommandsExecutor allCommandsExecutor,
         AllDialogConstructorsHolder allDialogConstructorsHolder, SimulationConstructionSet sim, YoVariableHolder yoVariableHolder, Robot[] robots,
         DataBuffer buffer, VarGroupList varGroupList, JFrame frame, YoVariableRegistry rootRegistry)
   {
      this(graphics3dAdapter, simulationSynchronizer, allCommandsExecutor, allDialogConstructorsHolder, sim, yoVariableHolder, robots, buffer, varGroupList,
            frame, null, rootRegistry);
   }

   public StandardSimulationGUI(Graphics3DAdapter graphics3dAdapter, SimulationSynchronizer simulationSynchronizer, AllCommandsExecutor allCommandsExecutor,
         AllDialogConstructorsHolder allDialogConstructorsHolder, SimulationConstructionSet sim, YoVariableHolder yoVariableHolder, Robot[] robots,
         DataBuffer buffer, VarGroupList varGroupList, JFrame frame, JApplet jApplet, YoVariableRegistry rootRegistry)
   {
      this.graphics3dAdapter = graphics3dAdapter;
      this.simulationSynchronizer = simulationSynchronizer;

      this.allCommandsExecutor = allCommandsExecutor;

      this.sim = sim;
      this.yoVariableHolder = yoVariableHolder;

      this.guiEnablerAndDisabler = sim;
      this.robots = robots;
      this.myDataBuffer = buffer;
      this.varGroupList = varGroupList;
      this.jFrame = frame;
      this.jApplet = jApplet;
      this.rootRegistry = rootRegistry;

      this.bookmarkedVariablesHolder = new BookmarkedVariablesHolder();

      if (frame != null)
      {
         parentContainer = frame;
      }
      else
      {
         parentContainer = jApplet;
      }

      // frame.addPropertyChangeListener(this);
      // frame.addWindowStateListener(this);
      // frame.addWindowListener(this);
      this.selectedVariableHolder = new SelectedVariableHolder();

      // graphArrayWindow = new GraphArrayWindow(selectedVariableHolder, myDataBuffer, new YoVariable[0]);
      if (robots != null)
      {
         for (Robot robot : robots)
         {
            robot.getCameraMountList(cameraMountList);
         }
      }
   }

   public ExtraPanelConfigurationList getExtraPanelConfigurationList()
   {
      return extraPanelConfigurationList;
   }

   public JFrame getFrame()
   {
      return this.jFrame;
   }

   public ViewportPanel getViewportPanel()
   {
      return viewportPanel;
   }

   public void setRobots(Robot[] robots)
   {
      if (this.robots != null)
      {
         throw new RuntimeException("robots != null. Can only setRobots once!");
      }

      this.robots = robots;

      HeightMap heightMap = null;

      // TODO: GroundProfile is just that of the first robot. Need to make it part of the sim or something...
      if ((robots != null) && (robots.length > 0))
      {
         GroundContactModel groundContactModel = robots[0].getGroundContactModel();
         heightMap = HeightMapFromGroundContactModel.getHeightMap(groundContactModel);
      }

      setup(heightMap);

      if (robots != null)
      {
         for (Robot robot : robots)
         {
            //TODO: Should use the graphics robots, not the robots for cameras...
            robot.getCameraMountList(cameraMountList);
         }
      }

      if (robots != null)
      {
         updateRobotsAndCamera();
      }
   }

   public GraphArrayWindow getGraphArrayWindow(String windowName)
   {
      if (graphArrayWindows == null)
         return null;

      for (int i = 0; i < graphArrayWindows.size(); i++)
      {
         GraphArrayWindow graphArrayWindow = graphArrayWindows.get(i);
         if (graphArrayWindow.getName().equals(windowName))
            return graphArrayWindow;
      }

      return null;
   }

   public GraphArrayWindow createNewGraphWindow(final String graphGroupName, final int screenID, final Point windowLocation, final Dimension windowSize,
         final boolean maximizeWindow)
   {
      if (graphArrayWindows == null)
      {
         graphArrayWindows = new ArrayList<GraphArrayWindow>();
      }

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            graphArrayWindow = new GraphArrayWindow(allCommandsExecutor, sim, guiEnablerAndDisabler, configurationList, graphGroupList, graphGroupName,
                  graphConfigurationList, selectedVariableHolder, myDataBuffer, standardGUIActions, screenID, windowLocation, windowSize, maximizeWindow);
         }
      });

      graphArrayWindows.add(graphArrayWindow);

      StandardGUIActions windowGUIActions = graphArrayWindow.getGUIActions();

      windowGUIActions.setupConfigurationMenu(configurationList, graphArrayWindow, allCommandsExecutor);
      GUIConfigurationSaveAndLoad guiConfigurationSaveAndLoad = new GUIConfigurationSaveAndLoad(guiEnablerAndDisabler, this);
      guiConfigurationSaveAndLoad.loadGraphConfigurationsInConfigurationMenu();

      windowGUIActions.setupGraphGroupsMenu(graphGroupList, graphArrayWindow);

      // windowGUIActions.setupEntryBoxGroupMenu(entryBoxGroupList, graphArrayWindow);

      return graphArrayWindow;
   }

   public ViewportWindow getViewportWindow(String windowName)
   {
      if (viewportWindows == null)
         return null;

      for (ViewportWindow viewportWindow : viewportWindows)
      {
         if (viewportWindow.getName().equals(windowName))
            return viewportWindow;
      }

      return null;
   }

   public ViewportWindow createNewViewportWindow(String viewportName, int screenID, boolean maximizeWindow)
   {
      return createNewViewportWindow(viewportName, screenID, maximizeWindow, null);
   }

   public ViewportWindow createNewViewportWindow(String viewportName, final int screenID, final boolean maximizeWindow, CameraConfiguration camConfig)
   {
      if (viewportWindows == null)
      {
         viewportWindows = new ArrayList<ViewportWindow>();
      }

      final YoVariableHolder yoVariableHolder = sim;
      final TimeHolder timeHolder = sim;

      final String selectedViewportName = viewportName;

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            viewportWindow = new ViewportWindow(allCommandsExecutor, yoVariableHolder, timeHolder, selectedViewportName, viewportConfigurationList,
                  cameraConfigurationList, cameraMountList, robots, varGroupList, myGraphArrayPanel, getStandardSimulationGUI(), graphics3dAdapter,
                  myDataBuffer, standardGUIActions, screenID, maximizeWindow, simulationSynchronizer);
         }
      });

      if (camConfig != null)
      {
         viewportWindow.getViewportPanel().setCameraConfiguration(camConfig, rootRegistry);
      }

      viewportWindows.add(viewportWindow);

      final StandardGUIActions viewportGUIActions = viewportWindow.getGUIActions();

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            viewportGUIActions.setupCameraMenu(cameraConfigurationList, viewportWindow.getViewportPanel());
            viewportGUIActions.setupViewportMenu(allCommandsExecutor, viewportConfigurationList, viewportWindow);
         }
      });

      // windowGUIActions.setupConfigurationMenu(configurationList, viewportWindow);
      // windowGUIActions.setupGraphGroupsMenu(graphGroupList, viewportWindow);
      return viewportWindow;
   }

   public void setupMultiViews(String viewportName, ViewportPanel viewport_Panel)
   {
      if (robots == null)
      {
         return;
      }

      ViewportConfiguration config = viewportConfigurationList.getViewportConfiguration(viewportName);

      if (config == null)
      {
         return;
      }

      viewport_Panel.setupViews(jFrame.getGraphicsConfiguration().getDevice(), config);

      makeCheckBoxesConsistentWithCamera();
      currentView = viewportName;

      // This is to prevent the viewports from not being drawing
      int width = sim.getJFrame().getWidth() + 1;
      int height = sim.getJFrame().getHeight() + 1;
      Dimension d = new Dimension(width, height);

      sim.setFrameSize(d);
      viewport_Panel.repaint();
   }

   public void setup(final HeightMap heightMap)
   {
      createGraphicsRobots();

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            initGUI(heightMap);
            showGUI();
         }
      });
   }

   public static void showSplashScreen()
   {
      SplashPanel splashPanel = new SplashPanel();
      splashWindow = splashPanel.showSplashScreen();

   }

   public static void disposeSplashWindow()
   {
      if (splashWindow != null)
      {
         splashWindow.dispose();
      }
   }

   private void showGUI()
   {
      GraphicsDevice gd = GraphicsEnvironment.getLocalGraphicsEnvironment().getDefaultScreenDevice();

      //   Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();

      int screenWidth = gd.getDisplayMode().getWidth();
      int screenHeight = gd.getDisplayMode().getHeight();

      int width = (int) (screenWidth * 7.0 / 8.0);
      int height = (int) (screenHeight * 7.0 / 8.0);

      parentContainer.setSize(width, height);

      // parentContainer.setSize(screenSize.width*1/8, screenSize.height*1/8);
      int x = width / 16;
      int y = height / 16;
      parentContainer.setLocation(x, y);
      parentContainer.validate();

      // parentContainer.pack();
      // parentContainer.setVisible(true);
   }

   private void initGUI(HeightMap heightMap)
   {
      if (heightMap == null)
         heightMap = new FlatGroundProfile();

      if (jFrame != null)
      {
         contentPane = jFrame.getContentPane();
      }
      else if (jApplet != null)
      {
         contentPane = jApplet.getContentPane();
      }

      contentPane.removeAll();

      if (robots != null)
      {
         myEntryBoxArrayPanel = new EntryBoxArrayTabbedPanel(parentContainer, selectedVariableHolder);

         yoVariableExplorerTabbedPane = new YoVariableExplorerTabbedPane(new YoVariableDoubleClickListener(myDataBuffer, jFrame), jFrame, bookmarkedVariablesHolder,
               selectedVariableHolder, null, sim, rootRegistry);
         YoVariableSearchPanel variableSearchPanel = new YoVariableSearchPanel(selectedVariableHolder, myDataBuffer, myGraphArrayPanel, myEntryBoxArrayPanel,
               bookmarkedVariablesHolder, yoVariableExplorerTabbedPane);
         yoVariableExplorerTabbedPane.addVariableSearchPanel(variableSearchPanel);

         myGraphArrayPanel = new GraphArrayPanel(selectedVariableHolder, myDataBuffer, jFrame, this);
      }

      if (jFrame != null)
      {
         jFrame.addWindowListener(new WindowAdapter()
         {
            @Override
            public void windowClosing(WindowEvent e)
            {
               if (standardGUIActions != null)
               {
                  saveDefaultGUIConfigurationFile();
               }
               saveRegistryConfigurations();

               if (sim.systemExitDisabled())
                  sim.closeAndDispose();
               else
                  System.exit(0);

               // This closes scs without the side effects of System.exit(0) (or should):
               //             System.out.println("Close SCS requested");

               //             System.out.println("SCS should be closed and disposed.");
               // System.exit closes the jvm and then closes other threads as well!
               // For example a thread controlling launch of multiple scs sessions would be closed as well
               // as the other scs sessions it was managing.
               // If a thread is supposed to be closed with scs, then it should be manually specified
               // For example with:  sim.attachExitActionListener(exitChecker);

               // TODO: Can't switch from System.exit(0) to closeAndDispose until following fixed:
               // Under some conditions (almost always), SCS doesn't finish closing properly (with closeAndDispose):
               // To reproduce:
               //
               // Start a Simulation, for example ValkyrieFlatGroundWalkingTrack
               // Start simulating
               // Search for a variable, for example 't'
               // Add the variable to a graph
               // Close SCS using the window close button
               //
               // It will hang there due to a deadlock on getting the AWTTreeLock.
               //
               // To probably fix this, the following things need to happen:
               //
               // All Swing/GUI calls should use SwingUtilities.invokeLater() if not invoked from the main Swing event loop (loads of places in SCS)
               // The search interface needs to be revamped, making sure it uses InvokeLater when necessary
               // All other deadlocks in the SCS code need to be removed.

            }
            
            @Override
            public void windowOpened(WindowEvent e)
            {
               scsWindowOpened = true;
               for (String earlySelection : panelsSelectedEarly)
               {
                  selectPanel(earlySelection);
               }
            }
         });
         jFrame.addComponentListener(new ComponentAdapter()
         {
            @Override
            public void componentResized(ComponentEvent e)
            {
               frameResized();
            }
         });

         if (shutdownHook == null)
         {
            if (DEBUG_CLOSE_AND_DISPOSE)
               System.out.println("Registering shutdown hook.");
            shutdownHook = new Thread()
            {
               @Override
               public void run()
               {
                  notifyExitActionListeners();

                  //                  if (standardGUIActions != null)
                  //                  {
                  //                     saveDefaultGUIConfigurationFile();
                  //                  }

               }
            };
            Runtime.getRuntime().addShutdownHook(shutdownHook);
         }

         /*
          * &frame.addWindowStateListener(new WindowStateListener() { public
          * void windowStateChanged(WindowEvent e) {
          * System.out.println("Window State Changed " + e); } });
          */
      }

      // GridBagConstraints g = new GridBagConstraints(0, 0, 1, 1, 1, 1, GridBagConstraints.CENTER, GridBagConstraints.BOTH, new Insets(0, 0, 0, 0), 0, 0);
      if (robots != null)
      {
         contentPane.setLayout(new GridLayout(1, 1));

         // contentPane.setLayout(new GridBagLayout());
      }
      else
      {
         // +++JEP!! Java3D is crashing in native methods if I use the default layout or GridLayout(1,1) or BorderLayout(), or BoxLayout(contentPane,BoxLayout.Y_AXIS)
         // as the Layout Manager!!!
         // contentPane.setLayout(new GridLayout(2,1));  // Do this for now.  It leaves a gap on the bottom, but it works...
         contentPane.setLayout(new BorderLayout());

         // contentPane.setLayout(new GridBagLayout());
         // new BoxLayout(contentPane,BoxLayout.Y_AXIS));   //new FlowLayout()); //(new GridLayout(1,1));
      }

      // TODO: Pull J3D renderer out of SCS and rewrite it using the adapter

      //    HeightMapFromGroundProfile heightMap = new HeightMapFromGroundProfile(groundProfile);
      graphics3dAdapter.setHeightMap(heightMap);

      // JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT);
      // contentPane.add(splitPane);
      // 3D Canvass Stuff goes here...

      // GUI Actions:
      if (robots != null)
      {
         standardGUIActions = new StandardGUIActions();
         allDialogConstructorsHolder = new StandardAllDialogConstructorsGenerator(sim, robots, myDataBuffer, this, varGroupList, myGraphArrayPanel, this,
               parentContainer, jFrame, simulationSynchronizer, standardGUIActions);
      }

      viewportPanel = createViewportPanel();

      // +++ jjc added for wheel mouse time step control
      timeStepMouseWheelListener = new TimeStepMouseWheelListener(this);
      viewportPanel.addMouseWheelListener(timeStepMouseWheelListener);
      contentPane.addMouseWheelListener(timeStepMouseWheelListener);
      
      splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT);
      contentPane.add(splitPane);

      mainPanel = new JPanel(new BorderLayout());
      mainPanelHolder = new JPanel(new BorderLayout());

      mainPanel.add(viewportPanel, BorderLayout.CENTER);

      splitPane.setContinuousLayout(true);
      splitPane.setDividerSize(3);
      mainPanelHolder.add(mainPanel);
      splitPane.setTopComponent(mainPanelHolder);

      splitPane.setResizeWeight(1);
      dividerLocation = splitPane.getDividerLocation(); //

      // Numeric Content Panel:
      // contentPane.setLayout(new BorderLayout());
      if (robots != null)
      {
         numericContentPane = new JPanel(new BorderLayout());

         // if (rob!=null)
         // contentPane.add(numericContentPane);
         splitPane.setBottomComponent(numericContentPane);
      }

      Dimension screenSize = Toolkit.getDefaultToolkit().getScreenSize();
      splitPane.setDividerLocation((screenSize.height * 7 / 8) / 2);

      // splitPane.add(numericContentPane);
      // Menu and Buttons:
      if (robots != null)
      {
         standardGUIActions.createMainWindowActions(allCommandsExecutor, allDialogConstructorsHolder, viewportPanel);
         buttonPanel = standardGUIActions.createMainWindowButtons();
         menuBar = standardGUIActions.createMainWindowMenus(this);
         StandardViewSetup.setupStandardViews(this);

         // Grab the ones we need:
         // buttonPanel = standardGUIActions.buttonPanel;
         // menuBar = standardGUIActions.menuBar;
         // Need to do the following since menus are lightweight and the Canvas3D is heavyweight:
         JPopupMenu.setDefaultLightWeightPopupEnabled(false);

         if (jFrame != null)
         {
            jFrame.setJMenuBar(menuBar);
         }
         else
         {
            jApplet.setJMenuBar(menuBar);
         }
      }

      // Numeric Entry Boxes on the Bottom:
      if (robots != null)
      {
         JButton plusButton = new JButton("+");
         JButton minusButton = new JButton("-");

         plusButton.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {

               myEntryBoxArrayPanel.addEmptyTab();

               //
               //               String nextName = entryBoxGroupList.getNextGroupName(selectedEntryBoxGroupName);
               //
               //               selectEntryBoxGroup(nextName);
            }
         });
         minusButton.addActionListener(new ActionListener()
         {
            @Override
            public void actionPerformed(ActionEvent e)
            {

               myEntryBoxArrayPanel.getCurrentPanel().closeAndDispose();
               myEntryBoxArrayPanel.remove(myEntryBoxArrayPanel.getCurrentPanel());

            }
         });

         /*
          * JPanel entryBoxArrayPanelHolder = new JPanel();
          * entryBoxArrayPanelHolder.add(myEntryBoxArrayPanel);
          * numericContentPane.add("South",entryBoxArrayPanelHolder);
          */
         numericContentPane.add("North", buttonPanel);

         // numericContentPane.add("South", myEntryBoxArrayPanel);

         // numericContentPane.add("West", myCombinedVarPanel);

         JPanel graphArrayAndButtonPanel = new JPanel(new BorderLayout());

         graphArrayAndButtonPanel.add("Center", myGraphArrayPanel);

         JPanel graphButtonPanel = myGraphArrayPanel.createGraphButtonPanel();

         graphArrayAndButtonPanel.add("South", graphButtonPanel);

         JPanel entryBoxPanel = new JPanel(new BorderLayout());
         JPanel entryBoxControlPanel = new JPanel(new GridLayout(1, 2));
         entryBoxControlPanel.add(plusButton);
         entryBoxControlPanel.add(minusButton);

         entryBoxPanel.add(entryBoxControlPanel, BorderLayout.WEST);
         entryBoxPanel.add(myEntryBoxArrayPanel, BorderLayout.CENTER);

         JPanel graphArrayButtonAndEntryBoxPanel = new JPanel(new BorderLayout());
         graphArrayButtonAndEntryBoxPanel.add(graphArrayAndButtonPanel, BorderLayout.CENTER);
         graphArrayButtonAndEntryBoxPanel.add(entryBoxPanel, BorderLayout.SOUTH);

         JSplitPane jSplitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, true, yoVariableExplorerTabbedPane, graphArrayButtonAndEntryBoxPanel);
         jSplitPane.setDividerSize(3);
         jSplitPane.addPropertyChangeListener(JSplitPane.DIVIDER_LOCATION_PROPERTY, new PropertyChangeListener()
         {
            @Override
            public void propertyChange(PropertyChangeEvent dividerLocationPropertyChangeEvent)
            {
               yoVariableExplorerTabbedPane.getYoVariableSearchPanel().refreshSearchPanelWidth();
            }
         });

         numericContentPane.add("Center", jSplitPane);

         // numericContentPane.add("Center", graphArrayAndButtonPanel);

         // numericContentPane.add("Center",myGraphArrayPanel);

         YoVariablePanelJPopupMenu varPanelJPopupMenu = new YoVariablePanelJPopupMenu(myGraphArrayPanel, myEntryBoxArrayPanel, selectedVariableHolder, yoVariableExplorerTabbedPane,
               bookmarkedVariablesHolder);
         yoVariableExplorerTabbedPane.setVarPanelJPopupMenu(varPanelJPopupMenu);
      }
   }

   private void createGraphicsRobots()
   {
      if (robots != null)
      {
         for (Robot robot : robots)
         {
            GraphicsRobot graphicsRobot = new GraphicsRobot(robot);
            graphicsUpdatables.add(graphicsRobot);
            graphicsRobots.put(robot, graphicsRobot);
            graphics3dAdapter.addRootNode(graphicsRobot.getRootNode());
         }
      }
   }

   public ViewportPanel createViewportPanel()
   {
      // Viewport:
      GraphicsDevice device = null;

      if (jFrame != null)
      {
         device = jFrame.getGraphicsConfiguration().getDevice();
      }

      // else if (jApplet != null) device = jApplet.getGraphicsConfiguration().getDevice();
      ViewportPanel viewportPanel = new ViewportPanel(yoVariableHolder, allCommandsExecutor, standardGUIActions, cameraConfigurationList, cameraMountList, graphics3dAdapter);
      viewportPanel.setupViews(device, null, jFrame);

      return viewportPanel;
   }

   public YoVariableExplorerTabbedPane getCombinedVarPanel()
   {
      return this.yoVariableExplorerTabbedPane;
   }

   public void updateNameSpaceHierarchyTree()
   {
      NameSpaceHierarchyTree nameSpaceHierarchyTree = yoVariableExplorerTabbedPane.getNameSpaceHierarchyTree();
      nameSpaceHierarchyTree.createdNewRegistries();
   }

   public void attachExitActionListener(ExitActionListener listener)
   {
      this.exitActionListeners.add(listener);
   }

   public void addButton(AbstractButton button)
   {
      buttonPanel.add(button);
      buttonPanel.updateUI();
   }

   public void addComboBox(JComboBox<?> comboBox)
   {
      buttonPanel.add(comboBox);
      buttonPanel.updateUI();
   }

   public void addJLabel(JLabel label)
   {
      buttonPanel.add(label);
      buttonPanel.updateUI();
   }

   public void addTextField(JTextField textField)
   {
      buttonPanel.add(textField);
      buttonPanel.updateUI();
   }

   public void addRadioButton(JRadioButton button)
   {
      buttonPanel.add(button);
      buttonPanel.updateUI();
   }

   public void addCheckBox(JCheckBox checkBox)
   {
      buttonPanel.add(checkBox);
      buttonPanel.updateUI();
   }

   public void addMenuBar(JMenuBar menuBar)
   {
      // System.out.println("Adding MenuBar with " + menuBar.getSubElements().length + " elements to buttonPanel.");
      if (buttonPanel != null)
      {
         buttonPanel.add(menuBar);
         buttonPanel.updateUI();
      }
   }

   public void addVarList(final YoVariableList list)
   {
      if ((list != null) && (!list.isEmpty()))
      {
         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override
            public void run()
            {
               addVarListVarPanel(list);
            }
         });
      }
   }

   public void setCameraTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ)
   {
      if (robots == null)
      {
         return;
      }

      viewportPanel.setCameraTracking(track, trackX, trackY, trackZ);
      makeCheckBoxesConsistentWithCamera();
   }

   public void setCameraDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ)
   {
      if (robots == null)
      {
         return;
      }

      viewportPanel.setCameraDolly(dolly, dollyX, dollyY, dollyZ);
      makeCheckBoxesConsistentWithCamera();
   }

   public void makeCheckBoxesConsistentWithCamera()
   {
      standardGUIActions.makeCheckBoxesConsistentWithCamera();
   }

   public void makeCameraConsistentWithCheckBoxes()
   {
      standardGUIActions.makeCameraConsistentWithCheckBoxes();
   }

   public void disableGUIComponents()
   {
      if (standardGUIActions != null)
      {
         standardGUIActions.disableGUIComponents();
      }

      if (myGraphArrayPanel != null)
      {
         myGraphArrayPanel.setInteractionEnable(false);
      }
   }

   public void enableGUIComponents()
   {
      if (standardGUIActions != null)
      {
         standardGUIActions.enableGUIComponents();
      }

      if (myGraphArrayPanel != null)
      {
         myGraphArrayPanel.setInteractionEnable(true);
      }
   }

   public void notifySimulationStopped()
   {
      if (standardGUIActions != null)
         standardGUIActions.notifySimulationStopped();
   }

   public void addStaticLinkGraphics(ArrayList<Graphics3DObject> staticLinkGraphics)
   {
      for (Graphics3DObject graphics3dObject : staticLinkGraphics)
      {
         this.addStaticLinkGraphics(graphics3dObject);
      }
   }

   public Graphics3DNode addStaticLinkGraphics(Graphics3DObject staticLinkGraphics)
   {
      return addStaticLinkGraphics(staticLinkGraphics, Graphics3DNodeType.GROUND);
   }

   public Graphics3DNode addStaticLinkGraphics(Graphics3DObject staticLinkGraphics, Graphics3DNodeType nodeType)
   {
      Graphics3DNode staticGraphics3d = new Graphics3DNode("Static Link Graphic", nodeType);
      staticGraphics3d.setGraphicsObject(staticLinkGraphics);
      graphics3dAdapter.addRootNode(staticGraphics3d);

      return staticGraphics3d;
   }

   public void setupGraph(String varname)
   {
      if (robots == null)
      {
         return;
      }

      myGraphArrayPanel.setupGraph(varname);
   }

   public void setupGraph(String[] varnames)
   {
      myGraphArrayPanel.setupGraph(varnames);
   }

   public void setupGraph(String[][] varnames)
   {
      if ((varnames.length > 1) && (varnames[1].length > 0))
      {
         String graphConfiguration = varnames[1][0];

         myGraphArrayPanel.setupGraph(varnames[0], graphConfigurationList.getGraphConfiguration(graphConfiguration));
      }
      else
      {
         myGraphArrayPanel.setupGraph(varnames[0]);
      }
   }

   public void setClipDistances(double near, double far)
   {
      viewportPanel.setClipDistances(near, far);
   }

   public void setFieldOfView(double fieldOfView)
   {
      viewportPanel.setFieldOfView(fieldOfView);
   }

   public void setBackgroundColor(Color3f color)
   {
      graphics3dAdapter.setBackgroundColor(color);
   }

   public void setBackgroundImage(URL fileURL, Graphics3DBackgroundScaleMode backgroundScaleMode)
   {
      graphics3dAdapter.setBackgroundImage(fileURL, backgroundScaleMode);
   }

   public void setGroundAppearance(AppearanceDefinition app)
   {
      graphics3dAdapter.setGroundAppearance(app);
   }

   public void setGroundVisible(boolean isVisible)
   {
      graphics3dAdapter.setGroundVisible(isVisible);
   }

   public void updateGraphsLeisurely(int leisureRate)
   {
      if (!graphsReadyToUpdate())
      {
         return;
      }

      updateCounts++;

      if (updateCounts < leisureRate)
      {
         return;
      }

      updateCounts = 0;
      repaintGraphs();
   }

   public void updateGraphs()
   {
      if (graphsReadyToUpdate())
      {
         repaintGraphs();
      }
   }

   private void repaintGraphs()
   {
      myGraphArrayPanel.repaintGraphs();

      if (graphArrayWindows != null)
      {
         for (int i = 0; i < graphArrayWindows.size(); i++)
         {
            GraphArrayWindow graphArrayWindow = graphArrayWindows.get(i);

            graphArrayWindow.updateGraphs();
         }
      }
   }

   private boolean graphsReadyToUpdate()
   {
      // if (!mySimulationGraphics.getPreRenderFlag()) return false;
      // Only update the graphs if keeping up with frame rate of the cartoon.
      // If going faster, wait for the cartoon to keep it responsive.
      // Only update the graphs if none of them are currently painting:

      if (myGraphArrayPanel == null)
         return false;

      if (myGraphArrayPanel.isPaintingPanel())
      {
         return false;
      }

      if (graphArrayWindows != null)
      {
         for (int i = 0; i < graphArrayWindows.size(); i++)
         {
            GraphArrayWindow graphArrayWindow = graphArrayWindows.get(i);

            if (graphArrayWindow.isPainting())
            {
               return false;
            }
         }
      }

      // mySimulationGraphics.setPreRenderFlag(false);
      return true;
   }

   public void updateGUI()
   {
      if (myGraphArrayPanel == null)
         return;

      myGraphArrayPanel.repaint(); // .updateUI();

      for (int i = 0; i < this.graphArrayWindows.size(); i++)
      {
         GraphArrayWindow graphArrayWindow = graphArrayWindows.get(i);

         graphArrayWindow.updateGUI();
      }
   }

   public void show()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            if (jFrame != null)
            {
               jFrame.setVisible(true);
            }

            if (jApplet != null)
            {
               ((JPanel) jApplet.getContentPane()).updateUI();
            }
         }
      });
   }

   public void zoomIn()
   {
      myGraphArrayPanel.zoomIn();
   }

   public void zoomOut()
   {
      myGraphArrayPanel.zoomOut();
   }

   public void gotoInPoint()
   {
      // myCombinedVarPanel.goToInPoint();
      // this.graphArrayPanel.goToInPointLater();
      if (myGraphArrayPanel != null)
      {
         myGraphArrayPanel.setIndexLater(myDataBuffer.getInPoint());
      }

      // myGraphArrayPanel.updateUI();
   }

   public void gotoInPointNow()
   {
      // myCombinedVarPanel.goToInPoint();
      // this.graphArrayPanel.goToInPointLater();
      if (myGraphArrayPanel != null)
      {
         myGraphArrayPanel.goToInPointNow();
      }

      // myGraphArrayPanel.updateUI();
   }

   public void gotoOutPoint()
   {
      if (myGraphArrayPanel != null)
      {
         myGraphArrayPanel.setIndexLater(myDataBuffer.getOutPoint());
      }

      // myCombinedVarPanel.goToOutPoint();
      // myGraphArrayPanel.updateUI();
   }

   public void gotoOutPointNow()
   {
      if (myGraphArrayPanel != null)
      {
         myGraphArrayPanel.goToOutPointNow();
      }
   }

   public void setInPoint()
   {
      myDataBuffer.setInPoint();

      if (rePaintOnSetPoint)
      {
         myGraphArrayPanel.RepaintOnSetPoint();
      }

      repaintWindows();
   }

   public void setOutPoint()
   {
      myDataBuffer.setOutPoint();

      if (rePaintOnSetPoint)
      {
         myGraphArrayPanel.RepaintOnSetPoint();
      }

      repaintWindows();
   }

   public void setInOutPointFullBuffer()
   {
      myDataBuffer.setInOutPointFullBuffer();

      if (rePaintOnSetPoint)
      {
         myGraphArrayPanel.RepaintOnSetPoint();
      }

      repaintWindows();
   }

   /**
    * Gets the KeyPoints in the cropped data
    *
    * @return The current KeyPoints as an ArrayList of Integer
    */
   public ArrayList<Integer> getKeyPoints()
   {
      return myDataBuffer.getKeyPoints();
   }

   public void setKeyPoint()
   {
      myDataBuffer.setKeyPoint();
      repaintWindows();
   }

   public void toggleCameraKeyMode()
   {
      getCameraPropertiesForActiveCamera().toggleCameraKeyMode();
   }

   public void addCameraKey()
   {
      addCameraKey(getCamera());
   }

   private void addCameraKey(TrackingDollyCameraController j3dCameraController)
   {
      j3dCameraController.setCameraKeyPoint(myDataBuffer.getIndex());
      repaintWindows();
   }

   public ArrayList<Integer> getCameraKeyPoints()
   {
      ClassicCameraController classicCameraController = (ClassicCameraController) viewportPanel.getCamera();

      return classicCameraController.getCameraKeyPoints();
   }

   public void removeCameraKey()
   {
      removeCameraKey(getCameraPropertiesForActiveCamera());
   }

   private void removeCameraKey(TrackingDollyCameraController j3dCameraController)
   {
      j3dCameraController.removeCameraKeyPoint(myDataBuffer.getIndex());
      repaintWindows();
   }

   public void nextCameraKey()
   {
      nextCameraKey(getCamera());
   }

   private void nextCameraKey(TrackingDollyCameraController j3dCameraController)
   {
      j3dCameraController.nextCameraKeyPoint(myDataBuffer.getIndex());
      repaintWindows();
   }

   public void previousCameraKey()
   {
      previousCameraKey(getCamera());
   }

   private void previousCameraKey(TrackingDollyCameraController j3dCameraController)
   {
      j3dCameraController.previousCameraKeyPoint(myDataBuffer.getIndex());
      repaintWindows();
   }

   public void stepBackward()
   {
      if (myGraphArrayPanel != null) myGraphArrayPanel.tickLater(-1);
   }

   public void stepBackward(int ticks)
   {
      if (myGraphArrayPanel != null) myGraphArrayPanel.tickLater(-ticks);
   }

   public void stepForward()
   {
      if (myGraphArrayPanel != null) myGraphArrayPanel.tickLater(1);
   }

   public void stepForward(int ticks)
   {
      if (myGraphArrayPanel != null) myGraphArrayPanel.tickLater(ticks);
   }

   public void stepForwardNow(int ticks)
   {
      if (myGraphArrayPanel != null) myGraphArrayPanel.tick(ticks);
   }

   // public void zoomIn(){myGraphArrayPanel.zoomIn(2);}
   // public void zoomOut(){myGraphArrayPanel.zoomOut(2);}
   public void zoomFullView()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            if (myGraphArrayPanel != null) myGraphArrayPanel.zoomFullView();

            for (int i = 0; i < graphArrayWindows.size(); i++)
            {
               GraphArrayWindow graphArrayWindow = graphArrayWindows.get(i);

               graphArrayWindow.zoomFullView();
            }
         }
      });
   }

   public void repaintWindows()
   {
      if (myGraphArrayPanel != null) myGraphArrayPanel.repaint();

      for (int i = 0; i < graphArrayWindows.size(); i++)
      {
         GraphArrayWindow graphArrayWindow = graphArrayWindows.get(i);

         graphArrayWindow.repaint();
      }
   }

   public void setCameraTrackingVars(String xName, String yName, String zName)
   {
      DoubleYoVariable xVar, yVar, zVar;

      xVar = (DoubleYoVariable) rootRegistry.getVariable(xName);
      yVar = (DoubleYoVariable) rootRegistry.getVariable(yName);
      zVar = (DoubleYoVariable) rootRegistry.getVariable(zName);
      viewportPanel.setCameraTrackingVars(xVar, yVar, zVar);
   }

   public void setCameraDollyVars(String xName, String yName, String zName)
   {
      DoubleYoVariable xVar, yVar, zVar;

      xVar = (DoubleYoVariable) rootRegistry.getVariable(xName);
      yVar = (DoubleYoVariable) rootRegistry.getVariable(yName);
      zVar = (DoubleYoVariable) rootRegistry.getVariable(zName);
      viewportPanel.setCameraDollyVars(xVar, yVar, zVar);
   }

   public void setCameraTrackingOffsets(double dx, double dy, double dz)
   {
      viewportPanel.setCameraTrackingOffsets(dx, dy, dz);
   }

   public void setCameraDollyOffsets(double dx, double dy, double dz)
   {
      viewportPanel.setCameraDollyOffsets(dx, dy, dz);
   }

   public void setCameraFix(double fixX, double fixY, double fixZ)
   {
      viewportPanel.setCameraFix(fixX, fixY, fixZ);
   }

   public void setCameraFix(Tuple3d cameraFix)
   {
      viewportPanel.setCameraFix(cameraFix);
   }

   public void setCameraPosition(double posX, double posY, double posZ)
   {
      viewportPanel.setCameraPosition(posX, posY, posZ);
   }

   public void setCameraPosition(Tuple3d cameraPosition)
   {
      viewportPanel.setCameraPosition(cameraPosition);      
   }

   public boolean allowTickUpdatesNow()
   {
      boolean ret = false;

      ret = ret | myGraphArrayPanel.allowTickUpdatesNow();

      for (int i = 0; i < this.graphArrayWindows.size(); i++)
      {
         GraphArrayWindow graphArrayWindow = graphArrayWindows.get(i);

         ret = ret | graphArrayWindow.allowTickUpdatesNow();
      }

      TrackingDollyCameraController[] cameras = this.getCameras();
      for (int i = 0; i < cameras.length; i++)
      {
         if (cameras[i].useKeyCameraPoints())
            cameras[i].setKeyFrameTime(myDataBuffer.getIndex());
      }

      for (TickUpdateListener tickUpdateListener : tickUpdateListeners)
      {
         tickUpdateListener.update(myDataBuffer.getIndex());
      }

      return ret;
   }

   public void addTickUpdateListener(TickUpdateListener tickUpdateListener)
   {
      tickUpdateListeners.add(tickUpdateListener);
   }

   public void removeTickUpdateListener(TickUpdateListener tickUpdateListener)
   {
      tickUpdateListeners.remove(tickUpdateListener);
   }

   @Override
   public void notifyExitActionListeners()
   {
      for (int i = 0; i < exitActionListeners.size(); i++)
      {
         ExitActionListener listener = exitActionListeners.get(i); // iter.next();

         listener.exitActionPerformed();
      }
   }

   protected void frameResized()
   {
      if (myEntryBoxArrayPanel != null && myEntryBoxArrayPanel.getCurrentPanel() != null)
      {
         myEntryBoxArrayPanel.getCurrentPanel().updateRowsColumns();

         // myEntryBoxArrayPanel.repaint();
         myEntryBoxArrayPanel.updateUI(); // +++ JEP: This works, but not sure if this is the way you're supposed to do this...
      }
   }

   public void setupConfiguration(String name, String graphGroupName, String entryBoxGroupName)
   {
      if (robots == null)
      {
         return;
      }

      Configuration config = new Configuration(name);
      config.setGraphGroupName(graphGroupName);
      config.setEntryBoxGroupName(entryBoxGroupName);
      configurationList.addConfiguration(config);

      standardGUIActions.setupConfigurationMenu(configurationList, this, allCommandsExecutor);

      GUIConfigurationSaveAndLoad guiConfigurationSaveAndLoad = new GUIConfigurationSaveAndLoad(guiEnablerAndDisabler, this);
      guiConfigurationSaveAndLoad.loadGraphConfigurationsInConfigurationMenu();
   }

   public void updateVarGroupList(final VarGroupList varGroupList)
   {
      if (standardGUIActions != null)
      {
         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override
            public void run()
            {
               standardGUIActions.updateVarGroupList(varGroupList, getStandardSimulationGUI());
            }
         });
      }
   }

   private StandardSimulationGUI getStandardSimulationGUI()
   {
      return this;
   }

   public void setupGraphGroup(String name, String[][] vars)
   {
      setupGraphGroup(name, vars, 1);
   }

   public void setupGraphGroup(String name, String[][][] vars)
   {
      setupGraphGroup(name, vars, 1);
   }

   public void setupGraphGroup(String name, String[][] vars, int numColumns)
   {
      if (robots == null)
      {
         return;
      }

      GraphGroup group = new GraphGroup(name);

      group.addGraphVars(vars);
      group.setNumColumns(numColumns);
      graphGroupList.addGraphGroup(group);
      standardGUIActions.setupGraphGroupsMenu(graphGroupList, this);
   }

   public void setupGraphGroup(String name, String[][][] vars, int numColumns)
   {
      if (robots == null)
      {
         return;
      }

      GraphGroup group = new GraphGroup(name);

      group.addGraphVars(vars);
      group.setNumColumns(numColumns);
      boolean exists = false;
      for (String currentGroup : graphGroupList.getGraphGroupNames())
      {
         if (currentGroup.equals(name))
            exists = true;
      }

      if (!exists)
      {
         graphGroupList.addGraphGroup(group);
         standardGUIActions.setupGraphGroupsMenu(graphGroupList, this);
      }
   }

   public void setupEntryBoxGroup(String name, String[] vars)
   {
      setupEntryBoxGroup(name, vars, null);

      /*
       * if (rob == null) return; EntryBoxGroup group = new EntryBoxGroup(name);
       * group.addEntryBoxVars(vars);
       * 
       * entryBoxGroupList.addEntryBoxGroup(group);
       * 
       * standardGUIActions.setupEntryBoxGroupMenu(entryBoxGroupList, this);
       */
   }

   public void setupEntryBoxGroup(String name, String[] vars, String[] regularExpressions)
   {
      if (robots == null)
      {
         return;
      }

      if (entryBoxGroupList.getEntryBoxGroup(name) != null)
      {
         entryBoxGroupList.removeEntryBoxGroup(entryBoxGroupList.getEntryBoxGroup(name));
      }
      EntryBoxGroup group = new EntryBoxGroup(name);

      if (vars != null)
      {
         group.addEntryBoxVars(vars);
      }

      if (regularExpressions != null)
      {
         group.addEntryBoxRegularExpressions(regularExpressions);
      }

      entryBoxGroupList.addEntryBoxGroup(group);
      standardGUIActions.setupEntryBoxGroupMenu(entryBoxGroupList, this);
   }

   @Override
   public void selectGraphConfiguration(String name)
   {
      if (robots == null)
      {
         return;
      }

      Configuration config = configurationList.getConfiguration(name);

      if (config == null)
      {
         return;
      }

      // selectVarGroup(config.getVarGroupName());
      selectGraphGroup(config.getGraphGroupName());
      createNewEntryBoxTabFromEntryBoxGroup(config.getEntryBoxGroupName());
   }

   @Override
   public void selectVarGroup(String name)
   {
      if (robots == null)
      {
         return;
      }

      VarGroup group = varGroupList.getVarGroup(name);

      if (group == null)
      {
         return;
      }

      String[] varNames = group.getVars();
      String[] regularExpressions = group.getRegularExpressions();
      ArrayList<YoVariable<?>> matchedVariables = rootRegistry.getMatchingVariables(varNames, regularExpressions);
      YoVariableList varList = new YoVariableList(name);
      varList.addVariables(matchedVariables);

      addVarListVarPanel(varList);
   }

   private void addVarListVarPanel(YoVariableList varList)
   {
      if (yoVariableExplorerTabbedPane != null)
      {
         YoVariablePanelJPopupMenu varPanelJPopupMenu = new YoVariablePanelJPopupMenu(myGraphArrayPanel, myEntryBoxArrayPanel, selectedVariableHolder, yoVariableExplorerTabbedPane,
               bookmarkedVariablesHolder);
         YoVariableListPanel panel = new YoVariableListPanel(varList, selectedVariableHolder, varPanelJPopupMenu);
         yoVariableExplorerTabbedPane.addExtraVarPanel(panel);

         if (UPDATE_UI)
         {
            yoVariableExplorerTabbedPane.updateUI();
         }
      }
   }

   @Override
   public void selectGraphGroup(final String name)
   {
      if (myGraphArrayPanel == null)
      {
         return;
      }

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            GraphGroup group = graphGroupList.getGraphGroup(name);

            if (group == null)
            {
               return;
            }

            myGraphArrayPanel.removeAllGraphs();
            myGraphArrayPanel.setNumColumns(group.getNumColumns());

            ArrayList<String[][]> graphVars = group.getGraphVars();

            for (int i = 0; i < graphVars.size(); i++)
            {
               setupGraph(graphVars.get(i));
            }
         }
      });
   }

   public EntryBoxArrayPanel getEntryBoxArrayPanel()
   {
      return myEntryBoxArrayPanel.getCurrentPanel(true);
   }

   
   public void createNewEntryBoxTabFromEntryBoxGroup(final String name)
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            createNewEntryBoxTabFromEntryBoxGroupLocal(name);
         }
      });
   }
   
   
   public void createNewEntryBoxTabFromEntryBoxGroupLocal(String name)
   {
      //      
      if (myEntryBoxArrayPanel.getTabCount() > 0)
      {
         for (int i = 0; i < myEntryBoxArrayPanel.getTabCount(); i++)
         {
            if (myEntryBoxArrayPanel.getComponentAt(i) != null)
            {
               if (myEntryBoxArrayPanel.getComponentAt(i).getName().equals(name))
               {
                  myEntryBoxArrayPanel.removeTabAt(i);

                  break;
               }
            }
         }
      }
      //            
      if (myEntryBoxArrayPanel == null)
      {
         return;
      }

      EntryBoxGroup group = entryBoxGroupList.getEntryBoxGroup(name);

      if (group == null)
      {
         return;
      }

      String[] entryBoxVars = group.getEntryBoxVars();
      String[] entryBoxRegularExpressions = group.getEntryBoxRegularExpressions();
      ArrayList<YoVariable<?>> matchingVariables = rootRegistry.getMatchingVariables(entryBoxVars, entryBoxRegularExpressions);

      EntryBoxArrayPanel tmpEntryBoxArrayPanel = new EntryBoxArrayPanel(parentContainer, selectedVariableHolder, matchingVariables);
      tmpEntryBoxArrayPanel.setName(name);
      myEntryBoxArrayPanel.addEntryBoxArrayPanel(name, tmpEntryBoxArrayPanel);

      this.myEntryBoxArrayPanel.getCurrentPanel().checkStatus();
      numericContentPane.repaint(); // updateUI();
   }

   @Override
   public void selectEntryBoxGroup(String name)
   {
      if (myEntryBoxArrayPanel == null)
      {
         return;
      }

      EntryBoxGroup group = entryBoxGroupList.getEntryBoxGroup(name);

      if (group == null)
      {
         return;
      }

      String[] entryBoxVars = group.getEntryBoxVars();
      String[] entryBoxRegularExpressions = group.getEntryBoxRegularExpressions();
      ArrayList<YoVariable<?>> matchingVariables = rootRegistry.getMatchingVariables(entryBoxVars, entryBoxRegularExpressions);

      setupEntryBox(matchingVariables);

      /*
       * String[] entryBoxVars = group.getEntryBoxVars(); for(int i=0;
       * i<entryBoxVars.length; i++) { this.setupEntryBox(entryBoxVars[i]); }
       * 
       * String[] entryBoxRegularExpressions =
       * group.getEntryBoxRegularExpressions(); for(int i=0;
       * i<entryBoxRegularExpressions.length; i++) {
       * this.setupEntryBoxRegularExpression(entryBoxRegularExpressions[i]); }
       */
      this.myEntryBoxArrayPanel.getCurrentPanel().checkStatus();
      numericContentPane.repaint(); // updateUI();
   }

   public void setupEntryBox(String varname)
   {
      final YoVariable<?> v = rootRegistry.getVariable(varname);

      if (v != null)
      {
         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override
            public void run()
            {
               myEntryBoxArrayPanel.getCurrentPanel(true).addEntryBox(v);
            }
         });
      }

      // numericContentPane.updateUI();
   }

   public void setupEntryBox(final ArrayList<YoVariable<?>> variables)
   {
      if (variables == null)
      {
         return;
      }

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            for (int i = 0; i < variables.size(); i++)
            {
               YoVariable<?> v = variables.get(i);

               if (v != null)
               {
                  myEntryBoxArrayPanel.getCurrentPanel(true).addEntryBox(v);
               }
            }
         }
      });

      // numericContentPane.updateUI();
   }

   public void setupEntryBox(String[] varnames)
   {
      if (varnames == null)
      {
         return;
      }

      for (int i = 0; i < varnames.length; i++)
      {
         YoVariable<?> v = rootRegistry.getVariable(varnames[i]);

         if (v != null)
         {
            this.myEntryBoxArrayPanel.getCurrentPanel().addEntryBox(v);
         }
      }

      // numericContentPane.updateUI();
   }

   public void setupCamera(CameraConfiguration cameraConfiguration)
   {
      if (robots == null)
      {
         return;
      }

      cameraConfigurationList.addCameraConfiguration(cameraConfiguration);

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            standardGUIActions.setupCameraMenu(cameraConfigurationList, getStandardSimulationGUI());
         }
      });
   }

   public void setupExtraPanels(ExtraPanelConfiguration panelConfiguration)
   {
      if (robots == null)
      {
         return;
      }

      extraPanelConfigurationList.addExtraPanelConfiguration(panelConfiguration);

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            standardGUIActions.setupExtraPanelsMenu(extraPanelConfigurationList, getStandardSimulationGUI());
         }
      });
   }

   @Override
   public void selectCamera(String cameraName)
   {
      viewportPanel.selectCamera(cameraName);
      makeCheckBoxesConsistentWithCamera();
   }

   public void drawMainViewportWithExtraPanels()
   {
      if (mainPanel != null)
      {
         mainPanel.removeAll();
      }

      if (jSplitPane != null)
      {
         jSplitPane.removeAll();
      }

      if (tempPanelsHolder.size() == 0)
      {
         mainPanel.add(viewportPanel);
      }
      else
      {
         ArrayList<JSplitPane> dividers = new ArrayList<JSplitPane>();
         jSplitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, viewportPanel, tempPanelsHolder.get(0));
         dividers.add(jSplitPane);

         for (int i = 1; i < tempPanelsHolder.size(); i++)
         {
            jSplitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, jSplitPane, tempPanelsHolder.get(i));
            dividers.add(jSplitPane);
         }

         int currentPlaceOfLastDivider = 0;
         for (int j = 0; j < dividers.size(); j++)
         {
            dividers.get(j).setDividerLocation(currentPlaceOfLastDivider + mainPanel.getWidth() / ((dividers.size() + 1)));
            currentPlaceOfLastDivider += mainPanel.getWidth() / ((dividers.size() + 1));
         }

         mainPanel.add(jSplitPane);
      }

      // This is to prevent the viewports from not being drawing
      int width = sim.getJFrame().getWidth() + 1;
      int height = sim.getJFrame().getHeight() + 1;
      Dimension d = new Dimension(width, height);

      sim.setFrameSize(d);
      viewportPanel.repaint();
      mainPanel.updateUI();
   }

   public void addJComponentToMainPanel(JComponent comp, String index)
   {
      mainPanelHolder.add(comp, index);
   }

   public void addJComponentToNumericContentPane(JComponent comp, String index)
   {
      numericContentPane.add(comp, index);
   }

   @Override
   public void selectPanel(String panelName)
   {
      if (!scsWindowOpened)
      {
         panelsSelectedEarly.add(panelName);
         return;
      }
      
      for (int i = 0; i < standardGUIActions.extraPanelsMenu.getItemCount(); i++)
      {
         if (standardGUIActions.extraPanelsMenu.getItem(i).getText().equals(panelName))
         {
            makeCheckMarksConsistentForExtraPanels(panelName, standardGUIActions.extraPanelsMenu.getItem(i).isSelected());

            if (standardGUIActions.extraPanelsMenu.getItem(i).isSelected())
            {
               tempPanelsHolder.add(getExtraPanel(panelName));
               drawMainViewportWithExtraPanels();
            }
            else
            {
               boolean wasInThisWindow = true;
               if (!tempPanelsHolder.contains(getExtraPanel(panelName)))
               {
                  wasInThisWindow = false;
               }

               removeExtraPanel(panelName);

               if (!wasInThisWindow)
               {
                  tempPanelsHolder.add(getExtraPanel(panelName));
                  drawMainViewportWithExtraPanels();
               }
            }
         }
      }
   }

   @Override
   public TrackingDollyCameraController getCameraPropertiesForActiveCamera()
   {
      return viewportPanel.getCameraPropertiesForActiveCamera();
   }

   public ViewportAdapter getActiveView()
   {
      return viewportPanel.getActiveView();
   }

   @Override
   public TrackingDollyCameraController getCamera()
   {
      return viewportPanel.getCamera();
   }

   @Override
   public TrackingDollyCameraController[] getCameras()
   {
      return viewportPanel.getCameras();
   }

   @Override
   public CaptureDevice getActiveCaptureDevice()
   {
      return this.getActiveView().getCaptureDevice();
   }

   public void setupViewport(ViewportConfiguration viewportConfiguration)
   {
      if (robots == null)
      {
         return;
      }

      viewportConfigurationList.addViewportConfiguration(viewportConfiguration);
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            standardGUIActions.setupViewportMenu(allCommandsExecutor, viewportConfigurationList, getStandardSimulationGUI());
         }
      });
   }

   @Override
   public void selectViewport(final String viewportName)
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            selectViewportLocal(viewportName);
         }
      });
   }
   
   public void selectViewportLocal(String viewportName)
   {
      if (robots == null)
      {
         return;
      }

      ViewportConfiguration config = viewportConfigurationList.getViewportConfiguration(viewportName);

      if (config == null)
      {
         return;
      }

      GraphicsDevice graphicsDevice;
      if (jFrame != null)
      {
         graphicsDevice = jFrame.getGraphicsConfiguration().getDevice();
      }
      else
      {
         // graphicsDevice = jApplet.getGraphicsConfiguration().getDevice();
         graphicsDevice = null;
      }

      viewportPanel.setupViews(graphicsDevice, config);

      makeCheckBoxesConsistentWithCamera();
      currentView = viewportName;

      // This is to prevent the viewports from not being drawing
      int width = parentContainer.getWidth() + 1;
      int height = parentContainer.getHeight() + 1;
      Dimension d = new Dimension(width, height);

      parentContainer.setSize(d);
      viewportPanel.repaint();

      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            mainPanel.updateUI();
         }
      });

   }

   public void selectViewport_ViewPorts(String view)
   {
      int z = multiViewCanvas;
      if (robots == null)
      {
         return;
      }

      ViewportConfiguration config = viewportConfigurationList.getViewportConfiguration(view);

      if (config == null)
      {
         return;
      }

      viewportWindows.get(z).getViewportPanel()
            .setupViews(viewportWindows.get(z).getViewportPanel().getParent().getGraphicsConfiguration().getDevice(), config);

      makeCheckBoxesConsistentWithCamera();

      int width = viewportWindows.get(z).getViewportPanel().getParent().getWidth() + 1;
      int height = viewportWindows.get(z).getViewportPanel().getParent().getHeight() + 1;
      Dimension d = new Dimension(width, height);

      viewportWindows.get(z).getViewportPanel().getParent().setSize(d);
      viewportWindows.get(z).getViewportPanel().updateUI();

   }

   public void setupGraphConfigurations(GraphConfiguration[] configurations)
   {
      if (robots == null)
      {
         return;
      }

      for (int i = 0; i < configurations.length; i++)
      {
         graphConfigurationList.addGraphConfiguration(configurations[i]);
      }
   }

   public void updateRobots()
   {
      for (Robot robot : robots)
      {
         robot.updateForPlayback();
      }
   }

   public void updateRobotsAndCamera()
   {
      if (robots != null)
      {
         for (Robot robot : robots)
         {
            robot.update();
         }
      }

      updateSimulationGraphics();
   }

   public void maximizeMainWindow()
   {
      this.jFrame.setExtendedState(Frame.MAXIMIZED_BOTH);
   }

   public void setExportDataDirectory(String directory)
   {
      standardGUIActions.setExportDataDirectory(directory);
   }

   public void setImportDataDirectory(String directory)
   {
      standardGUIActions.setImportDataDirectory(directory);
   }

   /*
    * Note that this method is called on startup of SCS Therefore, if you want
    * to load a configuration on start up, you need to wait a few seconds until
    * SCS has started before calling this method with the desired GUI config.
    */
   public void loadGUIConfigurationFile(File file)
   {
      standardGUIActions.loadGUIConfigurationFile(file);
   }

   @Override
   public void hideViewport()
   {
      boolean isEventDispatchingThread = SwingUtilities.isEventDispatchThread();
      if (!isEventDispatchingThread)
      {
         System.err.println("hideViewport() called but not isEventDispatchingThread. This can cause threading issues!");
      }

      isViewportHidden = true;
      dividerLocation = splitPane.getDividerLocation();
      contentPane.removeAll();
      contentPane.setLayout(new GridLayout(1, 1));
      contentPane.add(numericContentPane);
      viewportSelectorCommandListener.updateViewportStatus();

      // contentPane.repaint();
      contentPane.validate();
   }

   @Override
   public void showViewport()
   {
      contentPane.removeAll();
      contentPane.setLayout(new GridLayout(1, 1));
      splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT);
      contentPane.add(splitPane);
      splitPane.setContinuousLayout(true);
      splitPane.setDividerSize(3);
      splitPane.setResizeWeight(1);
      splitPane.setTopComponent(mainPanel);
      splitPane.setBottomComponent(numericContentPane);
      splitPane.setDividerLocation(dividerLocation);

      // contentPane.setLayout(new GridLayout(2, 1));
      // contentPane.add(viewportPanel);
      // contentPane.add(numericContentPane);

      isViewportHidden = false;
      viewportSelectorCommandListener.updateViewportStatus();

      // contentPane.repaint();
      contentPane.validate();
   }

   @Override
   public boolean isViewportHidden()
   {
      return isViewportHidden;
   }

   @Override
   public void registerViewportSelectorCommandListener(ViewportSelectorCommandListener viewportSelectorCommandListener)
   {
      this.viewportSelectorCommandListener = viewportSelectorCommandListener;
   }

   public ArrayList<ViewportWindow> getViewportWindows()
   {
      return viewportWindows;
   }

   public ArrayList<GraphArrayWindow> getGraphArrayWindows()
   {
      return graphArrayWindows;
   }

   public GraphArrayPanel getGraphArrayPanel()
   {
      return myGraphArrayPanel;
   }

   public String getXMLStyleRepresentationOfEntryBoxes()
   {
      return myEntryBoxArrayPanel.getXMLRepresentationOfClass();
   }

   public String getXMLStyleRepresentationOfGraphArrayPanel()
   {
      return myGraphArrayPanel.getXMLRepresentationOfClass();
   }

   public String getXMLStyleRepresentationofJPanels()
   {
      String textToWrite = "";
      textToWrite += "\n<Extra Panels>";
      textToWrite += "\n<Main Viewport>\n";

      for (Component panel : tempPanelsHolder)
      {
         textToWrite += panel.getName() + ",";
      }

      textToWrite += "\n</Main Viewport>";
      int viewportNumber = 1;
      for (ViewportWindow viewport : viewportWindows)
      {
         if (viewport.isVisable())
         {
            textToWrite += "\n<Viewport" + viewportNumber + ">\n";
            textToWrite += viewport.savingExtraPanels();
            textToWrite += "\n</Viewport" + viewportNumber + ">\n";
            viewportNumber++;
         }
      }

      textToWrite += "</Extra Panels>\n";

      return textToWrite;
   }

   public void addPanelToTempHolderMainViewport(String panelName)
   {
      tempPanelsHolder.add(getExtraPanel(panelName));
      drawMainViewportWithExtraPanels();
   }

   public void addPanelToTempHolderViewport(String panelName, int i)
   {
      i--;
      viewportWindows.get(i).addPanelToTempHolder(getExtraPanel(panelName));
   }

   public String getXMLStyleRepresentationOfViewPorts()
   {
      String textToWrite = "";
      int z = 1;
      int counter = 1;
      for (ViewportWindow viewport : getViewportWindows())
      {
         if (viewport.isVisable())
         {
            z++;
         }
      }

      textToWrite += "\n<Number of ViewPorts>" + z + "</Number of ViewPorts>";
      textToWrite += viewportPanel.getXMLStyleRepresentationOfMainViewPort(!isViewportHidden);

      for (ViewportWindow viewport : getViewportWindows())
      {
         if (viewport.isVisable())
         {
            textToWrite += "\n<Viewport" + counter + ">" + "\n<Visible>" + !viewport.isViewportHidden() + "</Visible>";
            int canvasNumber = 1;
            textToWrite += "\n<Canvas Number>" + viewport.getCameraAdapters().size() + "</Canvas Number>";

            for (ViewportAdapterAndCameraControllerHolder view3d : viewport.getCameraAdapters())
            {
               textToWrite += viewport.getXMLStyleRepresentationOfClass(view3d, canvasNumber);
               canvasNumber++;
            }

            textToWrite += "\n</Viewport" + counter + ">";
            counter++;
         }
      }

      return textToWrite;
   }

   public boolean setViewportFromXMLDescription(String importXML)
   {
      boolean visible = viewportPanel.setMainViewPortFromXMLDescription(importXML);

      return visible;
   }

   // public String getXMLStyleRepresentationOfGraphWindow(GraphArrayPanel graphArrayPanel)
   // {
   // String textToWrite = graphArrayPanel.getXMLRepresentationOfClass();
   //
   // return textToWrite;
   // }

   public String getXMLStyleRepresentationOfGraphWindows()
   {
      String textToWrite = "";
      int numberOfGraphWindows = 0;

      for (GraphArrayWindow grapharray : getGraphArrayWindows())
      {
         if (grapharray.isVisable())
         {
            numberOfGraphWindows++;
         }
      }

      textToWrite += "\n<Graph Array Window Size>" + numberOfGraphWindows + "</Graph Array Window Size>\n";
      int z = 0;
      int window = 1;
      for (GraphArrayWindow grapharray : getGraphArrayWindows())
      {
         if (grapharray.isVisable())
         {
            textToWrite += "<Graph Array Window" + window + ">\n";
            textToWrite += "<ScreenID>" + grapharray.getScreenID() + "</ScreenID>\n";

            Point windowLocation = grapharray.getWindowLocationOnScreen();
            textToWrite += "<WindowLocation>" + windowLocation.getX() + ", " + windowLocation.getY() + "</WindowLocation>\n";

            Dimension windowSize = grapharray.getWindowSize();
            textToWrite += "<WindowSize>" + windowSize.getWidth() + ", " + windowSize.getHeight() + "</WindowSize>\n";

            textToWrite += graphArrayWindows.get(z).myGraphArrayPanel.getXMLRepresentationOfClass();
            textToWrite += "\n</Graph Array Window" + window + ">";
            window++;
         }

         z++;
      }

      return textToWrite;
   }

   public void makeCheckBoxesConsistent(int item, boolean selected)
   {
      standardGUIActions.extraPanelsMenu.getItem(item).setSelected(selected);
   }

   public String getXMLStyleRepresentationofMultiViews()
   {
      return viewportPanel.getXMLStyleRepresentationofMultiViews(currentView);
   }

   public void setupMultiViews(String xmlRepresentation, String currentView)
   {
      viewportPanel.setupMultiViews(xmlRepresentation, currentView);
   }

   public void loadDefaultGUIConfigurationFile()
   {
      GUIConfigurationSaveAndLoad guiConfigurationSaveAndLoad = new GUIConfigurationSaveAndLoad(guiEnablerAndDisabler, this);
      guiConfigurationSaveAndLoad.loadGUIConfiguration(getDefaultFile(guiConfigFileEnding));

      loadRegistryConfiguration();
   }

   public void saveNormalGUIConfigurationFile()
   {
      GUIConfigurationSaveAndLoad guiConfigurationSaveAndLoad = new GUIConfigurationSaveAndLoad(guiEnablerAndDisabler, this);
      guiConfigurationSaveAndLoad.saveNormalGUIConfiguration();
   }

   private void saveDefaultGUIConfigurationFile()
   {
      GUIConfigurationSaveAndLoad guiConfigurationSaveAndLoad = new GUIConfigurationSaveAndLoad(guiEnablerAndDisabler, this);
      guiConfigurationSaveAndLoad.defaultSave(getDefaultFile(guiConfigFileEnding));
   }

   public void loadRegistryConfiguration()
   {
      String defaultFile = getDefaultFile(registryFileEnding);
      if (defaultFile == null)
         return;

      File loadFile = new File(defaultFile);
      if (loadFile.exists())
      {
         ArrayList<YoVariableRegistry> treeNodes = rootRegistry.getAllRegistriesIncludingChildren();

         // getAllRegistrys(rootRegistry, treeNodes);

         // System.out.println(i+" "+realCound+" "+rootRegistry.createVarListsIncludingChildren().size());
         SimpleFileReader reader = new SimpleFileReader(loadFile);
         String nextLine;
         while ((nextLine = reader.nextLine()) != null)
         {
            nextLine = nextLine.trim();
            StringTokenizer tok = new StringTokenizer(nextLine, "*");
            String name = tok.nextToken();
            YoVariableRegistry current = findRegistry(name, treeNodes);

            if (current == null)
            {
               // TODO: JEP 1/7/2013. I changed this so that it doesn't create the registry if it doesn't exist.
               // Otherwise if you try to add registries after loading, you'll get a repeated registry error.
               // FixMe this may be a better way of finding the registries instead of searching for them.
               // current = rootRegistry.getOrCreateAndAddRegistry(new NameSpace(name));
               try
               {
                  current = rootRegistry.getRegistry(new NameSpace(name));
               }
               catch (RuntimeException exception)
               {
                  //TODO: Create some way to automatically figure out the registry stuff
                  // and fix the config files when they don't have a registry.
//                  PrintTools.warn(StandardSimulationGUI.this, "Warning: Could not find registry " + name);
                  // new Throwable().printStackTrace();
               }
            }

            if (current != null)
            {
               if (tok.nextToken().equals("true"))
               {
                  current.setSending(true);
               }
               else
                  current.setSending(false);

               if (tok.nextToken().equals("true"))
               {
                  current.setLogging(true);
               }
               else
                  current.setLogging(false);
            }
         }
      }
   }

   private YoVariableRegistry findRegistry(String name, ArrayList<YoVariableRegistry> allReg)
   {
      for (YoVariableRegistry reg : allReg)
      {
         if (reg.getNameSpace().getName().equals(name))
         {
            return reg;
         }
      }

      return null;
   }

   public void saveRegistryConfigurations()
   {
      String filename = getDefaultFile(registryFileEnding);
      if (filename == null)
         return;

      SimpleFileWriter writer = new SimpleFileWriter(new File(filename));
      ArrayList<YoVariableRegistry> treeNodes = rootRegistry.getAllRegistriesIncludingChildren();

      String outString = "";
      for (YoVariableRegistry node : treeNodes)
      {
         outString += node.getNameSpace().getName() + "*" + node.isSent() + "*" + node.isLogged() + "\n";
      }

      writer.write(outString);
      writer.close();
   }

   private String getDefaultFile(String extension)
   {
      if (robots == null)
         return null;
      if (robots[0] == null)
         return null;

      configFileName = robots[0].getName() + "_" + sim.getRunningName() + "_" + extension;
      File Configs = new File("Configurations");

      if (!Configs.exists())
      {
         Configs.mkdir();
      }

      String path = Configs.toURI().getPath();

      String defaultConfiguration = path + configFileName;

      return defaultConfiguration;
   }

   public void setupMultiViewsMultipleViewports(String xmlRepresentation, int size)
   {
      int z = multiViewCanvas;
      for (int i = 0; i < size; i++)
      {
         int label = i + 1;
         String first = "<Canvas" + label + ">";
         String second = "</Canvas" + label + ">";
         String textToLoad = XMLReaderUtility.getMiddleString(0, xmlRepresentation, first, second);
         double posX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera X>", "</Viewport Camera X>"));
         double posY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera Y>", "</Viewport Camera Y>"));
         double posZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, textToLoad, "<Viewport Camera Z>", "</Viewport Camera Z>"));

         viewportWindows.get(z).getCameraAdapters().get(i).getCameraController().setCameraPosition(posX, posY, posZ);

         String Dolly = XMLReaderUtility.getMiddleString(0, textToLoad, "<Dolly data>", "</Dolly data>");
         double DollyX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position X>", "</Position X>"));
         double DollyY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position Y>", "</Position Y>"));
         double DollyZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Dolly, "<Position Z>", "</Position Z>"));

         viewportWindows.get(z).getCameraAdapters().get(i).getCameraController().setDollyOffsets(DollyX, DollyY, DollyZ);

         String Dolly_Boolean = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly>", "</Dolly>");
         String Dolly_Boolean_X = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly X>", "</Dolly X>");
         String Dolly_Boolean_Y = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly Y>", "</Dolly Y>");
         String Dolly_Boolean_Z = XMLReaderUtility.getMiddleString(0, Dolly, "<Dolly Z>", "</Dolly Z>");
         boolean dolly_set = true;
         boolean dolly_setX = true;
         boolean dolly_setY = true;
         boolean dolly_setZ = true;

         if (Dolly_Boolean.equals("false"))
         {
            dolly_set = false;
         }

         if (Dolly_Boolean_X.equals("false"))
         {
            dolly_setX = false;
         }

         if (Dolly_Boolean_Y.equals("false"))
         {
            dolly_setY = false;
         }

         if (Dolly_Boolean_Z.equals("false"))
         {
            dolly_setZ = false;
         }

         viewportWindows.get(z).getCameraAdapters().get(i).getCameraController().setDolly(dolly_set, dolly_setX, dolly_setY, dolly_setZ);

         String Track = XMLReaderUtility.getMiddleString(0, textToLoad, "<Track data>", "</Track data>");
         double TrackX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position X>", "</Position X>"));
         double TrackY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position Y>", "</Position Y>"));
         double TrackZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Track, "<Position Z>", "</Position Z>"));

         viewportWindows.get(z).getCameraAdapters().get(i).getCameraController().setTrackingOffsets(TrackX, TrackY, TrackZ);

         String Track_Boolean = XMLReaderUtility.getMiddleString(0, Track, "<Track>", "</Track>");
         String Track_Boolean_X = XMLReaderUtility.getMiddleString(0, Track, "<Track X>", "</Track X>");
         String Track_Boolean_Y = XMLReaderUtility.getMiddleString(0, Track, "<Track Y>", "</Track Y>");
         String Track_Boolean_Z = XMLReaderUtility.getMiddleString(0, Track, "<Track Z>", "</Track Z>");
         boolean track_set = true;
         boolean track_setX = true;
         boolean track_setY = true;
         boolean track_setZ = true;

         if (Track_Boolean.equals("false"))
         {
            track_set = false;
         }

         if (Track_Boolean_X.equals("false"))
         {
            track_setX = false;
         }

         if (Track_Boolean_Y.equals("false"))
         {
            track_setY = false;
         }

         if (Track_Boolean_Z.equals("false"))
         {
            track_setZ = false;
         }

         viewportWindows.get(z).getCameraAdapters().get(i).getCameraController().setTracking(track_set, track_setX, track_setY, track_setZ);

         String Fix = XMLReaderUtility.getMiddleString(0, textToLoad, "<Fix Position>", "</Fix Position>");
         double FixX = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix X>", "</Fix X>"));
         double FixY = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix Y>", "</Fix Y>"));
         double FixZ = Double.parseDouble(XMLReaderUtility.getMiddleString(0, Fix, "<Fix Z>", "</Fix Z>"));

         viewportWindows.get(z).getCameraAdapters().get(i).getCameraController().setFixPosition(FixX, FixY, FixZ);
      }

      multiViewCanvas++;
   }

   public Component getExtraPanel(String panelName)
   {
      return (extraPanelConfigurationList.getExtraPanelConfiguration(panelName).getPanel());
   }

   public void makeCheckMarksConsistentForExtraPanels(String panelName, boolean isSelected)
   {
      for (int i = 0; i < standardGUIActions.extraPanelsMenu.getItemCount(); i++)
      {
         if (panelName.equals(standardGUIActions.extraPanelsMenu.getItem(i).getText()))
         {
            standardGUIActions.extraPanelsMenu.getItem(i).setSelected(isSelected);
         }
      }

      for (ViewportWindow viewport : viewportWindows)
      {
         viewport.makeExtraPanelsMenuConsistent(panelName, isSelected);
      }
   }

   public void removeExtraPanel(String panelName)
   {
      for (ViewportWindow viewport : viewportWindows)
      {
         viewport.removeExtraPanel(panelName);
      }

      tempPanelsHolder.remove(getExtraPanel(panelName));
      drawMainViewportWithExtraPanels();
   }

   public void makeCheckMarksConsistentWithMainPanel(ViewportWindow viewport)
   {
      for (int i = 0; i < standardGUIActions.extraPanelsMenu.getItemCount(); i++)
      {
         if (standardGUIActions.extraPanelsMenu.getItem(i).isSelected())
         {
            viewport.makeExtraPanelsMenuConsistent(standardGUIActions.extraPanelsMenu.getItem(i).getText(), true);
         }
      }
   }

   public BookmarkedVariablesHolder getBookmarkedVariablesHolder()
   {
      return bookmarkedVariablesHolder;
   }

   public GraphicsDynamicGraphicsObject addYoGraphic(YoGraphic yoGraphic, boolean updateFromSimulationThread)
   {
      GraphicsDynamicGraphicsObject graphicsDynamicGraphicsObject = new GraphicsDynamicGraphicsObject(yoGraphic, closeableAndDisposableRegistry);

      if (updateFromSimulationThread)
      {
         graphicsUpdatables.add(graphicsDynamicGraphicsObject);
      }

      graphics3dAdapter.addRootNode(graphicsDynamicGraphicsObject);

      return graphicsDynamicGraphicsObject;
   }

   public void addYoGraphicsList(YoGraphicsList yoGraphicsList, boolean updateFromSimulationThread)
   {
      for (YoGraphic yoGraphic : yoGraphicsList.getYoGraphics())
      {
         addYoGraphic(yoGraphic, updateFromSimulationThread);
      }
   }

   public void addYoGraphicsList(YoGraphicsList yoGraphicObjectsList, boolean updateFromSimulationThread, List<GraphicsUpdatable> graphicsUpdatablesToPack)
   {
      for (YoGraphic yoGraphic : yoGraphicObjectsList.getYoGraphics())
      {
         graphicsUpdatablesToPack.add(addYoGraphic(yoGraphic, updateFromSimulationThread));
      }
   }

   public void addYoGraphicsLists(List<YoGraphicsList> yoGraphicObjectsLists, boolean updateFromSimulationThread,
         List<GraphicsUpdatable> graphicsUpdatablesToPack)
   {
      for (YoGraphicsList yoGraphicsList : yoGraphicObjectsLists)
      {
         addYoGraphicsList(yoGraphicsList, updateFromSimulationThread, graphicsUpdatablesToPack);
      }
   }

   public void addYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry, boolean updateFromSimulationThread)
   {
      if (!updateFromSimulationThread && graphics3dAdapter != null)
      {
         yoGraphicsListRegistry.setGraphicsConch(graphics3dAdapter.getGraphicsConch());
      }

      List<GraphicsUpdatable> graphicsDynamicGraphicsObjects = new ArrayList<GraphicsUpdatable>();
      addYoGraphicsLists(yoGraphicsListRegistry.getYoGraphicsLists(), updateFromSimulationThread, graphicsDynamicGraphicsObjects);

      if (!updateFromSimulationThread)
      {
         yoGraphicsListRegistry.addGraphicsUpdatables(graphicsDynamicGraphicsObjects);
      }

      yoGraphicsListRegistry.setYoGraphicsUpdatedRemotely(updateFromSimulationThread);
      yoGraphicsListRegistry.setYoGraphicsRegistered();
   }

   public GraphGroupList getGraphGroupList()
   {
      return graphGroupList;
   }

   public EntryBoxGroupList getEntryBoxGroupList()
   {
      return entryBoxGroupList;
   }

   public ConfigurationList getConfigurationList()
   {
      return configurationList;
   }

   public CameraConfigurationList getCameraConfigurationList()
   {
      return cameraConfigurationList;
   }

   public ViewportConfigurationList getViewportConfigurationList()
   {
      return viewportConfigurationList;
   }

   public GraphConfigurationList getGraphConfigurationList()
   {
      return graphConfigurationList;
   }

   public StandardGUIActions getStandardGUIActions()
   {
      return standardGUIActions;
   }

   @Override
   public void closeAndDispose()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            closeAndDisposeLocal();
         }
      });
   }

   private void closeAndDisposeLocal()
   {
      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Clearing Exit Action Listeners");
      System.out.flush();

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Clearing Default Var Lists");
      System.out.flush();

      if (closeableAndDisposableRegistry !=  null)
      {
         closeableAndDisposableRegistry.closeAndDispose();
         closeableAndDisposableRegistry = null;
      }
      
      if (configurationList != null)
      {
         configurationList = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing GraphArrayWindow");
      System.out.flush();

      if (graphArrayWindow != null)
      {
         for (GraphArrayWindow graphArrayWindow : graphArrayWindows)
         {
            graphArrayWindow.closeAndDispose();
         }

         graphArrayWindows.clear();
         graphArrayWindows = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing ViewportWindows");
      System.out.flush();

      if (viewportWindows != null)
      {
         for (ViewportWindow viewportWindow : viewportWindows)
         {
            if (DEBUG_CLOSE_AND_DISPOSE)
               System.out.println("   Closing and Disposing a ViewportWindow");
            System.out.flush();
            viewportWindow.closeAndDispose();
         }

         viewportWindows.clear();
         viewportWindows = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing JFrame");
      System.out.flush();

      if (jFrame != null)
      {
         if (DEBUG_CLOSE_AND_DISPOSE)
            System.out.println("Removing WindowListeners");
         System.out.flush();

         WindowListener[] windowListeners = jFrame.getWindowListeners();
         if (windowListeners != null)
         {
            for (WindowListener windowListener : windowListeners)
            {
               jFrame.removeWindowListener(windowListener);
            }
         }

         if (DEBUG_CLOSE_AND_DISPOSE)
            System.out.println("Removing ComponentListeners");
         System.out.flush();

         ComponentListener[] componenetListeners = jFrame.getComponentListeners();
         if (componenetListeners != null)
         {
            for (ComponentListener componentListener : componenetListeners)
            {
               jFrame.removeComponentListener(componentListener);
            }
         }

         if (DEBUG_CLOSE_AND_DISPOSE)
            System.out.println("Setting Menu Bar to null.");
         System.out.flush();
         jFrame.setMenuBar(null);

         if (menuBar != null)
         {
            menuBar.removeAll();
            menuBar = null;
         }

         if (DEBUG_CLOSE_AND_DISPOSE)
            System.out.println("Removing all From JFrame.");
         System.out.flush();

         jFrame.removeAll(); // Remove all seems to be taking too long. Perhaps it needs to be put on a Swing Thread or something?

         if (DEBUG_CLOSE_AND_DISPOSE)
            System.out.println("Disposing JFrame.");
         System.out.flush();
         jFrame.setVisible(false);
         jFrame.dispose();
         jFrame = null;
      }

      if (shutdownHook != null)
      {
         if (DEBUG_CLOSE_AND_DISPOSE)
            System.out.println("Deregistering shutdown hook.");
         Runtime.getRuntime().removeShutdownHook(shutdownHook);
         shutdownHook = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Removing all from Button Panel");
      System.out.flush();

      if (buttonPanel != null)
      {
         buttonPanel.removeAll(); // Remove all seems to be taking too long. Perhaps it needs to be put on a Swing Thread or something?
         buttonPanel = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Removing all from Content Pane");
      System.out.flush();

      if (contentPane != null)
      {
         contentPane.removeAll(); // Remove all seems to be taking too long. Perhaps it needs to be put on a Swing Thread or something?
         contentPane = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing CombinedVarPanel");
      System.out.flush();

      if (yoVariableExplorerTabbedPane != null)
      {
         yoVariableExplorerTabbedPane.closeAndDispose();
         yoVariableExplorerTabbedPane = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing EntryBoxArrayPanel");
      System.out.flush();

      if (myEntryBoxArrayPanel != null)
      {
         myEntryBoxArrayPanel.closeAndDispose();
         myEntryBoxArrayPanel = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing GraphArrayPanel");
      System.out.flush();

      if (myGraphArrayPanel != null)
      {
         myGraphArrayPanel.closeAndDispose();
         myGraphArrayPanel = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Removing all From NumericContentPane.");
      System.out.flush();

      if (numericContentPane != null)
      {
         numericContentPane.removeAll(); // Remove all seems to be taking too long. Perhaps it needs to be put on a Swing Thread or something?
         numericContentPane = null;
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing StandardGUIActions");
      System.out.flush();

      if (standardGUIActions != null)
      {
         standardGUIActions.closeAndDispose();
         standardGUIActions = null;
      }
      
      
      if (allDialogConstructorsHolder != null)
      {
         allDialogConstructorsHolder.closeAndDispose();
         allDialogConstructorsHolder = null;
      }
      
      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing ViewportPanel");
      System.out.flush();

      if (viewportPanel != null)
      {
         viewportPanel.closeAndDispose();
         viewportPanel = null;
      }
      
      if (timeStepMouseWheelListener != null)
      {
         timeStepMouseWheelListener.closeAndDispose();
         timeStepMouseWheelListener = null;
      }

      if (splitPane != null)
      {
         splitPane.removeAll(); // Remove all seems to be taking too long. Perhaps it needs to be put on a Swing Thread or something?
         splitPane = null;
      }

      if (jSplitPane != null)
      {
         jSplitPane.removeAll(); // Remove all seems to be taking too long. Perhaps it needs to be put on a Swing Thread or something?
         jSplitPane = null;
      }

      if (bookmarkedVariablesHolder != null)
      {
         bookmarkedVariablesHolder = null;
      }

      if (mainPanel != null)
      {
         mainPanel.removeAll(); // Remove all seems to be taking too long. Perhaps it needs to be put on a Swing Thread or something?
         mainPanel = null;
      }
      
      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Closing and Disposing graphics3dAdapter");
      System.out.flush();

      graphicsUpdatables = null;
      graphicsRobots = null;
      
      if (graphics3dAdapter != null)
      {
         graphics3dAdapter.closeAndDispose();
         graphics3dAdapter = null;
      }

      varGroupList = null;
      cameraMountList = null;
      graphGroupList = null;

      graphConfigurationList = null;
      entryBoxGroupList = null;
      cameraConfigurationList = null;
      extraPanelConfigurationList = null;

      viewportConfigurationList = null;

      splitPane = null;
      viewportSelectorCommandListener = null;

      jApplet = null;
      
      parentContainer.removeAll();
      parentContainer = null;

      robots = null;
      selectedVariableHolder = null;
      sim = null;
      graphArrayWindow = null;

      splashWindow = null;

      myDataBuffer = null;

      tempPanelsHolder = null;
      rootRegistry = null;

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Done Closing and Disposing StandardSimulationGUI");
      System.out.flush();

   }

   public void updateSimulationGraphics()
   {
      if (graphics3dAdapter != null)
      {
         synchronized (graphics3dAdapter.getGraphicsConch())
         {
            // TODO: We were often getting concurrentModificationException here. So I changed graphicsUpdatables to a ConcurrentLinkedQueue.
            // Need to test to make sure that fixes things.
            for (GraphicsUpdatable graphicsUpdatable : graphicsUpdatables)
            {
               graphicsUpdatable.update();
            }
         }

      }
   }
   
   public Object getGraphicsConch()
   {
      return graphics3dAdapter.getGraphicsConch();
   }

   public void removeStaticGraphics3dNode(Graphics3DNode nodeToRemove)
   {
      graphics3dAdapter.removeRootNode(nodeToRemove);
   }

   public void attachSelectedListener(SelectedListener selectedListener)
   {
      graphics3dAdapter.addSelectedListener(selectedListener);
   }

   public GraphicsRobot getGraphicsRobot(Robot robot)
   {
      return graphicsRobots.get(robot);
   }

   public void startStreamingVideoData(CameraConfiguration cameraConfiguration, int width, int height, RenderedSceneHandler videoDataServer,
         TimestampProvider timestampProvider, int framesPerSecond)
   {
      CameraTrackingAndDollyPositionHolder cameraTrackingAndDollyPositionHolder = new CameraTrackAndDollyYoVariablesHolder(yoVariableHolder);
      new OffscreenBufferVideoServer(graphics3dAdapter, cameraMountList, cameraConfiguration, cameraTrackingAndDollyPositionHolder, width, height,
            videoDataServer, timestampProvider, framesPerSecond);

   }

   public Graphics3DAdapter getGraphics3dAdapter()
   {
      return graphics3dAdapter;
   }

   public void clearAllEntryTabs()
   {
      myEntryBoxArrayPanel.closeAndDispose();
   }

   public void addViewportPanelToMainPanel()
   {
      mainPanel.add(viewportPanel); //TODO: Why is this here? 
   }

}
