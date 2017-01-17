package us.ihmc.simulationconstructionset;

import java.awt.Color;
import java.awt.Component;
import java.awt.Dimension;
import java.awt.Frame;
import java.awt.Point;
import java.awt.image.BufferedImage;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.Writer;
import java.net.URL;
import java.util.ArrayList;
import java.util.List;
import java.util.Vector;

import javax.swing.AbstractButton;
import javax.swing.ImageIcon;
import javax.swing.JApplet;
import javax.swing.JCheckBox;
import javax.swing.JComboBox;
import javax.swing.JDialog;
import javax.swing.JFileChooser;
import javax.swing.JFrame;
import javax.swing.JLabel;
import javax.swing.JMenuBar;
import javax.swing.JOptionPane;
import javax.swing.JRadioButton;
import javax.swing.JTextField;
import javax.vecmath.Color3f;
import javax.vecmath.Tuple3d;

import com.jme3.renderer.Camera;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.HeightMap;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.input.SelectedListener;
import us.ihmc.graphicsDescription.structure.Graphics3DNode;
import us.ihmc.graphicsDescription.structure.Graphics3DNodeType;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphic;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DAdapter;
import us.ihmc.jMonkeyEngineToolkit.Graphics3DBackgroundScaleMode;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.jMonkeyEngineToolkit.camera.CaptureDevice;
import us.ihmc.jMonkeyEngineToolkit.camera.RenderedSceneHandler;
import us.ihmc.robotics.TickAndUpdatable;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.robotics.dataStructures.listener.RewoundListener;
import us.ihmc.robotics.dataStructures.listener.YoVariableRegistryChangedListener;
import us.ihmc.robotics.dataStructures.registry.NameSpace;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.YoVariable;
import us.ihmc.robotics.dataStructures.variable.YoVariableList;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateMachinesJPanel;
import us.ihmc.robotics.time.GlobalTimer;
import us.ihmc.robotics.time.RealTimeRateEnforcer;
import us.ihmc.simulationconstructionset.DataBuffer.RepeatDataBufferEntryException;
import us.ihmc.simulationconstructionset.commands.AddCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.commands.AddKeyPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.CreateNewGraphWindowCommandExecutor;
import us.ihmc.simulationconstructionset.commands.CreateNewViewportWindowCommandExecutor;
import us.ihmc.simulationconstructionset.commands.CropBufferCommandExecutor;
import us.ihmc.simulationconstructionset.commands.CutBufferCommandExecutor;
import us.ihmc.simulationconstructionset.commands.DataBufferCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.ExportSnapshotCommandExecutor;
import us.ihmc.simulationconstructionset.commands.GotoInPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.GotoOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.NextCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.commands.PackBufferCommandExecutor;
import us.ihmc.simulationconstructionset.commands.PreviousCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.commands.RemoveCameraKeyCommandExecutor;
import us.ihmc.simulationconstructionset.commands.RunCommandsExecutor;
import us.ihmc.simulationconstructionset.commands.SetInPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.SetOutPointCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StepBackwardCommandExecutor;
import us.ihmc.simulationconstructionset.commands.StepForwardCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleCameraKeyModeCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandExecutor;
import us.ihmc.simulationconstructionset.commands.ToggleKeyPointModeCommandListener;
import us.ihmc.simulationconstructionset.commands.WriteDataCommandExecutor;
import us.ihmc.simulationconstructionset.graphics.GraphicsDynamicGraphicsObject;
import us.ihmc.simulationconstructionset.gui.DynamicGraphicMenuManager;
import us.ihmc.simulationconstructionset.gui.EventDispatchThreadHelper;
import us.ihmc.simulationconstructionset.gui.GraphArrayWindow;
import us.ihmc.simulationconstructionset.gui.StandardGUIActions;
import us.ihmc.simulationconstructionset.gui.StandardSimulationGUI;
import us.ihmc.simulationconstructionset.gui.ViewportWindow;
import us.ihmc.simulationconstructionset.gui.config.VarGroupList;
import us.ihmc.simulationconstructionset.gui.dialogConstructors.GUIEnablerAndDisabler;
import us.ihmc.simulationconstructionset.gui.hierarchyTree.NameSpaceHierarchyTree;
import us.ihmc.simulationconstructionset.physics.CollisionHandler;
import us.ihmc.simulationconstructionset.physics.ScsPhysics;
import us.ihmc.simulationconstructionset.physics.visualize.DefaultCollisionVisualizer;
import us.ihmc.simulationconstructionset.robotcommprotocol.GUISideCommandListener;
import us.ihmc.simulationconstructionset.robotcommprotocol.RobotConnectionGUIUpdater;
import us.ihmc.simulationconstructionset.robotcommprotocol.RobotSocketConnection;
import us.ihmc.simulationconstructionset.robotcommprotocol.SCSRobotGUICommunicatorComponents;
import us.ihmc.simulationconstructionset.robotdefinition.RobotDefinitionFixedFrame;
import us.ihmc.simulationconstructionset.scripts.Script;
import us.ihmc.simulationconstructionset.synchronization.SimulationSynchronizer;
import us.ihmc.simulationconstructionset.util.graphics.DynamicGraphicCheckBoxMenuItem;
import us.ihmc.tools.TimestampProvider;
import us.ihmc.tools.gui.GraphicsUpdatable;
import us.ihmc.tools.io.printing.PrintTools;
import us.ihmc.tools.thread.ThreadTools;

/**
 * <p>Title: SimulationConstructionSet</p>
 * <p/>
 * <p>Description: Class for constructing a simulation and for setting many of the configuration options for the simulation.
 * Many of the methods provided by this class provide easy access to features that may be modified directly elsewhere.
 * As a result the documentation provided in this class is of a general nature; for more detailed information see the tutorial or the class in question.
 * </p>
 * <p/>
 * <p><b><big>Cameras and Viewports</big></b></p>
 * <p>The simulation construction set provides a camera interface that is both robust and versatile.  Cameras can easily be configured to track or follow your robot from any conceivable angle.
 * The following provides a brief introduction to this functionality.  For more detailed information see {@link Camera Camera} and {@link ViewportConfiguration ViewportConfiguration}.</p>
 * <p/>
 * <p><b>Cameras</b></p>
 * <p>Each camera has two main parameters: fix and position.  A camera's fix is the point in space at which the camera is looking, while
 * the position is the location of the camera itself.  These parameters can be configured at any time for the camera currently selected by the user.
 * This camera is often referred to as the currently active camera and is identified by the red outline which surrounds it.</p>
 * <p/>
 * <p>Cameras may also be configured to track or follow the robot.  These two modes are referred to as track and dolly respectively.
 * In track mode the camera remains stationary keeping the robot in view by moving its fix.  When in dolly mode the camera's position is modified to keep up with the robot.
 * If both modes are enabled the camera will keep up with the robot while maintaining a constant view.
 * These features function by monitoring a YoVariable for each axis using the change in its value to modify the fix and or position of the camera.
 * The variables used by dolly and track modes are independant and may be changed at any time.  Each mode may also be configured with an offset to augment the fix or position with respect to the target.
 * In track mode this offset defaults to zero, keeping the camera's view directly on the target.  Dolly mode has a default offset of (2.0,12.0,0.0) without which the camera would be ontop of the robot.</p>
 * A camera may also be mounted to the robot itself, providing a "robot cam" view.  To accomplish this, a camera mount must be created and added to a joint on the robot.
 * Once the mount exists it may be specified by name in a camera configuration.  There is no way to mount a camera to the robot without a configuration.
 * <p/>
 * <p>While the ability to manipulate the active camera is useful, the true power of this system is the ability to store predefined camera configurations for easy access.
 * Once configured, a camera may be accessed in the GUI via the Viewport drop down menu or added to a ViewportConfiguration.  The following are several examples of camera configurations.</p>
 * <p/>
 * {@code CameraConfiguration camera1 = new CameraConfiguration("camera 1");}<br />
 * {@code camera1.setCameraFix(0.0, 0.0, 0.6);}<br />
 * {@code camera1.setCameraPosition(-9.0, 3.0, 0.8);}<br />
 * {@code camera1.setCameraTracking(true, true, true, false);}<br />
 * {@code camera1.setCameraDolly(false, true, true, false);}<br />
 * {@code sim.setupCamera(camera1);}<br />
 * <br />
 * {@code CameraConfiguration camera2 = new CameraConfiguration("camera 2");}<br />
 * {@code camera2.setCameraFix(0.0, 0.0, 0.6);}<br />
 * {@code camera2.setCameraPosition(-9.0, 3.0, 0.15);}<br />
 * {@code camera2.setCameraTracking(true, true, true, false);}<br />
 * {@code camera2.setCameraDolly(true, true, true, false);}<br />
 * {@code camera2.setCameraDollyOffsets(-3.6, 4.0, 0.0);}<br />
 * {@code camera2.setCameraTrackingOffsets(0.0, 0.0, 0.0);}<br />
 * {@code sim.setupCamera(camera2);}<br />
 * <br />
 * {@code CameraConfiguration camera3 = new CameraConfiguration("robot cam");}<br />
 * {@code camera3.setCameraMount("robot cam mount");}<br />
 * {@code sim.setupCamera(camera3);}<br />
 * <p/>
 * <p>In the examples above camera 1 is tracking only, camera 2 is tracking and dollying, and camera 3 is mounted to the robot.  "sim" is an instance of SimulationConstructionSet</p>
 * <p/>
 * <p><b>ViewportConfigurations</b></p>
 * <p>Once several CameraConfigurations have been created and added to the simulation, a {@link ViewportConfiguration ViewportConfiguration} may be assembled.
 * ViewportConfigurations allow a set of cameras to be displayed in a single window with each camera assigned to a specific position and size.
 * Once created, ViewportConfigurations may be selected via the Viewport drop down menu.  It is also possible to create a ViewportWindow based on a ViewportConfiguration.</p>
 * <p/>
 * <p><b><big>Display Groups</big></b></p>
 * <p/>
 * <p>A single simulation can easily involve a large number of variables which the user is interested in displaying.  This section provides a general overview of the methods
 * currently implemented to facilitate this.</p>
 * <p/>
 * <p><b>VarGroups</b></p>
 * <p/>
 * <p>VarGroups provide a convienent means by which to group and reference YoVariables.  A group may be created by providing a list of YoVariable names, regular expressions, or both.
 * By default, a group named "all" containing every simulation variable is created.  Once created, a VarGroup may be selected for display from the Configuration drop down menu.
 * VarGroups are also used during the creation of Configurations and while exporting data.</p>
 * <p/>
 * {@code sim.setupVarGroup("kinematics", new String[]{"t"}, new String[]{"q_.*", "qd_.*"});}<br />
 * <p/>
 * <p>In the example above, a VarGroup named kinematics is created which contains the YoVariable named "t" along with all YoVariables who's names begin with "q_." or "qd_.".</p>
 * <p/>
 * {@code sim.setupVarGroup("torques", null, new String[]{"t", "tau_.*"});}<br />
 * <p/>
 * <p>This example creates a VarGroup named "torques" containing the variable "t" as well as all YoVariables who's names begin with "tau_.".
 * For more information on regular expressions see java.util.regex.Pattern.</p>
 * <p/>
 * <p><b>GraphGroups</b></p>
 * <p/>
 * <p>Much like VarGroups, graph groups provide a means to store and reference a set of graphs.  GraphGroups are identified by name and contain a list of graphs, each of which may display a maximum of four variables.
 * At creation, separate {@link GraphConfiguration GraphConfigurations} can be provided for each graph which allow the configuration of graph min and max values. Once a GraphGroup has been added
 * to the simulation it may be selected from the configuration drop down menu.  Separate GraphWindows similar to ViewportWindows may be creating using a specific GraphGroup, which is often helpful
 * when screen space is an issue.  GraphGroups can be used in conjunction with VarGroups when Configurations are created.</p>
 * <p/>
 * {@code sim.setupGraphGroup("states", new String[][]{{"left_state", "right_state"}, {"t"}, {"q_x", "q_z"}});}<br />
 * <p/>
 * The example above creates a GraphGroup named "states" containing the following three graphs:<br />
 * <OL><LI>"left_state" and "right_state"</LI><LI>"t"</LI><LI>"q_x" and "q_z"</LI></OL>
 * When multiple variables are present in a single graph, each variable has a different color.<br />
 * <p/>
 * {@code sim.setupGraphGroup("joint angles", new String[][]{{"left_state", "right_state"}, {"q_lh"}, {"q_lk"}, {"q_la"}, {"q_rh"}, {"q_rk"}, {"q_ra"}});}<br />.
 * {@code sim.setupGraphGroup("joint velocities", new String[][]{{"qd_lh"}, {"qd_lk"}, {"qd_la"}, {"qd_rh"}, {"qd_rk"}, {"qd_ra"}});}<br />
 * <p>Examples two and three create a graph per specified variable.</p>
 * <p/>
 * {@code sim.setupGraphGroup("joint torques", new String[][]{{"tau_lh"}, {"tau_lk"}, {"tau_la"}, {"tau_rh"}, {"tau_rk"}, {"tau_ra"}}, 2);}<br />
 * <p/>
 * <p>In example four, each variable has a separate graph, however, these graphs are displayed in two columns.  Graphs are placed left to right, top to bottom beginning at the top left corner.
 * A single graph panel can display a maximum of 24 graphs over 4 columns.</p>
 * <p/>
 * <p><b>EntryBoxGroups</b></p>
 * <p/>
 * <p>EntryBoxGroups provide an easy means to select between different sets of variables for entry.  Once created an EntryBoxGroup may be selected from the Configuration drop down menu and or
 * added to a Configuration directly.  Entry boxes appear at the bottom of the main GUI frame and allow the user to modify variable values during simulation.  EntryBoxGroups are referenced by
 * their names given at creation.  YoVariables may be specified by name or by regular expression when added.</p>
 * <p/>
 * {@code sim.setupEntryBoxGroup("control vars1", new String[]{"t_gain","t_damp","hip_d","hip_hold","hip_gain","hip_damp", "swing_gain_knee","swing_damp_knee", "q_x", "q_y", "q_z"});}<br />
 * {@code sim.setupEntryBoxGroup("control vars1", new String[]{"t_gain","t_damp","hip_d","hip_hold","hip_gain","hip_damp", "swing_gain_knee","swing_damp_knee"}, new String[]{"q_*"});}<br />
 * <p>In the first example, the 11 variables are specified by name.  The second example specifies some variables by name as well as including an array of regular expressions to be used in finding
 * additional variables.</p>
 * <p/>
 * <p>Copyright: Copyright (c) 2005</p>
 * <p/>
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author Jerry Pratt
 * @version 1.0
 */
public class SimulationConstructionSet implements Runnable, YoVariableHolder, RunCommandsExecutor, AddKeyPointCommandExecutor, AddCameraKeyCommandExecutor,
      CreateNewGraphWindowCommandExecutor, CreateNewViewportWindowCommandExecutor, CropBufferCommandExecutor, CutBufferCommandExecutor, ExportSnapshotCommandExecutor,
      GotoInPointCommandExecutor, GotoOutPointCommandExecutor, NextCameraKeyCommandExecutor, PackBufferCommandExecutor, PreviousCameraKeyCommandExecutor,
      RemoveCameraKeyCommandExecutor, SetInPointCommandExecutor, SetOutPointCommandExecutor, StepBackwardCommandExecutor, StepForwardCommandExecutor,
      ToggleCameraKeyModeCommandExecutor, ToggleKeyPointModeCommandExecutor, GUIEnablerAndDisabler, WriteDataCommandExecutor, TimeHolder,
      DataBufferCommandsExecutor, TickAndUpdatable
{
   private static final boolean TESTING_LOAD_STUFF = false;

   /**
    * The default size of the data buffer in record steps.
    */
   public static final String rootRegistryName = "root";
   private static final boolean DEBUG_CLOSE_AND_DISPOSE = false;

   private static double REAL_TIME_RATE = 1.0;
   private double SECONDS_PER_FRAME = 0.04; // 0.1 seconds per frame (25Hz)
   private long TICKS_PER_PLAY_CYCLE;
   private long PLAY_CYCLE_TIME_MS;

   // private int count = 0;
   private double simulateDurationInSeconds = 10000.0;


   private StandardSimulationGUI myGUI;
   private StandardAllCommandsExecutor standardAllCommandsExecutor;

   private VarGroupList varGroupList = new VarGroupList();
   private YoVariableRegistry rootRegistry;

   private JFrame jFrame;
   private JApplet jApplet;

   // private boolean keyPointToggle = false;

   private boolean simulationThreadIsUpAndRunning = false;
   private boolean isSimulating = false;
   private boolean simulateNoFasterThanRealTime = false;
   private final RealTimeRateEnforcer realTimeRateEnforcer = new RealTimeRateEnforcer();
   private boolean isPlaying = false;
   private boolean fastSimulate = false;
   private int numberOfTicksBeforeUpdatingGraphs = 15;
   private int ticksToSimulate = 0;

   private ArrayList<PlaybackListener> playbackListeners = null;
   private ArrayList<PlayCycleListener> playCycleListeners = null;

   private Simulation mySimulation;
   private Robot[] robots;
   private final SimulationSynchronizer simulationSynchronizer;

   private ArrayList<YoGraphicsListRegistry> yoGraphicListRegistries = new ArrayList<YoGraphicsListRegistry>();
   private DataBuffer myDataBuffer;
   private boolean defaultLoaded = false;
   private int lastIndexPlayed = 0;

   private String runningName = ThreadTools.getBaseSimpleClassName();

   private ScsPhysics physics;

   private final SimulationConstructionSetParameters parameters;

   private final DynamicGraphicMenuManager dynamicGraphicMenuManager;

   public static SimulationConstructionSet generateSimulationFromDataFile(File chosenFile)
   {
      // / get file stuff
      System.out.println("Reading in Robot Configuration");
      RobotDefinitionFixedFrame robotConfig = new RobotDefinitionFixedFrame();
      robotConfig.createRobotDefinitionFromRobotConfigurationFile(chosenFile);

      // make a robot
      System.out.println("Creating a robot from the configuration.");
      Robot robot = new Robot(robotConfig, robotConfig.getRobotName());

      System.out.println("Creating a Simulation Construction Set.");
      SimulationConstructionSet sim = new SimulationConstructionSet(robot);

      System.out.println("Reading in the data.");

      if (chosenFile.canRead()
            && (chosenFile.getName().endsWith(".data") || chosenFile.getName().endsWith(".data.gz") || chosenFile.getName().endsWith(".data.csv")))
      {
         sim.readData(chosenFile);

      }

      else if (chosenFile.canRead() && (chosenFile.getName().endsWith(".state") || chosenFile.getName().endsWith(".state.gz")))
      {
         sim.readState(chosenFile);

      }

      return sim;
   }

   /**
    * Creates a SimulationConstructionSet with the specified Robot.
    * The GUI will be displayed and the initial DataBuffer size will be set to the default size.
    *
    * @param robot Robot to simulate.
    */
   public SimulationConstructionSet(Robot robot)
   {
      this(robot, new SimulationConstructionSetParameters());
   }

   public SimulationConstructionSet(Robot[] robots)
   {
      this(robots, new SimulationConstructionSetParameters());
   }


   /**
    * Creates a SimulationConstructionSet with no Robot.
    */
   public SimulationConstructionSet()
   {
      this(new SimulationConstructionSetParameters());
   }

   public SimulationConstructionSet(SimulationConstructionSetParameters parameters)
   {
      this((Robot) null, parameters);
   }

   /**
    * Creates a SimulationConstructionSet with the specified Robot and SimulationConstructionSetParameters parameters.
    * If parameters createGUI is true, the GUI will be displayed. If it is false, the GUI will not be displayed.
    * It is possible to run simulations without displaying the GUI, which can be useful when attempting to run
    * several simulations as a batch.
    *
    * @param robot               Robot to simulate.
    */
   public SimulationConstructionSet(Robot robot, SimulationConstructionSetParameters parameters)
   {
      this(new Robot[] { robot }, parameters);
   }

   public SimulationConstructionSet(Robot[] robotArray, SimulationConstructionSetParameters parameters)
   {
      this(new Simulation(robotArray, parameters.getDataBufferSize()), SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(parameters.getCreateGUI()), parameters);
   }

   public SimulationConstructionSet(Robot robot, Graphics3DAdapter graphicsAdapter, SimulationConstructionSetParameters parameters)
   {
      this(new Robot[] { robot }, graphicsAdapter, parameters);
   }

   public SimulationConstructionSet(Robot[] robotArray, Graphics3DAdapter graphicsAdapter, SimulationConstructionSetParameters parameters)
   {
      this(new Simulation(robotArray, parameters.getDataBufferSize()), graphicsAdapter, parameters);
   }

   public SimulationConstructionSet(Robot[] robotArray, SupportedGraphics3DAdapter supportedGraphicsAdapter, SimulationConstructionSetParameters parameters)
   {
      this(new Simulation(robotArray, parameters.getDataBufferSize()), supportedGraphicsAdapter.instantiateGraphics3DAdapter(), parameters);
   }

   public SimulationConstructionSet(Simulation simulation, SimulationConstructionSetParameters parameters)
   {
      this(simulation, SupportedGraphics3DAdapter.instantiateDefaultGraphicsAdapter(parameters.getCreateGUI()), parameters);
   }

   public SimulationConstructionSet(Simulation simulation, final Graphics3DAdapter graphicsAdapter, SimulationConstructionSetParameters parameters)
   {
      this.parameters = parameters;
      standardAllCommandsExecutor = new StandardAllCommandsExecutor();

      final boolean showGUI = ((graphicsAdapter != null) && (parameters.getCreateGUI()));
      this.rootRegistry = new YoVariableRegistry(rootRegistryName);

      if (showGUI)
      {
         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override
            public void run()
            {
               createFrame(showGUI);
            }
         });
         this.dynamicGraphicMenuManager = new DynamicGraphicMenuManager();
      }
      else
      {
         this.dynamicGraphicMenuManager = null;
      }

      mySimulation = simulation;
      this.myDataBuffer = mySimulation.getDataBuffer();
      this.simulationSynchronizer = mySimulation.getSimulationSynchronizer();

      ArrayList<YoVariable<?>> originalRootVariables = rootRegistry.getAllVariablesIncludingDescendants();
      try
      {
         for (YoVariable<?> yoVariable : originalRootVariables)
         {
            System.out.println("Original Variable: " + yoVariable);
         }

         this.myDataBuffer.addVariables(originalRootVariables);
      }
      catch (RepeatDataBufferEntryException e)
      {
         e.printStackTrace();

         throw new RuntimeException("Repeat Data Buffer Exception " + e);
      }

      setupVarGroup("all", new String[0], new String[] { ".*" });

      recomputeTiming();
      this.robots = mySimulation.getRobots();

      if (robots != null)
      {
         for (Robot robot : robots)
         {
            rootRegistry.addChild(robot.getRobotsYoVariableRegistry());
         }
      }

      createAndAttachAChangedListenerToTheRootRegistry();

      if (showGUI)
      {
         EventDispatchThreadHelper.invokeAndWait(new Runnable()
         {
            @Override
            public void run()
            {
               createGUI(graphicsAdapter);
            }
         });
      }

      mySimulation.getDataBuffer().copyValuesThrough(); // Copy the values through so that anything the user changed during initialization will be YoVariablized, and the default on all graphs.

      attachPlaybackListener(new PlaybackListener() {

		@Override
		public void indexChanged(int newIndex, double newTime) {


		}

		@Override
		public void stop() {
			if (myGUI != null && myGUI.getGraphics3dAdapter() != null)
				myGUI.getGraphics3dAdapter().pause();

		}

		@Override
		public void play(double realTimeRate) {
			if (myGUI != null && myGUI.getGraphics3dAdapter() != null)
				myGUI.getGraphics3dAdapter().play();

		}
	});
      if (robots != null)
      {
         for (Robot robot : robots)
         {
            for (YoGraphicsListRegistry yoGraphicsListRegistry : robot.yoGraphicsListRegistries)
            {
               addYoGraphicsListRegistry(yoGraphicsListRegistry);
            }
         }
      }

      if ((myGUI != null) && (robots != null))
      {
         for (Robot robot : robots)
         {
            ArrayList<Graphics3DObject> staticLinkGraphics = robot.getStaticLinkGraphics();
            myGUI.addStaticLinkGraphics(staticLinkGraphics);
         }
      }

      standardAllCommandsExecutor.setup(this, myGUI, myDataBuffer);
   }

   private void createAndAttachAChangedListenerToTheRootRegistry()
   {
      YoVariableRegistryChangedListener listener = new YoVariableRegistryChangedListener()
      {
         @Override
         public void yoVariableWasRegistered(YoVariableRegistry registry, YoVariable<?> registeredYoVariable)
         {
            // System.err.println("Registering YoVariable to the SCS root Registry after the SCS has been started! yoVariableWasRegistered: "
            // + registeredYoVariable);

            // Make sure RCS still works with all of this!
            try
            {
               myDataBuffer.addVariable(registeredYoVariable);
            }
            catch (RepeatDataBufferEntryException exception)
            {
               System.err.println("Already added to the dataBuffer. Not going to add it again!");
            }
         }

         @Override
         public void yoVariableRegistryWasCleared(YoVariableRegistry clearedYoVariableRegistry)
         {
            throw new RuntimeException("Clearing the SCS root Registry after the SCS has been started! Probably shouldn't do that...");
         }

         @Override
         public void yoVariableRegistryWasAdded(YoVariableRegistry addedRegistry)
         {
            // System.err.println("Adding a child YoVariableRegistry to the SCS root Registry after the SCS has been started! yoVariableRegistryWasAdded: "
            // + addedRegistry);

            try
            {
               myDataBuffer.addVariables(addedRegistry.getAllVariablesIncludingDescendants());
            }
            catch (RepeatDataBufferEntryException exception)
            {
               System.err.println("Already added to the dataBuffer. Not going to add it again!");
            }

            if (myGUI != null)
            {
               myGUI.updateNameSpaceHierarchyTree();
            }
         }
      };

      this.rootRegistry.attachYoVariableRegistryChangedListener(listener);
   }

   public SimulationConstructionSet(Robot robot, JApplet jApplet, SimulationConstructionSetParameters parameters)
   {
      this(new Robot[] { robot }, jApplet, parameters);
   }

   /**
    * Creates a SimulationConstructionSet with the specified Robot. The GUI will
    * be displayed using the specified JApplet and the initial DataBuffer size
    * will be set to the default size.
    *
    * @param robots  Robots to simulate.
    * @param jApplet JApplet to display the GUI in.
    */
   public SimulationConstructionSet(Robot[] robots, JApplet jApplet, SimulationConstructionSetParameters parameters)
   {
      this(robots, parameters);

      this.jFrame = null;
      this.jApplet = jApplet;

      throw new RuntimeException("Not sure if this is all we have to do for jApplet. Problably not. We need to test and debug this constructor...");
   }

   /**
    * Get the SimulationConstructionSetParameters that this simulation was created with
    *
    * @return SimulationConstructionSetParameters
    */
   public SimulationConstructionSetParameters getSimulationConstructionSetParameters()
   {
      return parameters;
   }

   /**
    * Get the simulation step size in seconds.
    *
    * @return Simulation step size.
    */
   public double getDT()
   {
      return mySimulation.getDT();
   }

   /**
    * Get the current simulation index (tick).
    *
    * @return Simulation index
    */
   @Override
   public int getIndex()
   {
      return myDataBuffer.getIndex();
   }

   @Override
   public boolean isIndexBetweenInAndOutPoint(int indexToCheck)
   {
      return myDataBuffer.isIndexBetweenInAndOutPoint(indexToCheck);
   }

   /**
    * Sets the simulation step size and the frequency to record data.
    *
    * @param simulateDT      Simulation step size in seconds.
    * @param recordFrequency Rate to record data in simulation step per records
    */
   public void setDT(double simulateDT, int recordFrequency)
   {
      mySimulation.setDT(simulateDT, recordFrequency);
      recomputeTiming();
   }

   /**
    * Retrieves the Robots used in this simulation.
    *
    * @return Robots contained in this simulation
    */
   public Robot[] getRobots()
   {
      return robots;
   }

   public StandardSimulationGUI getStandardSimulationGUI()
   {
      return myGUI;
   }

   /**
    * Add the specified script to this simulation.
    *
    * @param script Script to be added
    * @see us.ihmc.simulationconstructionset.scripts.Script
    */
   public void addScript(Script script)
   {
      mySimulation.addScript(script);
   }

   /**
    * Set the frame's size to the specified dimension.
    *
    * @param dimension Dimension to which the frame is resized.
    */
   public void setFrameSize(Dimension dimension)
   {
      jFrame.setSize(dimension);
   }

   /**
    * Move the upper left corner of the frame to the specified x & y coordinates.
    *
    * @param x Coordinate on the x axis
    * @param y Coordinate on the y axis
    */
   public void setFrameLocation(int x, int y)
   {
      jFrame.setLocation(x, y);
   }

   /**
    * Maximize the frame in both the horizontal and vertical directions.
    */
   public void setFrameMaximized()
   {
      jFrame.setExtendedState(Frame.MAXIMIZED_BOTH);
   }

   /**
    * Retrieve the frame of this SimulationConstructionSet.
    *
    * @return The JFrame containing the GUI
    */
   public JFrame getJFrame()
   {
      return jFrame;
   }

   /**
    * Specify if the frame should be "always on top" preventing it from being hidden
    * behind other windows.  This is false by default.
    *
    * @param alwaysOnTop boolean specifying if the frame is always on top.
    */
   public void setFrameAlwaysOnTop(boolean alwaysOnTop)
   {
      jFrame.setAlwaysOnTop(alwaysOnTop);
   }

   public YoVariableRegistry getRootRegistry()
   {
      return this.rootRegistry;
   }

   public void addYoVariableRegistry(YoVariableRegistry registry)
   {
      if (registry == null)
      {
         throw new RuntimeException("Cannot add a null registry to SCS!");
      }

      rootRegistry.addChild(registry);
   }

   @Override
   public ArrayList<YoVariable<?>> getAllVariables()
   {
      return mySimulation.getAllVariables();
   }

   @Override
   public YoVariable<?>[] getAllVariablesArray()
   {
      return mySimulation.getAllVariablesArray();
   }

   // Every time you call this in a control system, an angel loses its wings. Only call this for reflection and testing type purposes, such as
   // trying to compare if two simulations ran the same way.
   // For control systems, write a method to get the specific variable you need. Saves tons of work when refactoring later
   @Override
   public YoVariable<?> getVariable(String varname)
   {
      return mySimulation.getVariable(varname);
   }

   @Override
   public YoVariable<?> getVariable(String nameSpace, String varname)
   {
      return mySimulation.getVariable(nameSpace, varname);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(String nameSpace, String varname)
   {
      return mySimulation.getVariables(nameSpace, varname);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(String varname)
   {
      return mySimulation.getVariables(varname);
   }

   @Override
   public ArrayList<YoVariable<?>> getVariables(NameSpace nameSpace)
   {
      return mySimulation.getVariables(nameSpace);
   }

   @Override
   public boolean hasUniqueVariable(String varname)
   {
      return mySimulation.hasUniqueVariable(varname);
   }

   @Override
   public boolean hasUniqueVariable(String nameSpace, String varname)
   {
      return mySimulation.hasUniqueVariable(nameSpace, varname);
   }

   /**
    * Retrieves an ArrayList containing the YoVariables whos names contain the search string. If none exist, it returns null.
    *
    * @param searchString  String for which YoVariable names are checked.
    * @param caseSensitive Indicates if the search is to be case sensitive.
    * @return ArrayList of the YoVariables whos names contained searchString.
    */
   public ArrayList<YoVariable<?>> getVariablesThatContain(String searchString, boolean caseSensitive)
   {
      return mySimulation.getVariablesThatContain(searchString, caseSensitive);
   }

   /**
    * Retrieves an ArrayList containing the YoVariables with names that contain searchString.  If none exist, it returns null.
    * This method assumes the string is case insensitive.
    *
    * @param searchString String for which YoVariable names are checked.
    * @return ArrayList of the YoVariables whos names contained searchString.
    */
   public ArrayList<YoVariable<?>> getVariablesThatContain(String searchString)
   {
      return mySimulation.getVariablesThatContain(searchString, false);
   }

   /**
    * Retrieves an ArrayList containing the YoVariables with names that start with the searchString.  If none exist, it returns null.
    *
    * @param searchString String for which YoVariable names are checked.
    * @return ArrayList of the YoVariables whos names begin with searchString.
    */
   public ArrayList<YoVariable<?>> getVariablesThatStartWith(String searchString)
   {
      return mySimulation.getVariablesThatStartWith(searchString);
   }

   /**
    * Given an array of YoVariable names and an array of regular expressions this function returns an ArrayList of the YoVariables whos name's fit the regular expression.
    * If a given variable fits multiple expressions it will be added multiple times.
    *
    * @param varNames           String array of the name of YoVariables to be checked.
    * @param regularExpressions String array of regular expressions to use.
    * @return ArrayList of the YoVariables which have names that match the provided regular expressions.
    */
   public ArrayList<YoVariable<?>> getVars(String[] varNames, String[] regularExpressions)
   {
      return mySimulation.getVars(varNames, regularExpressions);
   }

   /**
    * The time in seconds between each recorded data point.  This value is ultimately stored as the ratio of simulation steps per record.
    * If the specified value is not evenly divisible by the simulation time step rounding will occur.
    * Once the value has been updated timing is recomputed.
    *
    * @param recordDT The new period, in seconds, between recorded data points.
    */
   public void setRecordDT(double recordDT)
   {
      mySimulation.setRecordDT(recordDT);
      recomputeTiming();
   }

   /**
    * Gets the Record Frequency for the simulation.
    *
    * @return Record Frequencuy.
    */
   public long getRecordFreq()
   {
      return mySimulation.getRecordFreq();
   }

   public double getTimePerRecordTick()
   {
      double simulationDT = this.getDT();
      long recordFrequency = this.getRecordFreq();
      double timePerRecordTick = ((double) recordFrequency) * simulationDT;
      return timePerRecordTick;
   }

   /**
    * Sets the realTimeRate.  This value is a percentage with 1.0 yielding 100% of standard time.  Greater values increase rate of time whereas smaller values decrease it.
    *
    * @param realTimeRate The desired playback rate in percentage of real time where 100% is specified as 1.0.
    */
   @Override
   public void setPlaybackRealTimeRate(double realTimeRate)
   {
      REAL_TIME_RATE = realTimeRate;
      recomputeTiming();
   }

   /**
    * Sets the desired playback frame rate in seconds per frame.  For example 0.1 would yield 10 frames/second.
    * The maximum frame rate is 500 or 0.002 seconds per frame while the minimum is 0.5 or 2.0 seconds per frame.
    * Once the value has been updated timing is recomputed.
    *
    * @param frameRate The new frame rate in seconds per frame.
    */
   public void setPlaybackDesiredFrameRate(double frameRate)
   {
      if (frameRate < 0.002)
      {
         frameRate = 0.002;
      }

      if (frameRate > 2.0)
      {
         frameRate = 2.0;
      }

      SECONDS_PER_FRAME = frameRate;
      recomputeTiming();
   }

   /**
    * Retrieve the current real time rate which is the percentage of real time the simulator is displaying.
    * 100% is represented as 1.0
    *
    * @return The current playback rate as a percentage of real time.
    */
   @Override
   public double getPlaybackRealTimeRate()
   {
      return REAL_TIME_RATE;
   }

   /**
    * Retrieve the playback frame rate in seconds per frame.
    *
    * @return The playback frame rate.
    */
   public double getPlaybackFrameRate()
   {
      return SECONDS_PER_FRAME;
   }

   /**
    * This method recomputes the internal timing related parameters.  Any time values that effect timing are changed this method is called to update all effected parameters.
    */
   public void recomputeTiming()
   {
      double dt = mySimulation.getDT();
      double recordFreq = mySimulation.getRecordFreq();

      TICKS_PER_PLAY_CYCLE = Math.max((int) (SECONDS_PER_FRAME * REAL_TIME_RATE / (dt * recordFreq)), 1);
      SECONDS_PER_FRAME = TICKS_PER_PLAY_CYCLE * (dt * recordFreq) / REAL_TIME_RATE;
      PLAY_CYCLE_TIME_MS = (long) (1000 * SECONDS_PER_FRAME);
   }

   public long getTicksPerPlayCycle()
   {
      return TICKS_PER_PLAY_CYCLE;
   }

   /**
    * Adds a static Link to the display environment. If the GUI does not exist the add fails and null is returned.
    * <p/>
    * Static links are purely cosmetic.
    *
    * @param staticLink The Link to be added statically to the environment.
    */
   public Graphics3DNode  addStaticLink(Link staticLink)
   {
      return addStaticLinkGraphics(staticLink.getLinkGraphics());
   }

   public Graphics3DNode addStaticLinkGraphics(Graphics3DObject staticLinkGraphics)
   {
      if (myGUI != null)
      {
         return myGUI.addStaticLinkGraphics(staticLinkGraphics);
      }
      else
      {
         return null;
      }
   }

   /**
    * Adds a static LinkGraphics to the display environment. If the GUI does not exist the add fails and null is returned.
    * <p/>
    * Static LinkGraphics are purely cosmetic.
    *
    * @param staticLinkGraphics The LinkGraphics to be added statically to the environment.
    */
   public Graphics3DNode addStaticLinkGraphics(Graphics3DObject staticLinkGraphics, Graphics3DNodeType nodeType)
   {
      if (myGUI != null)
      {
         return myGUI.addStaticLinkGraphics(staticLinkGraphics, nodeType);
      }
      else
      {
         return null;
      }
   }

   public void removeGraphics3dNode(Graphics3DNode nodeToRemove)
   {
      if (myGUI != null)
      {
         myGUI.removeStaticGraphics3dNode(nodeToRemove);
      }
   }

   public ArrayList<Graphics3DNode> addStaticLinkGraphics(ArrayList<Graphics3DObject> staticLinkGraphics)
   {
      ArrayList<Graphics3DNode> ret = new ArrayList<>(staticLinkGraphics.size());
      for (Graphics3DObject linkGraphics : staticLinkGraphics)
      {
         ret.add(addStaticLinkGraphics(linkGraphics));
      }

      return ret;
   }

   /**
    * This function modifies the camera tracking state for the selected viewport.  Tracking can be enabled or disabled in a general sense as well as in a specific axis.
    * For example, assuming track were enabled, if trackx were disabled the camera would not follow a target with a changing x co-ordinate.
    * <p/>
    * A camera set to track will not move instead it will rotate to keep the target in view.
    *
    * @param track  Enable or disable tracking for this camera.
    * @param trackX Enable or disable tracking in the x direction.
    * @param trackY Enable or disable tracking in the y direction.
    * @param trackZ Enable or disable tracking in the z direction.
    */
   public void setCameraTracking(boolean track, boolean trackX, boolean trackY, boolean trackZ)
   {
      if (myGUI != null)
      {
         myGUI.setCameraTracking(track, trackX, trackY, trackZ);
      }
   }

   /**
    * This function modifies the camera dolly state for the selected viewport.  Dolly can be enabled or disabled in a general sense and in terms of specific axis.
    * For example, if dolly were enabled overall but disabled in the x direction the camera would not follow if its target moved on the x axis.
    * <p/>
    * A camera with dolly enabled will move to keep its target in view from the same orientation.
    *
    * @param dolly  Enable or disable dolly for this camera.
    * @param dollyX Enable or disable dolly in the x direction.
    * @param dollyY Enable or disable dolly in the y direction.
    * @param dollyZ Enable or disable dolly in the z direction.
    */
   public void setCameraDolly(boolean dolly, boolean dollyX, boolean dollyY, boolean dollyZ)
   {
      if (myGUI != null)
      {
         myGUI.setCameraDolly(dolly, dollyX, dollyY, dollyZ);
      }
   }

   /**
    * Sets the camera tracking variables for the active viewport.  These variables control what the camera tracks when tracking is enabled.
    * By default the camera is set to track the Robot's x, y and z position if it exists.
    *
    * @param xName Name of the YoVariable to be referenced for x direction tracking.
    * @param yName Name of the YoVariable to be referenced for y direction tracking.
    * @param zName Name of the YoVariable to be referenced for z direction tracking.
    */
   public void setCameraTrackingVars(String xName, String yName, String zName)
   {
      if (myGUI != null)
      {
         myGUI.setCameraTrackingVars(xName, yName, zName);
      }
   }

   /**
    * Sets the camera dolly variables for the active viewport.  These variables control what the camera follows when dolly is enabled.
    * By default the camera is set to follow the Robot's x, y and z position if it exists.
    *
    * @param xName Name of the YoVariable to be referenced for x direction following.
    * @param yName Name of the YoVariable to be referenced for y direction following.
    * @param zName Name of the YoVariable to be referenced for z direction following.
    */
   public void setCameraDollyVars(String xName, String yName, String zName)
   {
      if (myGUI != null)
      {
         myGUI.setCameraDollyVars(xName, yName, zName);
      }
   }

   /**
    * Configures the offsets for tracking on the active camera.  These offsets are added to the tracking variables which specify the targets location to calculate the camera's focal point.  By default the offsets are zero.
    * <p/>
    * In tracking mode the camera is fixed but remains focused on the target.
    *
    * @param dx Offset in the x direction from the target.
    * @param dy Offset in the y direction from the target.
    * @param dz Offset in the z direction from the target.
    * @see #setCameraTrackingVars(String, String, String) setCameraTrackingVars
    */
   public void setCameraTrackingOffsets(double dx, double dy, double dz)
   {
      if (myGUI != null)
      {
         myGUI.setCameraTrackingOffsets(dx, dy, dz);
      }
   }

   /**
    * Configures the offset at which the active camera follows the target.  These offsets are added to the dolly variables which specify the location of the target.
    * By default there is an offset of 2.0 in the x direction and 12.0 in the y.
    * <p/>
    * In dolly mode the camera has a fixed orientation and moves to keep a constant view of the target.
    *
    * @param dx Offset in the x direction from the target.
    * @param dy Offset in the y direction from the target.
    * @param dz Offset in the z direction from the target.
    * @see #setCameraDollyVars(String, String, String) setCameraDollyVars
    */
   public void setCameraDollyOffsets(double dx, double dy, double dz)
   {
      if (myGUI != null)
      {
         myGUI.setCameraDollyOffsets(dx, dy, dz);
      }
   }

   /**
    * Modify the active camera's fix.  If track is enabled for this camera the new values will be overwritten, dolly has no effect.
    * <p/>
    * The camera fix is point in space at which the camera is looking.
    *
    * @param fixX X coordinate of the fix point.
    * @param fixY Y coordinate of the fix point.
    * @param fixZ Z coordinate of the fix point.
    */
   public void setCameraFix(double fixX, double fixY, double fixZ)
   {
      if (myGUI != null)
      {
         myGUI.setCameraFix(fixX, fixY, fixZ);
      }
   }

   /**
    * Modify the active camera's fix.  If track is enabled for this camera the new values will be overwritten, dolly has no effect.
    * <p/>
    * The camera fix is point in space at which the camera is looking.
    *
    * @param cameraFix coordinates of the fix point.
    */
   public void setCameraFix(Tuple3d cameraFix)
   {
      if (myGUI != null)
      {
         myGUI.setCameraFix(cameraFix);
      }
   }

   /**
    * Modifies the position at which the currently active camera is located.  If dolly is enabled for this camera the new values will be overwritten, track has no effect.
    *
    * @param posX X coordinate of the camera.
    * @param posY Y coordinate of the camera.
    * @param posZ Z coordinate of the camera.
    */
   public void setCameraPosition(double posX, double posY, double posZ)
   {
      if (myGUI != null)
      {
         myGUI.setCameraPosition(posX, posY, posZ);
      }
   }

   /**
    * Modifies the position at which the currently active camera is located.  If dolly is enabled for this camera the new values will be overwritten, track has no effect.
    *
    * @param cameraPosition coordinates of the camera.
    */
   public void setCameraPosition(Tuple3d cameraPosition)
   {
      if (myGUI != null)
      {
         myGUI.setCameraPosition(cameraPosition);
      }      
   }

   /**
    * Add another ExitActionListener to the set which is triggered by closing the simulation environment GUI.
    *
    * @param listener The ExitActionListener to be added.
    */
   public void attachExitActionListener(ExitActionListener listener)
   {
      if (myGUI != null)
      {
         myGUI.attachExitActionListener(listener);
      }
   }

   public void notifyExitActionListeners()
   {
      if (myGUI != null)
      {
         myGUI.notifyExitActionListeners();
      }
   }

   public void startOnAThread()
   {
      ThreadTools.startAThread(this, "Simulation Contruction Set");

      while (!this.isSimulationThreadUpAndRunning())
      {
         Thread.yield();
      }
   }

   @Override
   public void closeAndDispose()
   {
      if(myGUI != null)
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
      else
      {
         closeAndDisposeLocal();
      }
   }

   private void closeAndDisposeLocal()
   {
      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Stopping Simulation Thread");

      System.out.flush();
      stop();
      stopSimulationThread();

      if (jFrame != null)
      {
         jFrame.setVisible(false);
         jFrame.dispose();
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Notifying Exit Action Listeners");

      System.out.flush();
      notifyExitActionListeners();

      if (rootRegistry != null)
      {
         rootRegistry.closeAndDispose();
      }


      if (standardAllCommandsExecutor != null)
      {
         standardAllCommandsExecutor.closeAndDispose();
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Disposing StandardSimulationGUI");

      System.out.flush();

      if (myGUI != null)
      {
         if (DEBUG_CLOSE_AND_DISPOSE)
            System.out.println("Disposing StandardSimulationGUI and myGUI != null ");

         myGUI.closeAndDispose();
      }

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Disposing Simulation");//Doesn't reach this in some cases if I add a graph and add t to it!

      System.out.flush();

      if (mySimulation != null)
      {
         mySimulation.closeAndDispose();
      }



      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Disposing of JFrame");

      System.out.flush();

      if (DEBUG_CLOSE_AND_DISPOSE)
         System.out.println("Done Closing and Disposing SCS.");

      System.out.flush();

      rootRegistry = null;
      standardAllCommandsExecutor = null;
      myGUI = null;
      myDataBuffer = null;
      robots = null;
      mySimulation = null;
      jFrame = null;

      // Destroy the LWJGL Threads. Not sure if need to do Display.destroy() or not.
//      Display.destroy();
      ThreadTools.interruptLiveThreadsExceptThisOneContaining("LWJGL Timer"); // This kills the silly LWJGL sleeping thread which just sleeps and does nothing else...

      GlobalTimer.clearTimers();


   }

   /**
    * This function adds the specified button to the SCS GUI.  The button is added to a panel in the upper right corner to the left of the existing simulate, play, pause buttons.
    *
    * @param button JButton to be added.
    */
   public void addButton(AbstractButton button)
   {
      if (myGUI != null)
      {
         myGUI.addButton(button);
      }
   }

   public void addComboBox(JComboBox<?> comboBox)
   {
      if (myGUI != null)
      {
         myGUI.addComboBox(comboBox);
      }
   }

   public void addJLabel(JLabel label)
   {
      if (myGUI != null)
      {
         myGUI.addJLabel(label);
      }
   }

   public void addTextField(JTextField textField)
   {
      if (myGUI != null)
      {
         myGUI.addTextField(textField);
      }
   }

   /**
    * This function adds the specified button to the SCS GUI.  The button is added to a panel in the upper right corner to the right of the existing simulate, play, pause buttons.
    *
    * @param button JRadioButton to be added.
    */
   public void addRadioButton(JRadioButton button)
   {
      if (myGUI != null)
      {
         myGUI.addRadioButton(button);
      }
   }

   /**
    * This function adds the specified button to the SCS GUI.  The button is added to a panel in the upper right corner to the right of the existing simulate, play, pause buttons.
    *
    * @param checkBox JCheckBox to be added.
    */
   public void addCheckBox(JCheckBox checkBox)
   {
      if (myGUI != null)
      {
         myGUI.addCheckBox(checkBox);
      }
   }

   /**
    * This function adds the specified button to the SCS GUI.  The button is added to a panel in the upper right corner to the right of the existing simulate, play, pause buttons.
    *
    * @param menuBar JMenuBar to be added.
    */
   public void addMenuBar(JMenuBar menuBar)
   {
      if (myGUI != null)
      {
         myGUI.addMenuBar(menuBar);
      }
   }

   /**
    * Sets time and then increments the data buffer index and updates all of the entries min & max values.  If a GUI exists, its graphs are updated.
    */
   @Override
   public void tickAndUpdate(double timeToSetInSeconds)
   {
      this.setTime(timeToSetInSeconds);
      this.tickAndUpdate();
   }
   
   /**
    * Increments the data buffer index and updates all of the entries min & max values.  If a GUI exists, its graphs are updated.
    */
   @Override
   public void tickAndUpdate()
   {
      mySimulation.tickAndUpdate();

      if (myGUI != null)
      {
         if ((!fastSimulate))
         {
            myGUI.updateGraphs(); // If the GUI exists and fast simulate is disabled update graphs
         }
         else
         {
            // Prevents the graphs from being updated every record cycle to speed up simulation
            fastTicks++;

            if (fastTicks > numberOfTicksBeforeUpdatingGraphs)
            {
               fastTicks = 0;

               myGUI.updateGraphs();
            }
         }
         myGUI.updateSimulationGraphics();
      }
   }

   /**
    * Increments the data buffer index and updates all of the min and max values for each entry.  If a GUI exists, its graphs are updated after this method has been called the number of times specified by lesiureRate.
    * The value of leisureRate is not stored internally and may be changed at every call.  If the method has been called more than leisureRate times the graphs are updated and the count is reset.
    *
    * @param leisureRate iNumber of calls before the graphs should be updated.
    */
   public void tickAndUpdateLeisurely(int leisureRate)
   {
      mySimulation.tickAndUpdate();

      if (myGUI != null)
      {
         myGUI.updateGraphsLeisurely(leisureRate);
      }
   }

   /**
    * This method updates the min and max values for each entry in the data buffer. It then attempts to increment the index however, it will roll around the in and out points.  Once the final index is reached the entries are re-updated.
    * If a GUI exists the graphs will be updated.
    *
    * @return Did the index roll around the in and out points?
    */
   public boolean updateAndTick()
   {
      boolean ret = myDataBuffer.updateAndTick();
      if (myGUI != null)
      {
         myGUI.updateGraphs();
         myGUI.updateSimulationGraphics();
      }

      return ret;
   }

   /**
    * Triggers a tick to the next display cycle.  The data buffer is incremented to the data point for the next display frame.  The function returns true if this triggers a rollover between the in and out points.
    * Once this completes the robot is updated based on those values without updating the display. (This probably deals with playback only, simulation is separate)
    *
    * @return Did the data buffer roll between the in and out points?
    */
   public boolean tick()
   {
      boolean ret = myDataBuffer.tick((int) TICKS_PER_PLAY_CYCLE);

      for (Robot robot : robots)
      {
         robot.updateForPlayback();
      }

      // camera.update();
      if (myGUI != null)
      {
         myGUI.updateGraphs();
         myGUI.updateSimulationGraphics();
      }

      // +++JEP: Only let the pre-renderer update the robot graphics!! rob.updateGraphics();

      return ret;
   }

   public boolean unTick()
   {
      boolean ret = myDataBuffer.tick((int) -TICKS_PER_PLAY_CYCLE);

      for (Robot robot : robots)
      {
         robot.updateForPlayback();
      }

      // camera.update();
      if (myGUI != null)
      {
         myGUI.updateGraphs();
         myGUI.updateSimulationGraphics();
      }

      // +++JEP: Only let the pre-renderer update the robot graphics!! rob.updateGraphics();

      return ret;
   }

   @Override
   public boolean tickButDoNotNotifySimulationRewoundListeners(int ticks)
   {
      return tick(ticks, false);
   }

   @Override
   public boolean tick(int ticks)
   {
      return tick(ticks, true);
   }

   /**
    * Triggers a tick to the next display cycle. The data buffer is incremented
    * to the data point for the next display frame. The function returns true
    * if this triggers a rollover between the in and out points. Once this
    * completes the robot is updated based on those values without updating the
    * display. (This probably deals with playback only, simulation is separate)
    *
    * @param ticks The amount to tick.
    * @return Did the data buffer roll between the in and out points?
    */
   private boolean tick(int ticks, boolean notifySimulationRewoundListeners)
   {
      boolean ret;

      if (notifySimulationRewoundListeners)
         ret = myDataBuffer.tick(ticks);
      else
         ret = myDataBuffer.tickButDoNotNotifySimulationRewoundListeners(ticks);

      for (Robot robot : robots)
      {
         robot.updateForPlayback();
      }

      if (myGUI != null)
      {
         myGUI.updateGraphs();
         myGUI.updateSimulationGraphics();
      }

      return ret;
   }

   public boolean setTick(int tick)
   {
      boolean ret = myDataBuffer.tick((int) (((double) tick) * TICKS_PER_PLAY_CYCLE));
      for (Robot robot : robots)
      {
         robot.updateForPlayback();
      }

      if (myGUI != null)
      {
         myGUI.updateGraphs();
         myGUI.updateSimulationGraphics();
      }

      return ret;
   }

   /**
    * Adds an entry box for the specified variable.  This only occurs if a GUI is present and the specified variable exists.
    *
    * @param varname The name of the desired variable.
    */
   public void setupEntryBox(String varname)
   {
      if (myGUI != null)
      {
         myGUI.setupEntryBox(varname);
      }
   }

   /**
    * Adds a set of entry boxes, one for each variable name provided, assuming the GUI is present.
    *
    * @param varnames Array containing the names of variables for which to add entry boxes.
    */
   public void setupEntryBox(String[] varnames)
   {
      if (myGUI != null)
      {
         myGUI.setupEntryBox(varnames);
      }
   }

   /**
    * Adds a graph for the specified variable, assuming the GUI and the variable exist.
    *
    * @param varname Name of the variable for which to add a graph.
    */
   public void setupGraph(String varname)
   {
      if (myGUI != null)
      {
         myGUI.setupGraph(varname);
      }
   }

   /**
    * Adds a single graph displaying each variable named in the array, assuming the GUI and variable exist.
    *
    * @param varnames Array of variable names for which to create graphs.
    */
   public void setupGraph(String[] varnames)
   {
      if (myGUI != null)
      {
         myGUI.setupGraph(varnames);
      }
   }

   /**
    * Creates a new group with the given name contaning the variables specified in the array.  VarGroups are displayed in the left most column, they list the names and values of the variables they contain.
    * Once a var group has been created it may be selected from the configuration drop down menu.  It may also be added to a configuration along with a graphGroup and entryBoxGroup.
    *
    * @param name Name of the group.
    * @param vars Names of the variables to include.
    */
   public void setupVarGroup(String name, String[] vars)
   {
      varGroupList.setupVarGroup(name, vars);

      if (myGUI != null)
      {
         myGUI.updateVarGroupList(varGroupList);
      }

      // if (myGUI != null) myGUI.setupVarGroup(name, vars);
   }

   /**
    * Creates a new VarGroup with the specified name, containing the specified variables and regular expressions.  The regular expressions can be used to retrive additonal variables which match their patterns.
    * Once a var group has been created it may be selected from the configuration drop down menu.  It may also be added to a configuration along with a graphGroup and entryBoxGroup.
    *
    * @param name               Name of the group
    * @param vars               Array containing the names of variables to add.
    * @param regularExpressions Array of regular expressions to include.
    */
   public void setupVarGroup(String name, String[] vars, String[] regularExpressions)
   {
      varGroupList.setupVarGroup(name, vars, regularExpressions);

      if (myGUI != null)
      {
         myGUI.updateVarGroupList(varGroupList);
      }

      // if (myGUI != null) myGUI.setupVarGroup(name, vars);
   }

   /**
    * <p>Creates a new group of graphs with the given name containing the given variables.  Once a graph group has been created it can be selected from the configuration drop down menu.
    * It may also be added to a configuration along with a varGroup and entryBoxGroup.</p>
    * <p/>
    * For example:<br />
    * {@code setupGraphGroup("states", new String[][]{{"left_state", "right_state"}, {"t"}, {"q_x", "q_z"}});}<br />
    * This code creates the following three graphs:<br />
    * 1) left_state, right_state<br />
    * 2) t<br />
    * 3) q_x, q_z
    *
    * @param name Name of the group.
    * @param vars Array containing the names of variables for which graphs are to be created.  Each row is treated as a separate graph and may contain multiple variables.
    */
   public void setupGraphGroup(String name, String[][] vars)
   {
      if (myGUI != null)
      {
         myGUI.setupGraphGroup(name, vars);
      }
   }

   /**
    * <p>Creates a new group of graphs with the given name containing the given variables using the specified configuration.  Once a graph group has been created it can be selected from the configuration drop down menu.
    * It may also be added to a configuration along with a varGroup and entryBoxGroup.</p>
    * <p/>
    * For example:<br />
    * {@code setupGraphGroup("states", new String[][]{{{"left_state", "right_state"}, {"config_1}}, {{"t"}, {"config_2"}}, {{"q_x", "q_z"}, {"config_3"}}});}<br />
    * This code creates the following three graphs:<br />
    * 1) left_state, right_state both using config_1<br />
    * 2) t using config_2<br />
    * 3) q_x, q_z both using config_3
    *
    * @param name The name of the graph group.
    * @param vars String array containing the YoVariable names to be added to each graph in the group as well as the configurations to use.
    */
   public void setupGraphGroup(String name, String[][][] vars)
   {
      if (myGUI != null)
      {
         myGUI.setupGraphGroup(name, vars);
      }
   }

   /**
    * <p>Creates a new group of graphs with the given name containing the given variables spread over the specified number of columns.  Once a graph group has been created it can be selected from the configuration drop down menu.
    * It may also be added to a configuration along with a varGroup and entryBoxGroup.</p>
    * <br />
    * For example:<br />
    * {@code setupGraphGroup("states", new String[][]{{"left_state", "right_state"}, {"t"}, {"q_x", "q_z"}}, 2);}<br />
    * This code creates the following three graphs divided over 2 columns:<br />
    * 1) left_state, right_state<br />
    * 2) t<br />
    * 3) q_x, q_z
    *
    * @param name       Name of the group.
    * @param vars       String array containing the YoVariable names to be added to each graph in the group.
    * @param numColumns Number of columns over which to spread the graphs.
    */
   public void setupGraphGroup(String name, String[][] vars, int numColumns)
   {
      if (myGUI != null)
      {
         myGUI.setupGraphGroup(name, vars, numColumns);
      }
   }

   /**
    * Creates a new group of graphs with the given name containing the given variables using the specified configuration and number of columns.  Once a graph group has been created it can be selected from the configuration drop down menu.
    * It may also be added to a configuration along with a varGroup and entryBoxGroup.
    * <p/>
    * For example:
    * {@code setupGraphGroup("states", new String[][]{{{"left_state", "right_state"}, {"config_1}}, {{"t"}, {"config_2"}}, {{"q_x", "q_z"}, {"config_3"}}});}
    * This code creates the following three graphs separated into two columns:
    * 1) left_state, right_state both using config_1
    * 2) t using config_2
    * 3) q_x, q_z both using config_3
    *
    * @param name       Name of the group.
    * @param vars       String array containing the YoVariable names to be added to each graph in the group as well as the configurations to use.
    * @param numColumns Number of columns over which to spread the graphs.
    */
   public void setupGraphGroup(String name, String[][][] vars, int numColumns)
   {
      if (myGUI != null)
      {
         myGUI.setupGraphGroup(name, vars, numColumns);
      }
   }

   /**
    * Creates a grouping of entry boxes using the provided YoVariable names.  Entry boxes appear below the graphs at the bottom of the Simulation Construction Set GUI.  Once a group is created it can be selected from the configuration drop down menu.
    * It may also be added to a configuration along with a varGroup and graphGroup.
    *
    * @param name Name of the group.
    * @param vars Array containing the names of the YoVariables to include.
    */
   public void setupEntryBoxGroup(String name, String[] vars)
   {
      if (myGUI != null)
      {
         myGUI.setupEntryBoxGroup(name, vars);
      }
   }

   /**
    * Creates a grouping of entry boxes using the provided YoVariable names and any YoVariables that match the specified regular expressions.  Entry boxes appear below the graphs at the bottom of the Simulation Construction Set GUI.  Once a group is created it can be selected from the configuration drop down menu.
    * It may also be added to a configuration along with a varGroup and graphGroup.
    *
    * @param name               Name of the group.
    * @param vars               Array containing the names of the YoVariables to include.
    * @param regularExpressions Array containing regular expressions to use in searching for additonal YoVariables.
    */
   public void setupEntryBoxGroup(String name, String[] vars, String[] regularExpressions)
   {
      if (myGUI != null)
      {
         myGUI.setupEntryBoxGroup(name, vars, regularExpressions);
      }
   }

   /**
    * Creates a configuration which associates a varGroup, graphGroup and entryBoxGroup so that they may be easily activated simultaniously.  Configurations are selected in the configuration drop down menu.
    *
    * @param config            Name of the new configuration.
    * @param varGroupName      Name of the varGroup to associate with this configuration.
    * @param graphGroupName    Name of the graphGroup to associate with this configuration.
    * @param entryBoxGroupName Name of the entryBoxGroup to associate with this configuration.
    */
   public void setupConfiguration(String config, String varGroupName, String graphGroupName, String entryBoxGroupName)
   {
      if (myGUI != null)
      {
         if ((varGroupName != null) && (graphGroupName != null) && (entryBoxGroupName != null))
         {
            myGUI.setupConfiguration(config, graphGroupName, entryBoxGroupName);
         }
      }
   }

   private boolean isDefaultFileExist()
   {
      File configs = new File("Configurations");

      if (!configs.exists())
      {
         boolean res = configs.mkdir();
         if (!res) { return false; }
      }

      String path = configs.toURI().getPath();
      File defaultConfiguration = new File(path + "defaultConfiguration.guiConf");

      return defaultConfiguration.exists();
   }

   /**
    * Makes the specified configuration active.
    *
    * @param name Name of the configuration to activate.
    */
   public void selectConfiguration(String name)
   {
      if ((myGUI != null) && !isDefaultFileExist())
      {
         myGUI.selectGraphConfiguration(name);
      }
   }

   /**
    * <p>Adds the specified camera configuration to the simulation. Once added the camera configuration may be selected in the viewport menu.  It may also be added to a predefined viewport.</p>
    * <p/>
    * <p>The following code creates a camera configuration named "camera 1" in track mode with a camera position of (-9,3,0.8) and an initial fix of (0,0,0.6)<br />
    * <br />
    * {@code CameraConfiguration camera1 = new CameraConfiguration("camera 1");}<br />
    * {@code camera1.setCameraFix(0.0, 0.0, 0.6);}<br />
    * {@code camera1.setCameraPosition(-9.0, 3.0, 0.8);}<br />
    * {@code camera1.setCameraTracking(true, true, true, false);}<br />
    * {@code camera1.setCameraDolly(false, true, true, false);}<br />
    * {@code sim.setupCamera(camera1);}<br />
    * </p>
    *
    * @param cameraConfiguration CameraConfiguration
    * @see us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration CameraConfiguration
    */
   public void setupCamera(CameraConfiguration cameraConfiguration)
   {
      if (myGUI != null)
      {
         myGUI.setupCamera(cameraConfiguration);
      }
   }

   public void setupExtraPanel(ExtraPanelConfiguration panelConfiguration)
   {
      if (myGUI != null)
      {
         myGUI.setupExtraPanels(panelConfiguration);
      }
   }

   /**
    * @return get the Extra Panel from the StandardSimulationGUI
    */
   public Component getExtraPanel(String name)
   {
      return myGUI.getExtraPanel(name);
   }

   /**
    * Makes the specified camera active in the active view.  This view can be identified by its red border.
    *
    * @param cameraName Name of the camera to select.
    */
   public void selectCamera(String cameraName)
   {
      if (myGUI != null)
      {
         myGUI.selectCamera(cameraName);
      }
   }

   /**
    * Adds the specified ViewportConfiguration to the simulation.  Once added, the configuration may be selected in the viewport window.
    *
    * @param viewportConfiguration ViewportConfiguration
    * @see ViewportConfiguration ViewportConfiguration
    */
   public void setupViewport(ViewportConfiguration viewportConfiguration)
   {
      if (myGUI != null)
      {
         myGUI.setupViewport(viewportConfiguration);
      }
   }

   /**
    * Activate the specified viewport configuration in the primary viewport panel.
    *
    * @param viewportName Name of the viewport to activate.
    */
   public void selectViewport(String viewportName)
   {
      if (myGUI != null)
      {
         myGUI.selectViewport(viewportName);
      }
   }

   /**
    * Store the specified array of GraphConfigurations.
    *
    * @param configurations Array of GraphConfigurations to store.
    */
   public void setupGraphConfigurations(GraphConfiguration[] configurations)
   {
      if (myGUI != null)
      {
         myGUI.setupGraphConfigurations(configurations);
      }
   }

   /**
    * Standard GUI display method.  This allows the application to be run as a Java Applet.
    *
    * @param showGUI Specify if the gui should be displayed.
    */
   private void createFrame(boolean showGUI)
   {
      if (showGUI)
      {
         try

         // +++JEP: For Applets to work...
         {
            if (parameters.getShowSplashScreen()) StandardSimulationGUI.showSplashScreen();
         }
         catch (Exception e)
         {
         }

         // try{Thread.sleep(2000);}catch(InterruptedException e){}
         // StandardSimulationGUI.disposeSplashWindow();
      }

      jFrame = new JFrame("Simulation Construction Set");
      jFrame.setIconImage(new ImageIcon(getClass().getClassLoader().getResource("running-man-32x32-Sim.png")).getImage());

      try
      {
         // TODO: These lines have to be here or get weird results with the Viewport being minimized!?
         JDialog dialog = new JDialog(jFrame, "", true);
         dialog.pack();
      }

      catch (Exception e)
      {
      }

      jFrame.repaint();
   }

   /**
    * Set the robot to be used by this simulation.
    *
    * @param robot Robot to be used by the simulation.
    */
   public void setRobot(Robot robot)
   {
      YoVariableRegistry robotsYoVariableRegistry = robot.getRobotsYoVariableRegistry();
      YoVariableRegistry parentRegistry = robotsYoVariableRegistry.getParent();
      if (parentRegistry != null)
      {
         throw new RuntimeException("SimulationConstructionSet.setRobot(). Trying to add robot registry as child to root registry, but it already has a parent registry: " + parentRegistry);
      }

      boolean notifyListeners = false; // TODO: This is very hackish. If listeners are on, then the variables will be added to the data buffer. But mySimulation.setRobots() in a few lines does that...
      rootRegistry.addChild(robotsYoVariableRegistry, notifyListeners);

      mySimulation.setRobots(new Robot[] { robot });

      // recomputeTiming();
      this.robots = mySimulation.getRobots();

      if (myGUI != null)
      {
         myGUI.setRobots(robots);

         if (robots != null)
         {
            for (Robot robotToAddToGUI : robots)
            {
               ArrayList<RewoundListener> simulationRewoundListeners = robotToAddToGUI.getSimulationRewoundListeners();
               for (RewoundListener simulationRewoundListener : simulationRewoundListeners)
               {
                  myDataBuffer.attachSimulationRewoundListener(simulationRewoundListener);
               }
            }

            // *** JJC a add variable search panel was removed from here.
         }
      }
   }

   /**
    * Internal method for GUI creation.  If frame does not exist the gui will be a Java Applet.
    */
   private void createGUI(Graphics3DAdapter graphicsAdapter)
   {
      // Create a standard GUI:
      if (jFrame != null)
      {
         myGUI = new StandardSimulationGUI(graphicsAdapter, simulationSynchronizer, standardAllCommandsExecutor, null, this, this, robots, myDataBuffer,
               varGroupList, jFrame, rootRegistry);
      }
      else
      {
         myGUI = new StandardSimulationGUI(graphicsAdapter, simulationSynchronizer, standardAllCommandsExecutor, null, this, this, robots, myDataBuffer,
               varGroupList, jApplet, rootRegistry);
      }

      // +++JEP: They don't seem to be getting added to the GUI... this.addVariablesToSimulationAndGUI(rootRegistry);
      // this.addVariablesToGUI(rootRegistry.createVarListsIncludingChildren());

      // TODO: Fix the variable adding after sim starts problem and write unit tests for it!
      // addVariablesToGUI(rootRegistry);

      HeightMap heightMap = null;

      // TODO: GroundProfile is just that of the first robot. Need to make it part of the sim or something...
      if ((robots != null) && (robots.length > 0))
      {
         GroundContactModel groundContactModel = robots[0].getGroundContactModel();
         if (groundContactModel != null)
         {
            heightMap = HeightMapFromGroundContactModel.getHeightMap(groundContactModel);
         }
      }

      myGUI.setup(heightMap);

      // myGUI.updateCamera();
      if (robots != null)
      {
         myGUI.updateRobotsAndCamera();
         myGUI.updateSimulationGraphics();
      }
   }

   /**
    * Configure the clip distances for the currently active view.  These distances specify the draw region for objects.  If an object is closer than the near distance or furth than the far distance it will not be drawn.
    * The default values for these are 0.25 and 30 respectively.
    *
    * @param near Nearest point at which objects are drawn.
    * @param far  Furthest point at which objects are drawn.
    */
   public void setClipDistances(double near, double far)
   {
      if (myGUI != null)
      {
         myGUI.setClipDistances(near, far);
      }
   }

   /**
    * Specifies the horizontal field of view in radians for the currently active view.  Objects outside of the field of view are not drawn.
    *
    * @param fieldOfView Field of view in radians.
    */
   public void setFieldOfView(double fieldOfView)
   {
      if (myGUI != null)
      {
         myGUI.setFieldOfView(fieldOfView);
      }
   }

   /**
    * If the GUI exists, stop whatever it is doing and disable user actions.
    */
   @Override
   public void disableGUIComponents()
   {
      if (myGUI != null)
      {
         myGUI.disableGUIComponents();
      }

      // isRunning = false;
      isPlaying = false;
   }

   /**
    * If the GUI exists, enable user actions.
    */
   @Override
   public void enableGUIComponents()
   {
      if (myGUI != null)
      {
         myGUI.enableGUIComponents();
      }
   }

   /**
    * Set the specified background color
    *
    * @param color Color
    * @see Color
    */
   public void setBackgroundColor(Color color)
   {
      setBackgroundColor(new Color3f(color));
   }

   /**
    * Set the specified background color
    *
    * @param color Color3f
    * @see Color3f
    */
   public void setBackgroundColor(Color3f color)
   {
      if (myGUI != null)
      {
         myGUI.setBackgroundColor(color);
      }
   }

   /**
    * Set the specified background image
    *
    * @param fileURL        URL
    * @param backgroundScaleMode int
    */
   public void setBackgroundImage(URL fileURL, Graphics3DBackgroundScaleMode backgroundScaleMode)
   {
      myGUI.setBackgroundImage(fileURL, backgroundScaleMode);
   }

   /**
    * Set the specified background image
    *
    * @param fileURL URL
    * @see URL
    */
   public void setBackgroundImage(URL fileURL)
   {
      myGUI.setBackgroundImage(fileURL, Graphics3DBackgroundScaleMode.SCALE_REPEAT);
   }

   /**
    * Apply the specified appearance to the ground.  Custom appearances may be used here although several are provided in the YoAppearance class.
    *
    * @param app Appearance
    * @see YoAppearance YoAppearance
    */
   public void setGroundAppearance(AppearanceDefinition app)
   {
      if (myGUI != null)
      {
         myGUI.setGroundAppearance(app);
      }
   }

   /**
    * Sets the ground visibility.
    *
    * @param isVisible Specifies the visibility of the ground.
    */
   public void setGroundVisible(boolean isVisible)
   {
      if (myGUI != null)
      {
         myGUI.setGroundVisible(isVisible);
      }
   }

   private int fastTicks = 0;
   private boolean stopSimulationThread = false;

   /**
    * Function to run both the simulation and playback of the data. The robot and GUI are updated before simulation cycles begin.
    */
   @Override
   public void run()
   {
      // myGUI.setupConfiguration("all", "all", "all", "all");

      if (!TESTING_LOAD_STUFF)
      {
         if ((!defaultLoaded) && (myGUI != null))
         {
            if (parameters.getShowWindows())
            {
               myGUI.loadDefaultGUIConfigurationFile();

               // myGUI.loadRegistryConfiguration();
               defaultLoaded = true;
               myGUI.saveNormalGUIConfigurationFile();
            }
         }
      }

      if (robots != null)
      {
         for (Robot robot : robots)
         {
            robot.update();

            // Do the dynamics one tick to initialize things, but don't integrate them.
            // Also needed to initialize the IMUMounts.
            try
            {
               robot.doDynamicsButDoNotIntegrate();
            }
            catch (UnreasonableAccelerationException e)
            {
            }
            robot.updateIMUMountAccelerations();
         }

         // Do this to force loading of classes so no pauses on integration...
         if (mySimulation != null)
         {
            mySimulation.forceClassLoading();
         }

         if (myGUI != null)
         {
            myGUI.updateRobotsAndCamera();
            myGUI.updateSimulationGraphics();
         }
      }

      if (myGUI != null)
      {
         StandardSimulationGUI.disposeSplashWindow();
      }

      if (myGUI != null)
      {
         if (parameters.getShowWindows()) myGUI.show();

         if (!parameters.getShowYoGraphicObjects() && dynamicGraphicMenuManager != null)
         {
            dynamicGraphicMenuManager.hideAllGraphics();
         }

      }

      if (robots == null)
      {
         return;
      }

      fastTicks = 0;

      // t = rob.getVariable("t");

      simulationThreadIsUpAndRunning = true;

      // Three state loop, simulation is either playing, running, or waiting
      while (true)
      {
         if (isSimulating)
         {
            try
            {
               if (simulateNoFasterThanRealTime)
               {
                  realTimeRateEnforcer.sleepIfNecessaryToEnforceRealTimeRate(this.getTime());
               }

               simulateCycle();
            }
            catch (UnreasonableAccelerationException ex)
            {
               System.err.println("\nSimulation Stopped due to unreasonable acceleration.");
               System.err.println("   Simulation either went unstable or is too stiff.");
               System.err.println("   Try reducing gains, ground stiffness and damping, or DT");

               ArrayList<Joint> unreasonableAccelerationJoints = ex.getUnreasonableAccelerationJoints();
               System.err.println("   Joints with an unreasonable acceleration:");

               for (Joint joint : unreasonableAccelerationJoints)
               {
                  System.err.println("     " + joint.getName());
               }

//               ex.printStackTrace();

               stop();

               // Notify all the listeners that the simulation stopped...
               mySimulation.notifySimulateDoneListenersOfException(ex);
            }
            catch (Exception ex)
            {
               System.err.println("\nException while running simulation! Stack Trace:");
               ex.printStackTrace();
               stop();

               // Notify all the listeners that the simulation stopped...
               mySimulation.notifySimulateDoneListenersOfException(ex);
            }
         }
         else if (isPlaying)
         {
            playCycle();
         }
         else
         {
            loopCycle();

            try
            {
               Thread.sleep(50);
            }
            catch (InterruptedException e)
            {
            }
         }

         if (stopSimulationThread)
         {
            break;
         }

         // foo++;
         // System.out.println("Tick" + foo);
      }

      simulationThreadIsUpAndRunning = false;

   }

   /**
    * Causes the simulation thread to break if it exists.
    */
   public void stopSimulationThread()
   {
      stopSimulationThread = true;

      while (isSimulationThreadUpAndRunning())
      {
         try
         {
            Thread.sleep(100);
         }
         catch (InterruptedException e)
         {
         }
      }
   }

   /**
    * This field indicates the number of "ticks" or cycles simulated.
    */
   private int ticksSimulated = 0; // 1;

   private boolean synchronizeGraphicsAndCamerasWhileSimulating = false;

   /**
    * Temporary method for telling a sim to synchronize its graphics and cameras while simulating.
    * Only set to true if you are creating a game like simulation where the user is driving a vehicle from
    * a camera mount while simulating. This method should go away once we internally make camera updates
    * synched with GraphicsRobot, or part of GraphicsRobot...
    *
    * @param synchronizeGraphicsAndCamerasWhileSimulating
    */
   public void setSynchronizeGraphicsAndCamerasWhileSimulating(boolean synchronizeGraphicsAndCamerasWhileSimulating)
   {
      this.synchronizeGraphicsAndCamerasWhileSimulating = synchronizeGraphicsAndCamerasWhileSimulating;
   }

   /**
    * Internal function which controls simulation.  This function is synchronized.
    *
    * @throws UnreasonableAccelerationException
    *
    */
   private void simulateCycle() throws UnreasonableAccelerationException
   {
      synchronized (simulationSynchronizer) // Don't allow changes to stuff when simulating
      {
         long recordFreq = mySimulation.getRecordFreq();

         // Check for overflow.
         if (ticksToSimulate < 0)
         {
            ticksToSimulate = 0;
         }

         long ticksThisCycle = Math.min(ticksToSimulate, recordFreq - ticksSimulated);

         // for(int i=0;i<RECORD_FREQ;i++)
         for (int i = 0; i < ticksThisCycle; i++)
         {
            if ((myGUI != null) && (synchronizeGraphicsAndCamerasWhileSimulating))
            {
               synchronized (myGUI.getGraphicsConch())
               {
                  mySimulation.simulate();
                  myGUI.updateSimulationGraphics();
               }
            }

            else
            {
               mySimulation.simulate();
            }
         }

         // Update the tick counts.
         ticksToSimulate -= ticksThisCycle;
         ticksSimulated += ticksThisCycle;

         if (mySimulation.checkSimulateDoneCriterion())
         {
            ticksToSimulate = 0;
         }

         if (ticksToSimulate <= 0)
         {
            this.stop();

            // Notify all the listeners that the simulation stopped...
            mySimulation.notifySimulateDoneListeners();
         }

         if (ticksSimulated < recordFreq)
         {
            return; // Only update Data Buffer, graphs, and sleep, every RECORD_CYCLES
         }

         ticksSimulated -= recordFreq; // This prevents the following stuff from happening continuosly after one record cycle

         myDataBuffer.tickAndUpdate(); // Update the data buffer and the min max values of each point it contains

         if (myGUI != null)
         {
            if ((!fastSimulate))
            {
               myGUI.updateGraphs(); // If the GUI exists and fast simulate is disabled update graphs
            }
            else
            {
               // Prevents the graphs from being updated every record cycle to speed up simulation
               fastTicks++;

               if (fastTicks > numberOfTicksBeforeUpdatingGraphs)
               {
                  fastTicks = 0;

                  myGUI.updateGraphs();
               }
            }

            synchronized (simulationSynchronizer) // Synched so we don't update during a graphics redraw...
            {
               myGUI.updateSimulationGraphics();
            }
         }
      }
   }

   private boolean updateGraphsDuringPlayback = true;

   /**
    * Enables or disables graph updates during playback. Disabling updates may grant some improved performance but the current data point will nolonger be highlighted on the graphs.
    * This is true by default.
    *
    * @param updateGraphs Specify whether or not graphs should update during playback.
    */
   public void setGraphsUpdatedDuringPlayback(boolean updateGraphsDuringPlayback)
   {
      this.updateGraphsDuringPlayback = updateGraphsDuringPlayback;
   }

   /**
    * Check to see if graph updates are enabled during playback.
    *
    * @return Are graph updates enabled during playback?
    * @see #setGraphsUpdatedDuringPlayback setGraphsUpdatedDuringPlayback
    */
   public boolean areGraphsUpdatedDuringPlayback()
   {
      return updateGraphsDuringPlayback;
   }

   /**
    * If true, will slow down simulations that are faster than real time to simulate at exactly real time rate.
    *
    * @param simulateNoFasterThanRealTime
    */
   public void setSimulateNoFasterThanRealTime(boolean simulateNoFasterThanRealTime)
   {
      this.simulateNoFasterThanRealTime = simulateNoFasterThanRealTime;
      this.realTimeRateEnforcer.reset();
   }

   /**
    * Return whether simulation will slow down when faster than real time to simulate at exactly real time rate.
    *
    * @param simulateNoFasterThanRealTime
    */
   public boolean getSimulateNoFasterThanRealTime()
   {
      return this.simulateNoFasterThanRealTime;
   }

   private long nextWakeMillis;
   // private long graphDelayTicks = 0, graphTicks = 0;

   /**
    * Internal function which handles simulation playback.  This updates the Robot, the graphics, and steps through the data.
    */
   private void playCycle()
   {
      if (myGUI == null)
      {
         return; // Only works with a GUI
      }

      lastIndexPlayed = myDataBuffer.getIndex();

      //    count++;

      //
      long currentTime;
      while ((currentTime = System.currentTimeMillis()) < nextWakeMillis)
      {
         try
         {
            Thread.sleep(5);
         }
         catch (InterruptedException e)
         {
         }

         // Thread.yield();
      }

      long numTicks = (currentTime - nextWakeMillis) / ((PLAY_CYCLE_TIME_MS)) + 1;

      // System.out.println("tick number  = "+(Math.max((int) (TICKS_PER_PLAY_CYCLE * numTicks), 1)));
      nextWakeMillis = nextWakeMillis + ((PLAY_CYCLE_TIME_MS)) * numTicks;

      // myDataBuffer.tick(Math.max((int) (TICKS_PER_PLAY_CYCLE * numTicks), 1));

      // myGUI.allowTickUpdatesNow();

      synchronized (simulationSynchronizer) // Synched so we don't update during a graphics redraw...
      {
         int tick = Math.max((int) (TICKS_PER_PLAY_CYCLE * numTicks), 1);
         myDataBuffer.tickButDoNotNotifySimulationRewoundListeners(tick);
         myGUI.updateRobots();
         myGUI.allowTickUpdatesNow();
         if (playCycleListeners != null)
         {
            for (int i = 0; i < playCycleListeners.size(); i++)
            {
               playCycleListeners.get(i).update(tick);
            }
         }
         myGUI.updateSimulationGraphics();
      }

      // if (updateGraphs) myGUI.updateGraphsLeisurely(20);
      if (updateGraphsDuringPlayback)
      {
         myGUI.updateGraphs();
      }

      /*
       * if (numTicks == 1) graphDelayTicks--; else graphDelayTicks =
       * graphDelayTicks + numTicks*5;
       *
       * graphTicks++; if (graphTicks > graphDelayTicks) { graphTicks = 0;
       * myGUI.updateGraphs(); }
       */

      // if(last> myDataBuffer.getIndex())
      // {
      ////            System.out.println("times around = "+count);
      // last = myDataBuffer.getIndex();
      // }
      // System.out.println("play Cycle");
   }

   public Vector<File> saveSimulationAsSequenceOfImages(String path, String NameNoExtension, CaptureDevice captureDevice)
   {
      myGUI.updateGraphs();

      try
      {
         Thread.sleep(125);
      }
      catch (InterruptedException e)
      {
      }

      Vector<File> output = new Vector<File>();
      lastIndexPlayed = 0; // This keeps track of what the previous index was to stop the playback when it starts to loop back.

      if (myGUI == null)
      {
         return null; // Only works with a GUI
      }

      myDataBuffer.gotoInPoint();

      while (lastIndexPlayed < myDataBuffer.getOutPoint())
      {
         lastIndexPlayed = myDataBuffer.getIndex();

         File file = new File(path, NameNoExtension + "_" + lastIndexPlayed + ".jpeg");

         exportSnapshot(file, captureDevice);
         output.add(file);

         synchronized (simulationSynchronizer) // Synched so we don't update during a graphics redraw...
         {
            myDataBuffer.tick(1);
            myGUI.updateRobots();
            myGUI.updateSimulationGraphics();
            myGUI.allowTickUpdatesNow();
         }

         if (updateGraphsDuringPlayback)
         {
            myGUI.updateGraphs();
         }
      }

      return output;
   }

   /**
    * Primarily a waiting cycle, this method allows the GUI to catch up if it was behind, otherwise it waits for user action.
    */
   private void loopCycle()
   {
      synchronized (simulationSynchronizer) // Don't allow changes to stuff when simulating
      {
         if (myGUI != null)
         {
            boolean tickUpdates = myGUI.allowTickUpdatesNow();
            myGUI.updateRobots();
            myGUI.updateSimulationGraphics();

            if (tickUpdates)
            {
               myGUI.updateGraphs();
            }
         }
      }
   }

   /**
    * This function halts playback and simulation along with any playbackListeners that are enabled.  It also gives the GUI a chance to update.
    * Currently the only implementation of a playback listener is the StateMachinesJPanel class.
    */
   @Override
   public void stop()
   {
      isPlaying = false;
      isSimulating = false;
      realTimeRateEnforcer.reset();
      ticksToSimulate = 0;
      setScrollGraphsEnabled(true);

      synchronized (simulationSynchronizer)
      {
         if (myGUI != null)
         {
            myGUI.notifySimulationStopped();
            myGUI.updateGraphs();
            myGUI.updateSimulationGraphics();
            myGUI.updateGUI();
         }

         if (playbackListeners != null)
         {
            for (int i = 0; i < playbackListeners.size(); i++)
            {
               PlaybackListener listener = playbackListeners.get(i);
               listener.stop();
            }
         }

         myDataBuffer.notifySimulationRewoundListenerListeners();
      }

   }

   public boolean isSimulationThreadUpAndRunning()
   {
      return this.simulationThreadIsUpAndRunning;
   }

   /**
    * @return true if the simulation is simulating
    */
   @Override
   public boolean isSimulating()
   {
      return isSimulating;
   }

   /**
    * @return the Play Cycle Listeners
    */
   public ArrayList<PlayCycleListener> getPlayCycleListeners()
   {
      return playCycleListeners;
   }

   /**
    * @return true if the simulation is playing
    */
   public boolean isPlaying()
   {
      return isPlaying;
   }

   /**
    * @return true if fastSimulate is enabled
    */
   public boolean isFastSimulateEnabled()
   {
      return fastSimulate;
   }

   /**
    * @return number of ticks are needed before updating the graphs in fastSimulate mode
    */
   public int getNumberOfTicksBeforeUpdatingGraphs()
   {
      return numberOfTicksBeforeUpdatingGraphs;
   }

   /**
    * This method causes the GUI to enter play mode assuming it is not already in run (simulate) mode.
    */
   @Override
   public void play()
   {
      if (isSimulating || isPlaying)
      {
         return;
      }

      isSimulating = false;
      isPlaying = true;
      nextWakeMillis = System.currentTimeMillis();

      synchronized (simulationSynchronizer)
      {
         if (this.playbackListeners != null)
         {
            for (int i = 0; i < playbackListeners.size(); i++)
            {
               PlaybackListener listener = playbackListeners.get(i);
               listener.play(REAL_TIME_RATE);
            }
         }
      }
   }

   /**
    * When enabled fastSimulate causes the graphs to update less frequently improving simulation performance.
    *
    * @param fastSimulate Enables or disables fastSimulate.
    */
   public void setFastSimulate(boolean fastSimulate)
   {
      synchronized (simulationSynchronizer)
      {
         this.fastSimulate = fastSimulate;
      }
   }

   /**
    * When enabled fastSimulate causes the graphs to update less frequently improving simulation performance.
    * Specify how many ticks are needed before updating the graphs.
    *
    * @param fastSimulate Enables or disables fastSimulate.
    */
   public void setFastSimulate(boolean fastSimulate, int numberOfTicksBeforeUpdatingGraphs)
   {
      synchronized (simulationSynchronizer)
      {
         this.fastSimulate = fastSimulate;
         this.numberOfTicksBeforeUpdatingGraphs = numberOfTicksBeforeUpdatingGraphs;
      }
   }

   /**
    * Triggers a single simulation step.  This runs one simulation cycle and if appropriate records the data and updates graphs.
    *
    * @throws UnreasonableAccelerationException
    *          This exception indicates an unreasonable acceleration occured.  Try reducing gains, ground stiffness and damping, or DT.
    */
   public void simulateOneTimeStep() throws UnreasonableAccelerationException
   {
      synchronized (simulationSynchronizer)
      {
         // mySimulator.simulate();
         ticksToSimulate = 1;
         this.simulateCycle();
      }
   }

   /**
    * This triggers the simulation of a single record step which is usually several simulation steps.  This step will result in the storage of a data point.
    *
    * @throws UnreasonableAccelerationException
    *          This exception indicates an unreasonable acceleration occured.  Try reducing gains, ground stiffness and damping, or DT.
    * @see #setRecordDT setRecordDT
    */
   public void simulateOneRecordStep() throws UnreasonableAccelerationException
   {
      synchronized (simulationSynchronizer)
      {
         // mySimulator.simulate();
         ticksToSimulate = (int) mySimulation.getRecordFreq();
         this.simulateCycle();
      }
   }

   /**
    * Immediately simulates a record step, records the data point, and updates the graphs.
    *
    * @throws UnreasonableAccelerationException
    *          This exception indicates an unreasonable acceleration occured.  Try reducing gains, ground stiffness and damping, or DT.
    */
   public void simulateOneRecordStepNow() throws UnreasonableAccelerationException
   {
      synchronized (simulationSynchronizer)
      {
         long recordFreq = mySimulation.getRecordFreq();

         // for(int i=0;i<RECORD_FREQ;i++)
         for (int i = 0; i < recordFreq; i++)
         {
            mySimulation.simulate();
         }

         mySimulation.notifySimulateDoneListeners();

         myDataBuffer.tickAndUpdate();

         if (myGUI != null)
         {
            myGUI.updateGraphs();
            myGUI.updateSimulationGraphics();
         }
      }
   }

   /**
    * Simulates the specified number of steps.  If the number is less than a record cycle a data point will not be created.
    *
    * @param numTicks Number of ticks or steps to simulate.
    */
   public void simulate(int numTicks)
   {
      // waitUntilReadyToSimulate();

      if (isSimulating)
      {
         synchronized (simulationSynchronizer)
         {
            ticksToSimulate += numTicks;
         }
      }
      else
      {
         ticksToSimulate += numTicks;
         isPlaying = false;
         isSimulating = true;
         realTimeRateEnforcer.reset();
      }

   }

   /**
    * Simulates for the specified time duration in seconds.  The time specified must be at least one integration step (tick) in length.
    *
    * @param simulationTime Simulation time in seconds.
    */
   public void simulate(double simulationTime)
   {
      simulate((int) Math.round(simulationTime / mySimulation.getDT()));
   }

   /**
    * Begin simulation mode, unless playback mode is currently enabled.
    */
   @Override
   public void simulate()
   {
      setScrollGraphsEnabled(false);

      if (isSimulating)
      {
         return;
      }

      simulate(simulateDurationInSeconds);
   }

   public void setScrollGraphsEnabled(boolean enable)
   {
      myDataBuffer.setSafeToChangeIndex(enable);
   }

   public boolean isSafeToScroll()
   {
      return myDataBuffer.isSafeToChangeIndex();
   }

   public void setSimulateDuration(double simulateDurationInSeconds)
   {
      this.simulateDurationInSeconds = simulateDurationInSeconds;
   }

   public double getSimulateDuration()
   {
      return simulateDurationInSeconds;
   }

   /**
    * Calls the doControl() method of the robots contained in the underlying Simulator. Step 2/3 of Simulator.simulate().
    * Should only be used for testing purposes.
    */
   public void doControl()
   {
      mySimulation.doControl();
   }

   /**
    * Does the dynamics and integrates the state equations for all the robots contained in the underlying Simulator.
    * Step 3/3 of Simulator.simulate().
    * Should only be used for testing purposes.
    */
   public void doDynamicsAndIntegrate() throws UnreasonableAccelerationException
   {
      mySimulation.doDynamicsAndIntegrate();
   }

   /**
    * Add a simulate done listener to the simulation.  All SimulateDoneListeners will be triggered when the simulation completes.
    *
    * @param listener SimulationDoneListener
    */
   public void addSimulateDoneListener(SimulationDoneListener listener)
   {
      synchronized (simulationSynchronizer)
      {
         mySimulation.addSimulateDoneListener(listener);
      }
   }

   /**
    * Sets the criterion for simulation completion.  When the criterion is met the simulation will complete and SimulateDoneListeners will be triggered.
    *
    * @param criterion Criterion for simulation completion.
    * @see SimulationDoneCriterion SimulationDoneCriterion
    */
   public void setSimulateDoneCriterion(SimulationDoneCriterion criterion)
   {
      synchronized (simulationSynchronizer)
      {
         mySimulation.setSimulateDoneCriterion(criterion);
      }
   }

   /**
    * Removes the specified SimulateDoneListener.  Once removed the listener will nolonger be triggered when the simulation completes.
    *
    * @param listener Listener to be removed.
    */
   public void removeSimulateDoneListener(SimulationDoneListener listener)
   {
      synchronized (simulationSynchronizer)
      {
         mySimulation.removeSimulateDoneListener(listener);
      }
   }

   /**
    * Move the current data point to inPoint.  This has no effect while the simulation is running but will effect playback.
    */
   @Override
   public void gotoInPoint()
   {
      standardAllCommandsExecutor.gotoInPoint();
   }

   /**
    * Causes execution to continue from the inPoint.  This will happen in all modes and can cause exceptions in simulation mode if the transistion is too severe.
    */
   public void gotoInPointNow()
   {
      if (myGUI != null)
      {
         myGUI.gotoInPointNow();
      }
      else
      {
         myDataBuffer.gotoInPoint();
      }
   }

   /**
    * Moves the current data point to the outPoint.  This has no effect while simulating but will effect playback.
    */
   @Override
   public void gotoOutPoint()
   {
      standardAllCommandsExecutor.gotoOutPoint();
   }

   public void gotoOutPointNow()
   {
      if (myGUI != null)
      {
         myGUI.gotoOutPointNow();
      }
      else
      {
         myDataBuffer.gotoOutPoint();
      }
   }

   /**
    * Makes the current data point the in point. This works in all modes.
    */
   @Override
   public void setInPoint()
   {
      standardAllCommandsExecutor.setInPoint();
   }

   public void setInOutPointFullBuffer()
   {
      standardAllCommandsExecutor.setInOutPointFullBuffer();
   }

   /**
    * Makes the current data point a KeyPoint.  If the point is already a KeyPoint it ceases to be one.  KeyPoints are essentially bookmarks.  When they are enabled single steps will move between KeyPoints instead of traveling through the data in a continuous fashion.
    */
   @Override
   public void addKeyPoint()
   {
      standardAllCommandsExecutor.addKeyPoint();
   }

   /**
    * Gets the KeyPoints in the cropped data
    *
    * @return The current KeyPoints as an ArrayList of Integer
    */
   public ArrayList<Integer> getKeyPoints()
   {
      return standardAllCommandsExecutor.getKeyPoints();
   }

   /**
    * Makes the current data point a CameraKeyPoint.  If the point is already a CameraKeyPoint then it is replaced with the new CameraKeyPoint.  CameraKeyPoints are recorded positions for the camera to move to at a given time.
    */
   @Override
   public void addCameraKey()
   {
      standardAllCommandsExecutor.addCameraKey();
   }

   /**
    * Retrieves the camera KeyPoints used in this simulation.
    *
    * @return Camera KeyPoints contained in this simulation as ArrayList
    */
   public ArrayList<Integer> getCameraKeyPoints()
   {
      return standardAllCommandsExecutor.getCameraKeyPoints();
   }

   /**
    * Removes the CameraKeyPoint at the current data point.
    */
   @Override
   public void removeCameraKey()
   {
      standardAllCommandsExecutor.removeCameraKey();
   }

   /**
    * Removes the CameraKeyPoint at the current data point.
    */
   @Override
   public void nextCameraKey()
   {
      standardAllCommandsExecutor.nextCameraKey();
   }

   /**
    * Removes the CameraKeyPoint at the current data point.
    */
   @Override
   public void previousCameraKey()
   {
      standardAllCommandsExecutor.previousCameraKey();
   }

   /**
    * Makes the current data point the out point.  This has no effect in simulation mode as the current point is always the out point.
    */
   @Override
   public void setOutPoint()
   {
      standardAllCommandsExecutor.setOutPoint();
   }

   /**
    * Step backward one tick.  If KeyPoints are enabled step back to the first KeyPoint smaller than the current point.  This method has no effect while the simulation is running.
    */
   @Override
   public void stepBackward()
   {
      standardAllCommandsExecutor.stepBackward();
   }

   /**
    * Step backward the specified number of ticks.  If KeyPoints are enabled step back to the first KeyPoint smaller than the current point.  This method has no effect while the simulation is running.
    *
    * @param steps The number of steps, or ticks, to move backward through the data.
    */
   public void stepBackward(int steps)
   {
      if (myGUI != null)
      {
         myGUI.stepBackward(steps);
      }
      else
      {
         myDataBuffer.tick(-steps);
      }
   }

   /**
    * Immediately step backward one tick.  This method effects simulation mode, However a single backward tick will probably not cause an issue.
    */
   public void stepBackwardNow()
   {
      if (myGUI != null)
      {
         myGUI.stepForwardNow(-1);
      }
      else
      {
         myDataBuffer.tick(-1);
      }
   }

   /**
    * Step forward one tick.  If KeyPoints are enabled, step forward to the first KeyPoint larger than the current point.  This method has no effect while the simulation is running.
    */
   @Override
   public void stepForward()
   {
      standardAllCommandsExecutor.stepForward();
   }

   /**
    * Step forward the specified number of ticks.  If KeyPoints are enabled, step forward to the first KeyPoint larger than the current point.  This method has no effect while the simulation is running.
    *
    * @param steps int
    */
   public void stepForward(int steps)
   {
      if (myGUI != null)
      {
         myGUI.stepForward(steps);
      }
      else
      {
         myDataBuffer.tick(steps);
      }
   }

   /**
    * Immediately step forward the specified number of ticks.  This method effects simulation mode and as such can cause unpredictable behavior if stepped during simulation.
    *
    * @param steps int
    */
   public void stepForwardNow(int steps)
   {
      if (myGUI != null)
      {
         myGUI.stepForwardNow(steps);
      }
      else
      {
         myDataBuffer.tick(steps);
      }
   }

   // public void zoomIn(){if (myGUI != null) myGUI.zoomIn();}
   // public void zoomOut(){if (myGUI != null) myGUI.zoomOut();}

   /**
    * Crops the data buffer to the current in and out points.  All data outside of this range is discarded and the graphs are redrawn to fit the existing set.
    * Once the data is cropped the current index is moved to the new start point.  This method operates during all modes, if triggered during simulation the transition between the current point and the new start point may result in anomalous behavior.
    */
   @Override
   public void cropBuffer()
   {
      standardAllCommandsExecutor.cropBuffer();
   }

   @Override
   public void cutBuffer()
   {
      standardAllCommandsExecutor.cutBuffer();
   }

   public void thinBuffer(int keepEveryNthPoint)
   {
      standardAllCommandsExecutor.thinBuffer(keepEveryNthPoint);
   }

   /**
    * Packs the data buffer based on the current inPoint.  Essentially this shifts the data such that the inPoint is at index zero.  This method can operate during simulation which under some conditions may cause abnormal behavior.
    */
   @Override
   public void packBuffer()
   {
      standardAllCommandsExecutor.packBuffer();
   }

   /**
    * Specify whether or not the buffer should wrap once the end is reached.  By default the buffer expands until it reaches the predefined max size.  If enabled the buffer will not expand.
    *
    * @param wrap Should the buffer wrap to the beginning instead of expanding?
    */
   public void setWrapBuffer(boolean wrap)
   {
      myDataBuffer.setWrapBuffer(wrap);
   }

   /**
    * Either increase or decrease the data buffer's size in units of ticks.  The new buffer will begin with the current inPoint and end at one of two points.  If the buffer size is increased all of the original data persists otherwise the data is cropped between the inPoint and the new buffer size.
    * The data is packed if the buffer increases in size.  In either case the inPoint is shifted to the beginning and the graphs are zoomed to full view.
    *
    * @param bufferSize New buffer size in ticks, the buffer size must be a positive integer.
    */
   public void changeBufferSize(int bufferSize)
   {
      myDataBuffer.changeBufferSize(bufferSize);

      if (myGUI != null)
      {
         myGUI.zoomFullView();
      }
   }

   /**
    * Sets the maximum size, in ticks, to which the buffer will expand.  While nonsense values are not explicitly checked for, they will not cause the buffer to shrink or behave abnormally.
    *
    * @param maxBufferSize New max buffer size.
    */

   public void setMaxBufferSize(int maxBufferSize)
   {
      myDataBuffer.setMaxBufferSize(maxBufferSize);
   }

   /**
    * Specifies the directory to which data will be exported. By default, this directory is the one in which this simulation's robot is defined.
    *
    * @param directory Path name of the desired directory.
    */
   public void setExportDataDirectory(String directory)
   {
      myGUI.setExportDataDirectory(directory);
   }

   /**
    * Specifies the directory from which data will be imported.  By default this directory is the location of the robot class used in this simulation.
    *
    * @param directory String
    */
   public void setImportDataDirectory(String directory)
   {
      myGUI.setImportDataDirectory(directory);
   }

   /**
    * Exports a snapshot from the currently active view.  This image is saved to the specified file as a jpeg.
    *
    * @param snapshotFile File to which the image is saved.
    */
   @Override
   public void exportSnapshot(File snapshotFile)
   {
      CaptureDevice capturableCanvas = myGUI.getActiveCaptureDevice();
      capturableCanvas.exportSnapshot(snapshotFile);
   }

   /**
    * Exports a snapshot from the passed viewportSelector.  This image is saved to the specified file as a jpeg.
    *
    * @param snapshotFile     File to which the image is saved.
    * @param viewportSelector the viewport to take the snapshot from
    */
   public void exportSnapshot(File snapshotFile, ViewportWindow viewportSelector)
   {
      CaptureDevice capturableCanvas = viewportSelector.getActiveCaptureDevice();
      capturableCanvas.exportSnapshot(snapshotFile);
   }

   public void exportSnapshot(File snapshotFile, CaptureDevice captureDevice)
   {
      captureDevice.exportSnapshot(snapshotFile);
   }

   public BufferedImage exportSnapshotAsBufferedImage()
   {
      CaptureDevice capturableCanvas = myGUI.getActiveCaptureDevice();

      return capturableCanvas.exportSnapshotAsBufferedImage();
   }

   public BufferedImage exportSnapshotAsBufferedImage(CaptureDevice captureDevice)
   {
      return captureDevice.exportSnapshotAsBufferedImage();
   }

   /**
    * Writes the data recorded by the simulation to the specified file.  This data is stored in a compressed binary format.  To import the file with SCS it must have the extension data.gz
    *
    * @param chosenFile File to which the data is saved.
    */
   public void writeData(File chosenFile)
   {
      boolean binary = true;
      boolean compress = true;

      writeData("all", binary, compress, chosenFile);
   }

   /**
    * Writes the data recorded by the simulation to a file with the provided path.  This data is stored in a compressed binary format.  To import the file with SCS it must have the extension data.gz
    *
    * @param filename String
    */
   public void writeData(String filename)
   {
      File file = new File(filename);
      writeData(file);
   }

   /**
    * Stores the data for the specified varGroup.  This data can be stored as either binary or text and may be compressed or uncompressed.  To import the file with SCS it must have the proper extension based on its format.
    * There are two possibilities:<br />
    * Compressed: data.gz<br />
    * Uncompressed: data<br />
    * As an example stuffVars.data.gz would be an acceptable name for a compressed file.<br />
    * VarGroup "all" contains all simulation variables.
    *
    * @param varGroup Name of the desired varGroup.
    * @param binary   Specify the file format, binary or ASCII.
    * @param compress Specify the presence of compression.
    * @param filename Path to the desired file, depends on compress.
    */
   public void writeData(String varGroup, boolean binary, boolean compress, String filename)
   {
      File file = new File(filename);
      writeData(varGroup, binary, compress, file);
   }

   /**
    * Stores simulation data from the specified varGroup in the given File.  This data is stored in ASCII CSV (comma separated value) format.  To be imported by SCS the file must have the extension .data.csv
    * As this data is uncompressed text it will be far larger than some of the other data storage options.<br />
    * VarGroup "all" contains all simulation variables.
    *
    * @param varGroupName Name of the varGroup to be stored
    * @param chosenFile   File in which to store the data.
    */
   public void writeSpreadsheetFormattedData(String varGroupName, File chosenFile)
   {
      DataFileWriter dataWriter = new DataFileWriter(chosenFile);
      PrintTools.info(this, "Writing Data File " + chosenFile.getAbsolutePath());

      // ArrayList vars = myGUI.getVarsFromGroup(varGroup);
      ArrayList<YoVariable<?>> vars = myDataBuffer.getVarsFromGroup(varGroupName, varGroupList);

      // dataWriter.writeSpreadsheetFormattedData(myDataBuffer, (mySimulation.getDT() * mySimulation.getRecordFreq()), vars);
      dataWriter.writeSpreadsheetFormattedData(myDataBuffer, vars);
   }

   /**
    * Stores the data for the specified varGroup.  This data can be stored as either binary or text and may be compressed or uncompressed.  To import the file with SCS it must have the proper extension based on its format.
    * There are two possibilities:<br />
    * Compressed: data.gz<br />
    * Uncompressed: data<br />
    * As an example stuffVars.data.gz would be an acceptable name for a compressed file.<br />
    * VarGroup "all" contains all simulation variables.
    *
    * @param varGroupName   Name of the desired varGroup.
    * @param binary     Specify the file format, binary or ASCII.
    * @param compress   Specify the presence of compression.
    * @param chosenFile File to which data will be saved
    */
   public void writeData(String varGroupName, boolean binary, boolean compress, File chosenFile)
   {
      ArrayList<YoVariable<?>> vars = myDataBuffer.getVarsFromGroup(varGroupName, varGroupList);
      writeData(vars, binary, compress, chosenFile);
   }

   /**
    * Stores the data for the specified list of YoVariables.  This data can be stored as either binary or text and may be compressed or uncompressed.  To import the file with SCS it must have the proper extension based on its format.
    * There are two possibilities:<br />
    * Compressed: data.gz<br />
    * Uncompressed: data<br />
    * As an example stuffVars.data.gz would be an acceptable name for a compressed file.<br />
    *
    * @param vars       Variables to write to file
    * @param binary     Specify the file format, binary or ASCII.
    * @param compress   Specify the presence of compression.
    * @param chosenFile File to which data will be saved
    */
   @Override
   public void writeData(ArrayList<YoVariable<?>> vars, boolean binary, boolean compress, File chosenFile)
   {
      DataFileWriter dataWriter = new DataFileWriter(chosenFile);
      PrintTools.info(this, "Writing Data File " + chosenFile.getAbsolutePath());

      dataWriter.writeData(robots[0].getName(), mySimulation.getDT() * mySimulation.getRecordFreq(), myDataBuffer, vars, binary, compress, robots[0]);
   }


   public void writeMatlabData(String varGroup, File chosenFile)
   {
      DataFileWriter dataWriter = new DataFileWriter(chosenFile);
      PrintTools.info(this, "Writing Data File " + chosenFile.getAbsolutePath());

      ArrayList<YoVariable<?>> vars = myDataBuffer.getVarsFromGroup(varGroup, varGroupList);
      dataWriter.writeMatlabBinaryData( mySimulation.getDT() * mySimulation.getRecordFreq(), myDataBuffer, vars);
   }

   public File createVideo(String videoFilename)
   {
      File videoFile = new File(videoFilename);
      createVideo(videoFile);

      return videoFile;

   }

   public void createVideo(File videoFile)
   {
      if (myGUI != null) myGUI.getViewportPanel().getStandardGUIActions().createVideo(videoFile);
   }

   /**
    * Accessor method for the data buffer used by this simulation.  This buffer holds all of the record data for each YoVariable used in this simulation.
    *
    * @return The internal dataBuffer.
    */
   public DataBuffer getDataBuffer()
   {
      return myDataBuffer;
   }

   /**
    * Method to get the StandardGUIActions.
    *
    * @return StandardGUIActions.
    */
   public StandardGUIActions getStandardGUIActions()
   {
      return myGUI.getStandardGUIActions();
   }

   /**
    * Sets the name of the variable that the GUI uses for time.
    * Needed for things like doing FFTs and Bode Diagrams in the GUI.
    * Defaults to "t"
    *
    * @param timeVariableName Variable name.
    */
   public void setTimeVariableName(String timeVariableName)
   {
      myDataBuffer.setTimeVariableName(timeVariableName);
   }

   /**
    * Returns the variable that the GUI uses for time.
    * Needed for things like doing FFTs and Bode Diagrams in the GUI.
    */
   public String getTimeVariableName()
   {
      return myDataBuffer.getTimeVariableName();
   }

   /**
    * Stores the current state of all variables to the chosen file.  This file is uncompressed ASCII, the expected extension is .state
    *
    * @param chosenFile File in which to store the current state.
    */
   public void writeState(File chosenFile)
   {
      writeState("all", false, false, chosenFile);
   }

   /**
    * Stores the current state of all variables to the file given by the specified path.  This file is uncompressed ASCII, the expected extension is .state
    *
    * @param filename Path to which the file should be stored.
    */
   public void writeState(String filename)
   {
      File file = new File(filename);
      writeState(file);
   }

   /**
    * Stores the current state of all variables contained in the specified varGroup to the file given by the specified path.  These variable states can be stored as ASCII text or binary data and may be compressed.
    * Depending on the presence of compression the proper file extension is either .state or .state.gz<br />
    * VarGroup "all" contains all simulation variables.
    *
    * @param varGroup Name of the variable group to store.
    * @param binary   Specify binary data format as opposed to ASCII text.
    * @param compress Indicates whether or not the data is to be compressed.
    * @param filename The path to the desired file.
    */
   public void writeState(String varGroup, boolean binary, boolean compress, String filename)
   {
      File file = new File(filename);
      writeState(varGroup, binary, compress, file);
   }

   /**
    * Stores the current state of all variables contained in the specified varGroup to the file given by the specified path.  These variable states can be stored as ASCII text or binary data and may be compressed.
    * Depending on the presence of compression the proper file extension is either .state or .state.gz<br />
    * VarGroup "all" contains all simulation variables.
    *
    * @param varGroupName   Name of the variable group to store.
    * @param binary     Specify binary data format as opposed to ASCII text.
    * @param compress   Indicates whether or not the data is to be compressed.
    * @param chosenFile File in which the states are to be stored.
    */
   public void writeState(String varGroupName, boolean binary, boolean compress, File chosenFile)
   {
      DataFileWriter dataWriter = new DataFileWriter(chosenFile);
      System.out.println("Writing State File " + chosenFile.getName()); // filename);

      // ArrayList vars = myGUI.getVarsFromGroup(varGroup);
      ArrayList<YoVariable<?>> vars = myDataBuffer.getVarsFromGroup(varGroupName, varGroupList);
      dataWriter.writeState(robots[0].getName(), (mySimulation.getDT() * mySimulation.getRecordFreq()), vars, binary, compress);
   }

   /**
    * Stores the current state of all variables contained in the specified varGroup to the file given by the specified path.  These variable states can be stored in .csv format.
    * The proper file extension is .csv<br />
    * VarGroup "all" contains all simulation variables.
    *
    * @param varGroupName Name of the variable group to store.
    * @param chosenFile   File in which the states are to be stored.
    */
   public void writeSpreadsheetFormattedState(String varGroupName, File chosenFile)
   {
      DataFileWriter dataWriter = new DataFileWriter(chosenFile);
      PrintTools.info(this, "Writing Data File " + chosenFile.getAbsolutePath());

      ArrayList<YoVariable<?>> vars = myDataBuffer.getVarsFromGroup(varGroupName, varGroupList);

      dataWriter.writeSpreadsheetFormattedState(myDataBuffer, vars);
   }

   /**
    * Import simulation data from the file at the specified path.  The file must be a data export from simulation instruction set.  There are three possible extensions; .data, .data.gz, and .data.csv
    * When this function is executed it will replace the current simulation data, if this occurs while the simulation is running numerical instability may ensue.
    *
    * @param filename Path name of the file to be imported
    */
   public void readData(String filename)
   {
      File file = new File(filename);
      readData(file);
   }

   /**
    * Import simulation data from the file at the specified URL.  The file must be a data export from simulation instruction set.  There are three possible extensions; .data, .data.gz, and .data.csv
    * When this function is executed it will replace the current simulation data, if this occurs while the simulation is running numerical instability may ensue.
    *
    * @param url URL at which the source file is located.
    */
   public void readData(URL url)
   {
      DataFileReader dataReader = new DataFileReader(url);
      readData(dataReader);
   }

   /**
    * Import simulation data from the specified file.  The file must be a data export from simulation instruction set.  There are three possible extensions; .data, .data.gz, and .data.csv
    * When this function is executed it will replace the current simulation data, if this occurs while the simulation is running numerical instability may ensue.
    *
    * @param chosenFile File to load.
    */
   public void readData(File chosenFile)
   {
      DataFileReader dataReader = new DataFileReader(chosenFile);
      readData(dataReader);
   }

   /**
    * Internal function for loading data from file.  Assuming the dataReader was created with a valid data file this function will load the data into the simulation.
    *
    * @param dataReader DataFileReader used.
    */
   private void readData(DataFileReader dataReader)
   {
      YoVariableList newVarList = new YoVariableList("Imported");

      try
      {
         int npoints = dataReader.readData(newVarList, rootRegistry, myDataBuffer, this); // robVarList, myDataBuffer);

         if (npoints > 0)
         {
            double recordDT = dataReader.getRecordDT();
            this.setRecordDT(recordDT);

            // Now that passing in a registry, not sure if we have to do this...
            // addVariablesToSimulationAndGUI(newVarList);

            if (myGUI != null)
            {
               // myDataBuffer.changeBufferSize(npoints);
               myGUI.zoomFullView();
               myGUI.updateGraphs();
               myGUI.updateSimulationGraphics();

               // if (myGUI != null) myGUI.updateComb();
            }
         }
         else
         {
            EventDispatchThreadHelper.invokeLater(new Runnable()
            {
               @Override
               public void run()
               {
                  JOptionPane.showMessageDialog(jFrame, "File not valid data file!");
               }
            });
         }
      }
      catch (IOException exception)
      {
         JOptionPane.showMessageDialog(jFrame, "IOException in read data: " + exception); // exception.getMessage());
         exception.printStackTrace();
         System.exit(-1);
      }
   }

   private void addVariablesToGUI(YoVariableList varList)
   {
      if (!varList.isEmpty())
      {
         if (myGUI != null)
         {
            myGUI.addVarList(varList);
         }

         // try
         // {
         // mySimulation.addVarList(varList);
         // }
         // catch (RepeatDataBufferEntryException ex)
         // {
         // ex.printStackTrace();
         // System.err.println("Exception in SimulationConstructionSet.addVarList(). VarList has one or more YoVariable repeats including " + ex + ".");
         // System.err.println("This could be due to having YoVariables with the same name, or due to trying to add a VarList that has been already added,");
         // System.err.println("or from having a YoVariable in multiple VarLists. Therefore the VarList was not added. VarList name = " + varList.getName()
         // + "\nVarList = \n" + varList + "\n");
         // }
      }
   }

   // private void addVariablesToSimulationAndGUI(YoVariableRegistry registry)
   // {
   //    addVariablesToSimulationAndGUI(registry.createVarListsIncludingChildren());
   // }
   //
   // private void addVariablesToSimulationAndGUI(ArrayList<VarList> varLists)
   // {
   //    for (int i = 0; i < varLists.size(); i++)
   //    {
   //       addVarList(varLists.get(i));
   //    }
   // }

   public YoVariableList getCombinedVarList()
   {
      return mySimulation.getCombinedVarList();
   }

   private void addVariablesToSimulationAndGUI(YoVariableList varList)
   {
      this.addVariablesToGUI(varList);

      try
      {
         mySimulation.addVarList(varList);
      }
      catch (RepeatDataBufferEntryException ex)
      {
         ex.printStackTrace();
         System.err.println("Exception in SimulationConstructionSet.addVarList(). VarList has one or more YoVariable repeats including " + ex + ".");
         System.err.println("This could be due to having YoVariables with the same name, or due to trying to add a VarList that has been already added,");
         System.err.println("or from having a YoVariable in multiple VarLists. Therefore the VarList was not added. VarList name = " + varList.getName()
               + "\nVarList = \n" + varList + "\n");
      }
   }

   /**
    * Read in the variable states specified by the file at the provided path.  If this method is called during simulation the current variable states will be replaced, this usually results in unpredictable behavior.
    *
    * @param filename Path to desired file.
    */
   public void readState(String filename)
   {
      readState(filename, true);
   }

   public void readState(String filename, boolean printErrorForMissingVariables)
   {
      File file = new File(filename);
      readState(file, printErrorForMissingVariables);
   }

   /**
    * Read in the variable states specified in the provided file.  If this method is called during simulation the current variable states will be replaced, usually resulting in unpredictable behavior.
    * This occurs as the simulation is based off both the current and previous data values.
    *
    * @param chosenFile File from which to read the new variable states.
    */
   public void readState(File chosenFile)
   {
      readState(chosenFile, true);
   }

   public void readState(File chosenFile, boolean printErrorForMissingVariables)
   {
      DataFileReader dataReader = new DataFileReader(chosenFile);

      try
      {
         dataReader.readState(mySimulation.getCombinedVarList(), printErrorForMissingVariables);
         myDataBuffer.tickAndUpdate();
         myDataBuffer.setInPoint();
         myDataBuffer.setOutPoint();

         if (myGUI != null)
         {
            myGUI.updateGraphs();
            myGUI.updateSimulationGraphics();
         }
      }
      catch (IOException exception)
      {
         JOptionPane.showMessageDialog(jFrame, "IOException in read state: " + exception.getMessage());
      }
   }

   /**
    * Maximizes the primary SCS window in both the x and y directions.  When this instruction is executed the window should become full screen.
    */
   public void maximizeMainWindow()
   {
      if (myGUI != null)
      {
         myGUI.maximizeMainWindow();
      }
   }

   @Override
   public GraphArrayWindow getGraphArrayWindow(String windowName)
   {
      return standardAllCommandsExecutor.getGraphArrayWindow(windowName);
   }

   @Override
   public ViewportWindow getViewportWindow(String windowName)
   {
      return standardAllCommandsExecutor.getViewportWindow(windowName);
   }

   @Override
   public GraphArrayWindow createNewGraphWindow()
   {
      return standardAllCommandsExecutor.createNewGraphWindow();
   }

   /**
    * Creates a new window containing the specified graph group.
    *
    * @param graphGroupName Name of the desired graph group.
    */
   @Override
   public GraphArrayWindow createNewGraphWindow(String graphGroupName)
   {
      return standardAllCommandsExecutor.createNewGraphWindow(graphGroupName);
   }

   /**
    * Creates a new window containing the specified graph group.  By default the window will be created at the minimum preferred size based on its contents, however, maximization may be specified.
    * The screen ID of the window determines the configuration used.  0 is the default screen ID.  For more information on this see the GraphicsConfiguration class in the Java API.
    *
    * @param graphGroupName Name of the desired graph group.
    * @param screenID       The desired screen ID
    * @param maximizeWindow Specify whether or not the window should be maximized.
    */
   public GraphArrayWindow createNewGraphWindow(String graphGroupName, int screenID, boolean maximizeWindow)
   {
      return createNewGraphWindow(graphGroupName, screenID, null, null, maximizeWindow);
   }

   @Override
   public GraphArrayWindow createNewGraphWindow(String graphGroupName, int screenID, Point windowLocation, Dimension windowSize, boolean maximizeWindow)
   {
      return standardAllCommandsExecutor.createNewGraphWindow(graphGroupName, screenID, windowLocation, windowSize, maximizeWindow);
   }

   @Override
   public ViewportWindow createNewViewportWindow()
   {
      return standardAllCommandsExecutor.createNewViewportWindow();
   }

   /**
    * Creates a new window containing the specified viewport configuration.
    *
    * @param viewportName Name of the desired viewport configuration.
    * @return The new ViewportWindow.
    * @see ViewportConfiguration ViewportConfiguration
    */
   @Override
   public ViewportWindow createNewViewportWindow(String viewportName)
   {
      return standardAllCommandsExecutor.createNewViewportWindow(viewportName);
   }

   /**
    * Creates a new window containing the specified viewport configuration.  By default the window will be created at the minimum preferred size based on its contents, however, maximization may be specified.
    * The screen ID of the window determines the configuration used.  0 is the default screen ID.  For more information on this see the GraphicsConfiguration class in the Java API.
    *
    * @param viewportName   Name of the desired viewport configuration.
    * @param screenID       The desired screen ID
    * @param maximizeWindow Specify whether or not the window should be maximized.
    * @return ViewportWindow The new ViewportWindow
    * @see ViewportConfiguration ViewportConfiguration
    */
   @Override
   public ViewportWindow createNewViewportWindow(String viewportName, int screenID, boolean maximizeWindow)
   {
      return standardAllCommandsExecutor.createNewViewportWindow(viewportName, screenID, maximizeWindow);
   }

   public RobotSocketConnection allowTCPConnectionToHost(String HOST)
   {
      return allowTCPConnectionToHost(HOST, (ArrayList<NewDataListener>) null);
   }

   public RobotSocketConnection allowTCPConnectionToHost(String HOST, NewDataListener newDataListener)
   {
      if (newDataListener == null)
      {
         return allowTCPConnectionToHost(HOST);
      }

      ArrayList<NewDataListener> newDataListeners = new ArrayList<NewDataListener>(1);
      newDataListeners.add(newDataListener);

      return allowTCPConnectionToHost(HOST, newDataListeners);
   }

   /**
    * Creates a SCSRobotGUICommandListener which communicates to a robot at the specified ethernet HOST address.
    * Only the variables specified in the YoVariableRegistries, set through the GUI are sent and/or logged on the robot side.
    * This method adds the relevant GUI modifications for robot communication.
    * Once communication is established the new RobotSocketConnection monitors and manages robot communication.
    *
    * @param HOST IP address of robot.
    * @return The new SCSRobotGUICommandListener.
    * @see SCSRobotGUICommunicatorComponents SCSRobotGUICommandListener
    */
   public RobotSocketConnection allowTCPConnectionToHost(String HOST, ArrayList<NewDataListener> newDataListeners)
   {
      RobotConnectionGUIUpdater guiUpdater = new RobotConnectionGUIUpdater(this);

      GUISideCommandListener robotCommandListener = new GUISideCommandListener(mySimulation.getDataBuffer(), rootRegistry, guiUpdater, guiUpdater);
      RobotSocketConnection robotSocketConnection = new RobotSocketConnection(HOST, robotCommandListener, rootRegistry, newDataListeners);

      if (myGUI != null)
      {
         NameSpaceHierarchyTree nameSpaceHierarchyTree = myGUI.getCombinedVarPanel().getNameSpaceHierarchyTree();
         nameSpaceHierarchyTree.addRegistrySettingsChangedListener(robotSocketConnection);
         robotCommandListener.addCreatedNewRegistryListener(nameSpaceHierarchyTree);
      }

      SCSRobotGUICommunicatorComponents robotGUI = new SCSRobotGUICommunicatorComponents(robotSocketConnection);

      robotCommandListener.attachDoDisconnectListener(robotSocketConnection);
      robotCommandListener.attachDoDisconnectListener(robotGUI);

      robotGUI.putButtonsAndExitActionListenerOnSimulationGUI(this);

      return robotSocketConnection;
   }

   // ++JEP:  These aren't good, but is only there so that PC104 stuff can work...

   /**
    * Adds all of the YoVariables contained in the specified VarList to the simulation.  If any of the variables are already present an exception will occur.
    *
    * @param newVarList VarList to be merged with the existing data.
    */
   public void addVarList(YoVariableList newVarList)
   {
      addVariablesToSimulationAndGUI(newVarList);
   }

   /**
    * Adds all of the YoVariables contained in the provided list of VarLists to the simulation.  If any of the variables are already present an exception will occur and that particular VarList will fail.
    * Each varlist will have its own tab in the var panel.
    *
    * @param newVarLists List of VarLists to be added.
    */
   public void addVarLists(YoVariableList[] newVarLists)
   {
      for (int i = 0; i < newVarLists.length; i++)
      {
         addVarList(newVarLists[i]);
      }
   }

   /**
    * Adds all of the YoVariables contained in the provided ArrayList of VarLists to the simulation.  If any of the variables are already present an exception will occur and that particular VarList will fail.
    * Each varlist will have its own tab in the var panel.
    *
    * @param newVarLists ArrayList
    */
   public void addVarLists(ArrayList<YoVariableList> newVarLists)
   {
      for (int i = 0; i < newVarLists.size(); i++)
      {
         addVarList(newVarLists.get(i));
      }

      /*
       * This makes sure that the initial values of all YoVariables that are
       * added to the scs (i.e. at index 0 of the data buffer) are properly
       * stored in the data buffer
       */
      getDataBuffer().copyValuesThrough();
   }

   // /**
   // * ???  This does some stuff  ???
   // *
   // * @param cameraConfiguration CameraConfiguration
   // * @param width int
   // * @param height int
   // * @param dpi double
   // * @return OffScreen3DView
   // */
   // public OffScreen3DView createOffScreenCanvas3D(CameraConfiguration cameraConfiguration, int width, int height, double dpi)
   // {
   // OffScreen3DView offScreen3DView = null;
   //
   // offScreen3DView = new OffScreen3DView(this, width, height, dpi);
   // myGUI.setupOffScreen3DView(offScreen3DView, cameraConfiguration);
   //
   // return offScreen3DView;
   // }

   /**
    * Hides the main viewport integrated into the SCS GUI.  This method has no effect on ViewportWindows.
    */
   public void hideViewport()
   {
      standardAllCommandsExecutor.hideViewport();
   }

   public Boolean isViewportHidden()
   {
      return standardAllCommandsExecutor.isViewportHidden();
   }

   /**
    * Shows the main viewport integrated into the SCS GUI.  This method has no effect on ViewportWindows.
    */
   public void showViewport()
   {
      standardAllCommandsExecutor.showViewport();
   }

   /**
    * Attaches the specified PlaybackListener to the simulation.  This listener will only operate while the simulation is running or playing.  Whenever the simulation exits either state the listener will stop.
    * For an example of a playbackListener see StateMachinesJPanel
    *
    * @param playbackListener Listener to add.
    * @see StateMachinesJPanel StateMachinesJPanel
    */
   public void attachPlaybackListener(PlaybackListener playbackListener)
   {
      if (playbackListeners == null)
      {
         this.playbackListeners = new ArrayList<PlaybackListener>();
      }

      playbackListeners.add(playbackListener);

      myDataBuffer.attachIndexChangedListener(playbackListener);
   }

   /**
    * Attaches the specified PlayCycleListener to the simulation. THis listener will only operate while the simulation is playing. No events are fired during simulation
    *
    * @param playCycleListener
    */
   public void attachPlayCycleListener(PlayCycleListener playCycleListener)
   {
      if (playCycleListeners == null)
      {
         playCycleListeners = new ArrayList<PlayCycleListener>();
      }

      playCycleListeners.add(playCycleListener);
   }

   /**
    * Applies a function to the recorded data starting from the in point to the out point.
    *
    * @param dataProcessingFunction DataProcessingFunction to be applied to the data.
    * @see DataProcessingFunction
    */
   public void applyDataProcessingFunction(DataProcessingFunction dataProcessingFunction)
   {
      myDataBuffer.applyDataProcessingFunction(dataProcessingFunction);
   }

   /**
    * Applies a function to the recorded data starting from the out point to the in point.
    *
    * @param dataProcessingFunction DataProcessingFunction to be applied to the data.
    * @see DataProcessingFunction
    */
   public void applyDataProcessingFunctionBackward(DataProcessingFunction dataProcessingFunction)
   {
      myDataBuffer.applyDataProcessingFunctionBackward(dataProcessingFunction);
   }

   /**
    * Adds the specified SelectedListener.  This listener will be able to react to any mouse event on any camera and is provided with the MouseEvent,  point clicked, camera position, and camera fix point.  Points are provided as Point3D objects.
    * For more information see the SelectedListener class.
    *
    * @param selectedListener SelectedListener
    * @see SelectedListener SelectedListener
    */
   public void attachSelectedListener(SelectedListener selectedListener)
   {
      if (myGUI != null)
      {
         myGUI.attachSelectedListener(selectedListener);
      }
   }

   /**
    * Adds the specified SimulationRewoundListener. This listener will be able to react when the simulation is rewound.
    *
    * @param simulationRewoundListener SimulationRewoundListener
    */
   public void attachSimulationRewoundListener(RewoundListener simulationRewoundListener)
   {
      myDataBuffer.attachSimulationRewoundListener(simulationRewoundListener);
   }

   /**
    * Indicates whether or not KeyPoints are in use.  Key points are bookmarks in the data, when KeyPoints are in use steps during playback will only  move between KeyPoints.
    *
    * @return Are key points in use?
    */
   @Override
   public boolean isKeyPointModeToggled()
   {
      return standardAllCommandsExecutor.isKeyPointModeToggled();
   }

   /**
    * Toggle between enabling and disabling the use of KeyPoints.   Key points are bookmarks in the data, when KeyPoints are in use steps during playback will only  move between KeyPoints.
    */
   @Override
   public void toggleKeyPointMode()
   {
      standardAllCommandsExecutor.toggleKeyPointMode();
   }

   @Override
   public void registerToggleKeyPointModeCommandListener(ToggleKeyPointModeCommandListener listener)
   {
      standardAllCommandsExecutor.registerToggleKeyPointModeCommandListener(listener);
   }

   /**
    * Toggle between enabling and disabling the use of CameraKeyPoints.
    */
   @Override
   public void toggleCameraKeyMode()
   {
      standardAllCommandsExecutor.toggleCameraKeyMode();
   }

   public void addExtraJpanel(Component extraPanel, String name)
   {
      ExtraPanelConfiguration extraPanelConfig = new ExtraPanelConfiguration(name);
      extraPanelConfig.setupPanel(extraPanel);
      extraPanelConfig.setName(name);
      setupExtraPanel(extraPanelConfig);
   }

   public void exportRobotDefinition(Robot robot, File chosenFile)
   {
      disableGUIComponents();

      try
      {
         Writer output = null;

         output = new BufferedWriter(new FileWriter(chosenFile));
         output.write(writeRobotConfig(robot));

         output.close();
      }
      catch (Exception e)
      {
         System.err.println("Error While Saving Robot Configuration File");
         e.printStackTrace();
      }

      enableGUIComponents();
   }

   private String writeRobotConfig(Robot r)
   {
      String textToWrite = "";
      RobotDefinitionFixedFrame rd = new RobotDefinitionFixedFrame();
      rd.createRobotDefinitionFromRobot(r);
      textToWrite += rd.toString();

      return textToWrite;
   }

   public static void main(String[] args)
   {
      JFileChooser fileChooser = new JFileChooser(new File("."));
      int returnVal = fileChooser.showOpenDialog(new JFrame());

      if (returnVal == JFileChooser.APPROVE_OPTION)
      {
         File file = fileChooser.getSelectedFile();
         SimulationConstructionSet scs = generateSimulationFromDataFile(file);

         Thread thread = new Thread(scs);
         thread.start();
      }
      else
      {
         System.err.println("Open command cancelled by user.");
      }

   }

   public String getRunningName()
   {
      return runningName;
   }

   public void setRunName(String name)
   {
      runningName = name;
   }

   @Override
   public double getTime()
   {
      return robots[0].getTime();
   }

   public void setTime(double time)
   {
      for (Robot robot : robots)
      {
         robot.setTime(time);
      }
   }

   public VarGroupList getVarGroupList()
   {
      return varGroupList;
   }

   public static String getVersion()
   {
      return "12.06.22";
   }

   @Override
   public int getInPoint()
   {
      return myDataBuffer.getInPoint();
   }

   @Override
   public void setIndex(int index)
   {
      myDataBuffer.setIndex(index);
   }

   @Override
   public void setIndexButDoNotNotifySimulationRewoundListeners(int index)
   {
      myDataBuffer.setIndexButDoNotNotifySimulationRewoundListeners(index);
   }

   @Override
   public int getOutPoint()
   {
      return myDataBuffer.getOutPoint();
   }

   public GraphicsDynamicGraphicsObject addYoGraphic(YoGraphic yoGraphicObject)
   {
      return addYoGraphic(yoGraphicObject, true);
   }

   public GraphicsDynamicGraphicsObject addYoGraphic(YoGraphic yoGraphic, boolean updateFromSimulationThread)
   {
      if (myGUI != null)
      {
         return myGUI.addYoGraphic(yoGraphic, updateFromSimulationThread);
      }
      else
      {
         return null;
      }
   }

   public void addYoGraphicsList(YoGraphicsList yoGraphicsList, boolean updateFromSimulationThread)
   {
      if (myGUI != null)
         myGUI.addYoGraphicsList(yoGraphicsList, updateFromSimulationThread);
   }

   public void addYoGraphicsList(YoGraphicsList yoGraphicsList, boolean updateFromSimulationThread, List<GraphicsUpdatable> graphicsUpdatablesToPack)
   {
      if (myGUI != null)
         myGUI.addYoGraphicsList(yoGraphicsList, updateFromSimulationThread, graphicsUpdatablesToPack);
   }

   public void addYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry)
   {
      addYoGraphicsListRegistry(yoGraphicsListRegistry, true);
   }

   public void addYoGraphicsListRegistry(YoGraphicsListRegistry yoGraphicsListRegistry, boolean updateFromSimulationThread)
   {
      if (yoGraphicsListRegistry.areYoGraphicsRegistered())
         throw new RuntimeException("Already added this YoGraphicsListRegistry To SimulationConstructionSet: " + yoGraphicsListRegistry);

      ArrayList<GraphicsUpdatable> graphicsUpdatablesToUpdateInAPlaybackListener = yoGraphicsListRegistry.getGraphicsUpdatablesToUpdateInAPlaybackListener();
      if (graphicsUpdatablesToUpdateInAPlaybackListener != null)
      {

         GraphicsUpdatablePlaybackListener playbackListener = new GraphicsUpdatablePlaybackListener(graphicsUpdatablesToUpdateInAPlaybackListener);
         this.attachPlaybackListener(playbackListener);
      }

      if (myGUI != null)
         myGUI.addYoGraphicsListRegistry(yoGraphicsListRegistry, updateFromSimulationThread);

      List<YoGraphicsList> yoGraphicsLists = yoGraphicsListRegistry.getYoGraphicsLists();

      if (yoGraphicsLists == null) return;

      if(dynamicGraphicMenuManager != null)
      {
         addCheckBoxesToDynamicGraphicCheckBoxMenuItem(yoGraphicsLists);
         displayYoGraphicMenu();
      }

      yoGraphicListRegistries.add(yoGraphicsListRegistry);
   }

   private void addCheckBoxesToDynamicGraphicCheckBoxMenuItem(final List<YoGraphicsList> yoGraphicsLists)
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            for (YoGraphicsList yoGraphicsList : yoGraphicsLists)
            {
               if (yoGraphicsList == null)
               {
                  System.out.println("graphics list is null");
                  continue;
               }

               String label = yoGraphicsList.getLabel();
               ArrayList<YoGraphic> yoGraphics = yoGraphicsList.getYoGraphics();
               boolean selectedState = yoGraphicsList.checkAllYoGraphicsAreShowing();
               DynamicGraphicCheckBoxMenuItem checkBox = new DynamicGraphicCheckBoxMenuItem(label, yoGraphics, selectedState);
               dynamicGraphicMenuManager.addCheckBox(checkBox);
            }
         }
      });
   }

   public void hideAllDynamicGraphicObjects()
   {
      int numberOfElements = yoGraphicListRegistries.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicListRegistries.get(i);
         yoGraphicsListRegistry.hideYoGraphics();
      }
   }

   public boolean checkAllDynamicGraphicObjectsListRegistriesAreShowing()
   {
      boolean ret = true;
      int numberOfElements = yoGraphicListRegistries.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicListRegistries.get(i);
         ret = ret && yoGraphicsListRegistry.checkAllYoGraphicsListAreShowing();
      }

      return ret;
   }

   public void setDynamicGraphicObjectsListVisible(String name, boolean visible)
   {
      ArrayList<YoGraphicsList> lists = new ArrayList<YoGraphicsList>();
      int numberOfElements = yoGraphicListRegistries.size();

      for (int i = 0; i < numberOfElements; i++)
      {
         YoGraphicsListRegistry yoGraphicsListRegistry = yoGraphicListRegistries.get(i);
         lists.clear();
         yoGraphicsListRegistry.getRegisteredDynamicGraphicObjectsLists(lists);
         int n = lists.size();

         for (int j = 0; j < n; j++)
         {
            YoGraphicsList yoGraphicsList = lists.get(j);
            if (yoGraphicsList.getLabel().equals(name))
            {
               yoGraphicsList.setVisible(visible);
            }
         }
      }
   }

   public void bringToFront()
   {
      jFrame.toFront();
      jFrame.repaint();

   }

   public ArrayList<YoGraphicsListRegistry> getDynamicGraphicObjectsListRegistries()
   {
      return yoGraphicListRegistries;
   }

   private void displayYoGraphicMenu()
   {
      EventDispatchThreadHelper.invokeAndWait(new Runnable()
      {
         @Override
         public void run()
         {
            JMenuBar menuBar = new JMenuBar();
            menuBar.add(dynamicGraphicMenuManager.getjMenu());
            addMenuBar(menuBar);
         }
      });
   }

   public Graphics3DAdapter getGraphics3dAdapter()
   {
      if (myGUI != null)
      {
         return myGUI.getGraphics3dAdapter();
      }

      return null;
   }

   public void startStreamingVideoData(CameraConfiguration cameraConfiguration, int width, int height, RenderedSceneHandler videoDataServer,
         TimestampProvider timestampProvider, int framesPerSecond)
   {
      if (myGUI != null)
      {
         myGUI.startStreamingVideoData(cameraConfiguration, width, height, videoDataServer, timestampProvider, framesPerSecond);
      }
   }

   private boolean systemExitDisabled = false;

   public void disableSystemExit()
   {
      systemExitDisabled = true;
   }

   public void enableSystemExit()
   {
      systemExitDisabled = false;
   }

   public boolean systemExitDisabled()
   {
      return systemExitDisabled;
   }

   public StandardSimulationGUI getGUI()
   {
      return myGUI;
   }

   public ScsPhysics getPhysics()
   {
      return physics;
   }

   // TODO: this should be moved to the constructor once it is fully integrated
   public void initPhysics(ScsPhysics physics)
   {
      this.physics = physics;
      mySimulation.initPhysics(physics);
   }

   public void addForceSensor(WrenchContactPoint sensor)
   {
      mySimulation.addForceSensor(sensor);
   }

   public ArrayList<WrenchContactPoint> getForceSensors()
   {
      return mySimulation.getForceSensors();
   }

   public void repaintWindows()
   {
      if (myGUI != null)
      {
         myGUI.repaintWindows();
      }
   }

   public void initializeCollisionDetectionAndHandling(DefaultCollisionVisualizer collisionVisualizer, CollisionHandler collisionHandler)
   {
      mySimulation.initializeCollisionDetectionAndHandling(collisionVisualizer, collisionHandler);
   }

}
