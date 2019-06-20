package us.ihmc.exampleSimulations.springflamingo;

import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JOptionPane;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationConstructionSetTools.dataExporter.TorqueSpeedDataExporter;
import us.ihmc.simulationConstructionSetTools.util.visualizers.RobotFreezeFramer;
import us.ihmc.simulationconstructionset.DynamicIntegrationMethod;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.SimulationConstructionSetParameters;
import us.ihmc.simulationconstructionset.SupportedGraphics3DAdapter;
import us.ihmc.simulationconstructionset.ViewportConfiguration;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.simulationconstructionset.util.simulationRunner.StateFileComparer;
import us.ihmc.simulationconstructionset.util.simulationRunner.VariableDifference;

public class SpringFlamingoSimulation
{
   private static final SupportedGraphics3DAdapter graphics3DAdapterToUse = SupportedGraphics3DAdapter.JAVA_MONKEY_ENGINE;
//   private static final SupportedGraphics3DAdapter graphics3DAdapterToUse = SupportedGraphics3DAdapter.JAVA3D;

   private static final boolean SHOW_MULTIPLE_VIEWPORT_WINDOW = false;

   private static final int
      BALLISTIC_WALKING_CONTROLLER = 0, FAST_WALKING_CONTROLLER = 1;

// private static int controllerToUse = FAST_WALKING_CONTROLLER;
   private static int controllerToUse = BALLISTIC_WALKING_CONTROLLER;

   public static double DT = 0.0001;
   public static int TICKS_PER_RECORD = 100;
   public static int TICKS_PER_CONTROL = 10;

   static
   {
      if (controllerToUse == BALLISTIC_WALKING_CONTROLLER)
      {
         DT = 0.0004;
         TICKS_PER_RECORD = 25;
         TICKS_PER_CONTROL = 4;
      }
   }

   // Here's a change to test.

   public static final double EARTH_GRAVITY = 9.81;
   public static final double MOON_GRAVITY = EARTH_GRAVITY / 6.0;

   private static final boolean ENABLE_FREEZE_FRAME = true;


   private boolean BENCHMARK = false;
   private boolean RUN_AND_COMPARE = false;
   private boolean SHOW_GUI = true;

   private boolean SHOW_EXPORT_TORQUE_AND_SPEED = true;

   private final SimulationConstructionSet sim;

   public SpringFlamingoSimulation() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      double baseGravity = EARTH_GRAVITY;    // MOON_GRAVITY; //1.5 * EARTH_GRAVITY; //4.0; //9.81;
      double minGravity = MOON_GRAVITY;    // 1.64; //4.0;

      int numberOfFlamingos = 1;
      if (controllerToUse == FAST_WALKING_CONTROLLER)
      {
         numberOfFlamingos = 6;
      }

      Robot[] springFlamingos = new Robot[numberOfFlamingos];

      for (int i = 0; i < numberOfFlamingos; i++)
      {
         String name = "SpringFlamingoRobot";
         if (i > 0)
            name = name + i;
         SpringFlamingoRobot springFlamingoConstructor = new SpringFlamingoRobot(name);

         Robot springFlamingo = springFlamingoConstructor.getRobot();

         springFlamingo.setDynamicIntegrationMethod(DynamicIntegrationMethod.EULER_DOUBLE_STEPS);
         springFlamingos[i] = springFlamingo;

         // springFlamingo.createControllerBase(System.out, "SpringFlamingo");
         double gravity = -baseGravity;
         if (numberOfFlamingos > 1)
            gravity = -(baseGravity - (((double) i) / ((double) (numberOfFlamingos - 1))) * (baseGravity - minGravity));



         // System.out.println(springFlamingo);
         RobotController controller = null;
         if (controllerToUse == BALLISTIC_WALKING_CONTROLLER)
         {
            controller = new SpringFlamingoController(springFlamingoConstructor, "springFlamingoController");
//            controller = new SpringFlamingoController(springFlamingo, "springFlamingoController", icpVisualizer); //TODO
//            System.out.println("I am using your controller");
         }
         else if (controllerToUse == FAST_WALKING_CONTROLLER)
         {
            controller = new SpringFlamingoFastWalkingController(springFlamingoConstructor, gravity, "springFlamingoFastWalkingController");
         }

         springFlamingo.setController(controller, TICKS_PER_CONTROL);


         if (controllerToUse == BALLISTIC_WALKING_CONTROLLER)
         {
            springFlamingo.setGroundContactModel(new LinearGroundContactModel(springFlamingo, 14220, 150.6, 125.0, 300.0, springFlamingo.getRobotsYoVariableRegistry()));
         }
         else if (controllerToUse == FAST_WALKING_CONTROLLER)
         {
            springFlamingo.setGroundContactModel(new LinearGroundContactModel(springFlamingo, 50000.0, 2500.0, 100.0, 500.0, springFlamingo.getRobotsYoVariableRegistry()));

            // springFlamingo.setGroundContactModel(new CollisionGroundContactModel(this, 0.1, 0.7));
         }

         double xStart = 0.0 + 1.5 * ((double) i);

         System.out.println("xStart = " + xStart + ", gravity = " + gravity);
         springFlamingoConstructor.q_x.set(xStart);
         springFlamingo.setGravity(gravity);
      }


      if (!SHOW_GUI)
      {
         SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
         parameters.setCreateGUI(false);
         parameters.setDataBufferSize(512);

         sim = new SimulationConstructionSet(springFlamingos, parameters);
      }
      else
      {
         int initialBufferSize = 4*8192;
         sim = new SimulationConstructionSet(springFlamingos, graphics3DAdapterToUse, new SimulationConstructionSetParameters(initialBufferSize));
      }

      Graphics3DObject coneGraphics = new Graphics3DObject();
      coneGraphics.translate(-20.0, -2.0, 0.0);
      coneGraphics.addGenTruncatedCone(0.5, 0.25, 0.25, 0.1, 0.1, YoAppearance.Red());
      sim.addStaticLinkGraphics(coneGraphics);

      CameraConfiguration camera1 = new CameraConfiguration("camera 1");
      camera1.setCameraFix(0.0, 0.0, 0.6);
      camera1.setCameraPosition(-9.0, 3.0, 0.8);
      camera1.setCameraTracking(true, true, true, false);
      camera1.setCameraDolly(false, true, true, false);
      sim.setupCamera(camera1);

      CameraConfiguration camera2 = new CameraConfiguration("camera 2");
      camera2.setCameraFix(0.0, 0.0, 0.6);
      camera2.setCameraPosition(-9.0, 3.0, 0.15);
      camera2.setCameraTracking(true, true, true, false);
      camera2.setCameraDolly(true, true, true, false);
      camera2.setCameraDollyOffsets(-3.6, 4.0, 0.0);
      camera2.setCameraTrackingOffsets(0.0, 0.0, 0.0);
      sim.setupCamera(camera2);

      CameraConfiguration camera3 = new CameraConfiguration("camera 3");
      camera3.setCameraFix(-9.0, 0.0, 0.6);
      camera3.setCameraPosition(-9.0, 0.01, 8.0);
      camera3.setCameraTracking(false, true, true, false);
      camera3.setCameraDolly(false, true, true, false);
      camera3.setCameraDollyOffsets(0.0, 0.01, 8.0);
      camera3.setCameraTrackingOffsets(0.0, 0.0, 0.0);
      sim.setupCamera(camera3);

      CameraConfiguration camera4 = new CameraConfiguration("robot cam");
      camera4.setCameraMount("robot cam mount");
      sim.setupCamera(camera4);

      ViewportConfiguration view1 = new ViewportConfiguration("view1");
      view1.addCameraView("camera 1", 0, 0, 2, 2);

      ViewportConfiguration view2 = new ViewportConfiguration("view2");
      view2.addCameraView("camera 1", 0, 0, 2, 2);
      view2.addCameraView("camera 2", 2, 0, 2, 2);
      view2.addCameraView("robot cam", 0, 2, 4, 1);

      ViewportConfiguration view3 = new ViewportConfiguration("view3");
      view3.addCameraView("camera 1", 0, 0, 1, 1);
      view3.addCameraView("camera 2", 1, 0, 2, 1);

//    view3.addCameraView("nothing", 1, 0, 1, 1);  // This needs to be fixed. If comment out this line, get different behavior. Fixed!
      view3.addCameraView("camera 1", 0, 1, 2, 1);
      view3.addCameraView("robot cam", 2, 1, 1, 1);

      sim.setupViewport(view1);
      sim.setupViewport(view2);
      sim.setupViewport(view3);

      if (SHOW_MULTIPLE_VIEWPORT_WINDOW)
      {
         sim.hideViewport();

         if (SHOW_GUI)
         {
            sim.createNewViewportWindow("view3", 1, false);
         }
      }
      else
      {
         sim.selectViewport("view1");
      }

      sim.setClipDistances(0.1, 100.0);
      sim.setFieldOfView(1.0);

      sim.setDT(DT, TICKS_PER_RECORD);
      sim.setPlaybackRealTimeRate(1.0);
      sim.setPlaybackDesiredFrameRate(0.033);
      sim.setGraphsUpdatedDuringPlayback(true);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();//TODO
//      ICPVisualizer icpVisualizer = new ICPVisualizer(registry, yoGraphicsListRegistry);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry); //TODO

      // Set up VarGroups, GraphGroups, EntryBoxGroups, and Configs.
      sim.setupVarGroup("kinematics", new String[] {"t"}, new String[] {"q_.*", "qd_.*"});

      sim.setupVarGroup("torques", null, new String[] {"t", "tau_.*"});

      sim.setupGraphGroup("states", new String[][]
      {
         {"leftState", "rightState"}, {"t"}, {"q_x", "q_z"}, {"l_swing_flag", "r_swing_flag"}
      });
      sim.setupGraphGroup("joint angles", new String[][]
      {
         {"leftState", "rightState"}, {"q_lh"}, {"q_lk"}, {"q_la"}, {"q_rh"}, {"q_rk"}, {"q_ra"}
      });
      sim.setupGraphGroup("joint velocities", new String[][]
      {
         {"qd_lh"}, {"qd_lk"}, {"qd_la"}, {"qd_rh"}, {"qd_rk"}, {"qd_ra"}, {"average_qd_la"}
      });
      sim.setupGraphGroup("joint torques", new String[][]
      {
         {"tau_lh"}, {"tau_lk"}, {"tau_la"}, {"tau_rh"}, {"tau_rk"}, {"tau_ra"}
      }, 2);

      sim.setupEntryBoxGroup("control vars1", new String[]
      {
         "t_gain", "t_damp", "hip_d", "hip_hold", "hip_down", "hip_gain", "hip_damp", "swing_gain_knee", "swing_damp_knee", "q_x", "q_y", "q_z"
      });

      sim.setupConfiguration("states", "all", "states", "control vars1");
      sim.setupConfiguration("joint angles", "all", "joint angles", "control vars1");
      sim.setupConfiguration("joint velocities", "all", "joint velocities", "control vars1");
      sim.setupConfiguration("joint torques", "all", "joint torques", "control vars1");

      sim.selectConfiguration("states");

//    sim.selectConfiguration("joint angles");

      sim.setFastSimulate(true);

//      if (controllerToUse == BALLISTIC_WALKING_CONTROLLER)
//      {
//         sim.readState("resources/initial_state.state");
//      }
//      else if (controllerToUse == FAST_WALKING_CONTROLLER)
//      {
////       sim.readState("resources/initial_state.state");
//      }

      if (ENABLE_FREEZE_FRAME & SHOW_GUI)
      {
         RobotFreezeFramer robotFreezeFramer = new RobotFreezeFramer(springFlamingos[0], sim);
         robotFreezeFramer.setDoFreezeFrame(false);
         robotFreezeFramer.setFreezeInterval(1.5);
         robotFreezeFramer.setNextFreezeTime(0.6);
         springFlamingos[0].setController(robotFreezeFramer);
      }
      
      sim.setParameterRootPath(sim.getRootRegistry());

      Thread myThread = new Thread(sim);
      myThread.start();

      if (BENCHMARK)
      {
         long startTime = System.currentTimeMillis();

         sim.simulate();

         while (true)
         {
            try
            {
               Thread.sleep(10000);
            }
            catch (InterruptedException e)
            {
            }

            double elapsedTime = (System.currentTimeMillis() - startTime) / 1000.0;

            System.out.println("t = " + sim.getTime() + "   elapsed_time = " + elapsedTime + "   RealTimeRate = " + sim.getTime() / elapsedTime);
         }
      }

      else if (RUN_AND_COMPARE)
      {
         BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(sim, 600.0);
         blockingSimulationRunner.simulateAndBlock(5.0);

//       sim.writeState("simAfter5Seconds.state");
         sim.writeState("newSimAfter5Seconds.state");

         ArrayList<VariableDifference> variableDifferences = StateFileComparer.absoluteCompareStateFiles("simAfter5Seconds.state", "newSimAfter5Seconds.state", 0.001, null);

         if (variableDifferences.isEmpty())
            System.out.println("Sims are the same!");
         else
         {
            String message = "Variables changed: \n" + VariableDifference.allVariableDifferencesToString(variableDifferences);

            System.err.println(message);
            JOptionPane.showMessageDialog(null, message);
         }
      }
      
      if (SHOW_EXPORT_TORQUE_AND_SPEED)
      {
         JButton exportTorqueAndSpeedButton = new JButton("Export Torque And Speed");
         TorqueSpeedDataExporter dataExporter = new TorqueSpeedDataExporter(sim, springFlamingos[0]);
         dataExporter.setRootDirectory("D:/DataAndVideos");
         exportTorqueAndSpeedButton.addActionListener(dataExporter);
         sim.addButton(exportTorqueAndSpeedButton);
      }
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return sim;
   }

   public static void main(String[] args) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      new SpringFlamingoSimulation();
   }
}
