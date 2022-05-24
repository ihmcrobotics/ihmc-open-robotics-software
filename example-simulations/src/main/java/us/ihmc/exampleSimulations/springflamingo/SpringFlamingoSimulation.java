package us.ihmc.exampleSimulations.springflamingo;

import java.util.ArrayList;
import java.util.Random;

import javax.swing.JButton;
import javax.swing.JOptionPane;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.camera.CameraConfiguration;
import us.ihmc.simulationConstructionSetTools.dataExporter.TorqueSpeedDataExporter;
import us.ihmc.simulationConstructionSetTools.util.ground.CombinedTerrainObject3D;
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

   // Choose the controller here. Leap of faith will do step downs nicely. Ballistic is simple, exploiting natural dynamics. Fast can be faster and does speed control.
   private enum FlamingoController
   {
      BALLISTIC_WALKING_CONTROLLER, FAST_WALKING_CONTROLLER, LEAP_OF_FAITH_CONTROLLER;
   }

//      private static FlamingoController controllerToUse = FlamingoController.FAST_WALKING_CONTROLLER;
//      private static FlamingoController controllerToUse = FlamingoController.BALLISTIC_WALKING_CONTROLLER;
   private static FlamingoController controllerToUse = FlamingoController.LEAP_OF_FAITH_CONTROLLER;

   
   // Choose a Terrain here if the robot is doing the Leap of Faith controller. If not, it can only walk on flat.
   private enum TerrainIfLeapOfFaithController
   {
      FLAT, SLOPE, SLOPE_AND_STEPS, STEP_DOWNS_AND_FLATS;
   }

   private static TerrainIfLeapOfFaithController terrainIfLeapOfFaith = TerrainIfLeapOfFaithController.STEP_DOWNS_AND_FLATS;
//   private static TerrainIfLeapOfFaithController terrainIfLeapOfFaith = TerrainIfLeapOfFaithController.SLOPE_AND_STEPS;
//   private static TerrainIfLeapOfFaithController terrainIfLeapOfFaith = TerrainIfLeapOfFaithController.SLOPE;
//   private static TerrainIfLeapOfFaithController terrainIfLeapOfFaith = TerrainIfLeapOfFaithController.FLAT;

   public static double DT = 0.0001;
   public static int TICKS_PER_RECORD = 10;
   public static int TICKS_PER_CONTROL = 10;

   static
   {
      switch (controllerToUse)
      {
         case FAST_WALKING_CONTROLLER:
            DT = 0.0001;
            TICKS_PER_RECORD = 100;
            TICKS_PER_CONTROL = 10;
            break;
         case BALLISTIC_WALKING_CONTROLLER:
            DT = 0.0004;
            TICKS_PER_RECORD = 25;
            TICKS_PER_CONTROL = 4;
            break;
         case LEAP_OF_FAITH_CONTROLLER:
            DT = 0.0001;
            TICKS_PER_RECORD = 10;
            TICKS_PER_CONTROL = 10;
            break;
      }
   }

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
      double baseGravity = EARTH_GRAVITY; // MOON_GRAVITY; //1.5 * EARTH_GRAVITY; //4.0; //9.81;
      double minGravity = MOON_GRAVITY; // 1.64; //4.0;

      int numberOfFlamingos = 1;
      if (controllerToUse == FlamingoController.FAST_WALKING_CONTROLLER)
      {
         numberOfFlamingos = 6;
      }

      Robot[] springFlamingos = new Robot[numberOfFlamingos];

      CombinedTerrainObject3D profile3D = null;
      if (controllerToUse == FlamingoController.LEAP_OF_FAITH_CONTROLLER)
      {
         switch (terrainIfLeapOfFaith)
         {
            case SLOPE:
               profile3D = createSlope();
               break;
            case SLOPE_AND_STEPS:
               profile3D = createSlopeAndStepTerrainObject();
               break;
            case STEP_DOWNS_AND_FLATS:
               profile3D = createStepDownsAndFlatsTerrain();
               break;
            case FLAT:
               break;
         }

      }

      for (int i = 0; i < numberOfFlamingos; i++)
      {
         String name = "SpringFlamingoRobot";
         if (i > 0)
            name = name + i;
         SpringFlamingoRobot springFlamingoConstructor = new SpringFlamingoRobot(name);

         Robot springFlamingo = springFlamingoConstructor.getRobot();

         springFlamingo.setDynamicIntegrationMethod(DynamicIntegrationMethod.EULER_DOUBLE_STEPS);
         springFlamingos[i] = springFlamingo;

         double gravity = -baseGravity;
         if (numberOfFlamingos > 1)
            gravity = -(baseGravity - (((double) i) / ((double) (numberOfFlamingos - 1))) * (baseGravity - minGravity));

         RobotController controller = null;
         LinearGroundContactModel gcModel = null;

         switch (controllerToUse)
         {
            case BALLISTIC_WALKING_CONTROLLER:
               controller = new SpringFlamingoController(springFlamingoConstructor, "springFlamingoController");
               gcModel = new LinearGroundContactModel(springFlamingo, 14220, 150.6, 125.0, 300.0, springFlamingo.getRobotsYoRegistry());
               break;
            case FAST_WALKING_CONTROLLER:
               controller = new SpringFlamingoFastWalkingController(springFlamingoConstructor, gravity, "springFlamingoFastWalkingController");
               gcModel = new LinearGroundContactModel(springFlamingo, 50000.0, 2500.0, 100.0, 500.0, springFlamingo.getRobotsYoRegistry());
               break;
            case LEAP_OF_FAITH_CONTROLLER:
               controller = new SpringFlamingoLeapOfFaithController(springFlamingoConstructor);
               gcModel = new LinearGroundContactModel(springFlamingo, 14220, 150.6, 125.0, 300.0, springFlamingo.getRobotsYoRegistry());
               break;
         }

         springFlamingo.setController(controller, TICKS_PER_CONTROL);

         if (profile3D != null)
            gcModel.setGroundProfile3D(profile3D);
         springFlamingo.setGroundContactModel(gcModel);

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
         int initialBufferSize = 2 * 8192;
         sim = new SimulationConstructionSet(springFlamingos, graphics3DAdapterToUse, new SimulationConstructionSetParameters(initialBufferSize));
      }

      if (profile3D != null)
      {
         sim.setGroundVisible(false);
         sim.addStaticLinkGraphics(profile3D.getLinkGraphics());
      }

      setupCamerasAndViewports(sim);

      sim.setDT(DT, TICKS_PER_RECORD);
      sim.setPlaybackRealTimeRate(1.0);
      sim.setPlaybackDesiredFrameRate(0.033);
      sim.setGraphsUpdatedDuringPlayback(true);

//      sim.setSimulateDuration(3.0);
      sim.setSimulateDuration(12.5);
      sim.setSimulateNoFasterThanRealTime(false);

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();//TODO
      //      ICPVisualizer icpVisualizer = new ICPVisualizer(registry, yoGraphicsListRegistry);
      sim.addYoGraphicsListRegistry(yoGraphicsListRegistry); //TODO

      // Set up VarGroups, GraphGroups, EntryBoxGroups, and Configs.
      sim.setupVarGroup("kinematics", new String[] {"t"}, new String[] {"q_.*", "qd_.*"});

      sim.setupVarGroup("torques", null, new String[] {"t", "tau_.*"});

      sim.setupGraphGroup("states", new String[][] {{"leftState", "rightState"}, {"t"}, {"q_x", "q_z"}, {"l_swing_flag", "r_swing_flag"}});
      sim.setupGraphGroup("joint angles", new String[][] {{"leftState", "rightState"}, {"q_lh"}, {"q_lk"}, {"q_la"}, {"q_rh"}, {"q_rk"}, {"q_ra"}});
      sim.setupGraphGroup("joint velocities", new String[][] {{"qd_lh"}, {"qd_lk"}, {"qd_la"}, {"qd_rh"}, {"qd_rk"}, {"qd_ra"}, {"average_qd_la"}});
      sim.setupGraphGroup("joint torques", new String[][] {{"tau_lh"}, {"tau_lk"}, {"tau_la"}, {"tau_rh"}, {"tau_rk"}, {"tau_ra"}}, 2);

      sim.setupEntryBoxGroup("control vars1",
                             new String[] {"t_gain", "t_damp", "hip_d", "hip_hold", "hip_down", "hip_gain", "hip_damp", "swing_gain_knee", "swing_damp_knee",
                                   "q_x", "q_y", "q_z"});

      sim.setupConfiguration("states", "all", "states", "control vars1");
      sim.setupConfiguration("joint angles", "all", "joint angles", "control vars1");
      sim.setupConfiguration("joint velocities", "all", "joint velocities", "control vars1");
      sim.setupConfiguration("joint torques", "all", "joint torques", "control vars1");

      sim.selectConfiguration("states");

      //    sim.selectConfiguration("joint angles");

      sim.setFastSimulate(true);

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

         ArrayList<VariableDifference> variableDifferences = StateFileComparer.absoluteCompareStateFiles("simAfter5Seconds.state",
                                                                                                         "newSimAfter5Seconds.state",
                                                                                                         0.001,
                                                                                                         null);

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

   private void setupCamerasAndViewports(SimulationConstructionSet sim2)
   {
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

   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return sim;
   }

   private CombinedTerrainObject3D createSlope()
   {
      Vector3D surfaceNormal = new Vector3D(-0.35, 0.0, 1.0);
      double maxXY = 10.0;

      surfaceNormal.normalize();

      double yStart = -1.0;
      double yEnd = 1.0;

      double xStart = -maxXY;
      double xEnd = 0.0;

      double zStart = surfaceNormal.getX() / surfaceNormal.getZ() * maxXY;
      double zEnd = 0.0;

      CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Terrain");
      terrainObject.addRamp(xStart, yStart, xEnd, yEnd, zStart, zEnd, YoAppearance.Green());

      return terrainObject;
   }

   private CombinedTerrainObject3D createSlopeAndStepTerrainObject()
   {
      double rampDownOneStartX = -5.0;
      double rampDownOneEndX = -10.0;
      double rampDownOneBottomZ = -1.0;
      double rampDownTwoEndX = -15.0;
      double rampDownTwoBottomZ = -3.0;

      double dropOffX = -18.0;
      double dropOffBottomZ = rampDownTwoBottomZ - 0.15;
      double dropOffEndX = -22.0;

      //      double rampUpStartX = 18.0;
      //      double rampUpEndX = 40.0;

      CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Terrain");
      terrainObject.addBox(3.0, -1.0, rampDownOneStartX, 1.0, -0.01, 0.0);
      terrainObject.addRamp(rampDownOneEndX, -1.0, rampDownOneStartX, 1.0, rampDownOneBottomZ, 0.0, YoAppearance.Red());
      terrainObject.addRamp(rampDownTwoEndX, -1.0, rampDownOneEndX, 1.0, rampDownTwoBottomZ, rampDownOneBottomZ, YoAppearance.Blue());

      terrainObject.addBox(rampDownTwoEndX, -1.0, dropOffX, 1.0, rampDownTwoBottomZ - 0.01, rampDownTwoBottomZ);

      double minimumStepLength = 0.3, maximumStepLength = 0.6, minimumStepHeight = 0.01, maximumStepHeight = 0.18;
      int numberOfSteps = 10;

      Random random = new Random(1776L);
      double stepStartX = dropOffX;
      double stepStartZ = rampDownTwoBottomZ;

      for (int i = 0; i < numberOfSteps; i++)
      {
         double stepEndX = stepStartX - minimumStepLength - random.nextDouble() * (maximumStepLength - minimumStepLength);
         double stepEndZ = stepStartZ - minimumStepHeight - random.nextDouble() * (maximumStepHeight - minimumStepHeight);

         terrainObject.addBox(stepStartX, -1.0, stepEndX, 1.0, stepEndZ, stepStartZ);

         stepStartX = stepEndX;
         stepStartZ = stepEndZ;
      }

      terrainObject.addBox(stepStartX, -1.0, stepStartX + 1.0, 1.0, stepStartZ - 0.01, stepStartZ);

      //      terrainObject.addRamp(rampUpStartX, -1.0, rampUpEndX, 1.0, rampDownTwoBottomZ, 0.0, YoAppearance.Red());

      //      RampTerrainObject downRamp = new RampTerrainObject(10.0, -1.0, 5.0, 1.0, -1.0, 0.0);

      //      terrainObject.addRamp(3.0, -1.0, 20.0, 1.0, 1.0, YoAppearance.AliceBlue());
      //      Vector3D surfaceNormal = new Vector3D(0.5, 0.0, 1.0);
      //      Point3D intersectionPoint = new Point3D();
      //      double maxXY = 100.0;
      //      SlopedPlaneGroundProfile profile3D = new SlopedPlaneGroundProfile(surfaceNormal, intersectionPoint, maxXY);
      return terrainObject;
   }

   private CombinedTerrainObject3D createStepDownsAndFlatsTerrain()
   {
//      double[] stepDownXs = new double[] {3.0, -2.40, -4.95, -9.0};
//      double[] stepDownBottomZs = new double[] {0.0, -0.25, -0.5};
//      double[] stepDownBottomZs = new double[] {0.0, -0.10, -0.2};
      
      double[] stepDownXs = new double[] {3.0, -0.75, -2, -3.1, -3.9, -4.6, -5.4, -10.0};
      double[] stepDownBottomZs = new double[] {0.0, -0.05, -0.15, -0.30, -0.50, -0.75, -1.0};
      
      AppearanceDefinition[] appearance = new AppearanceDefinition[] 
            {YoAppearance.Green(), YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Red(), YoAppearance.Green(), YoAppearance.Red()};

      CombinedTerrainObject3D terrainObject = new CombinedTerrainObject3D("Terrain");
      double yStart = -1.0;
      double yEnd = 1.0;
      
      for (int i=0; i<stepDownXs.length - 1; i++)
      {
         double xStart = stepDownXs[i];
         double stepDownX = stepDownXs[i+1];
         double zHeight = stepDownBottomZs[i];
         
         terrainObject.addBox(xStart, yStart, stepDownX, yEnd, zHeight -0.03, zHeight, appearance[i]);
         xStart = stepDownX;
      }

      return terrainObject;
   }

   public static void main(String[] args) throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      new SpringFlamingoSimulation();
   }
}
