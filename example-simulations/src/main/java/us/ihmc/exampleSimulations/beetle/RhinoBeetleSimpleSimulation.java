package us.ihmc.exampleSimulations.beetle;

import java.util.ArrayList;

import javax.swing.JButton;

import us.ihmc.commons.PrintTools;
import us.ihmc.exampleSimulations.beetle.controller.HexapodSimulationController;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleInverseDynamicsParameters;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleModelFactory;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleSimInitialSetup;
import us.ihmc.exampleSimulations.beetle.parameters.RhinoBeetleVirtualModelControlParameters;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.simulationConstructionSetTools.dataExporter.TorqueSpeedDataExporter;
import us.ihmc.simulationconstructionset.FloatingRootJointRobot;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.gui.SimulationOverheadPlotter;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.tools.inputDevices.joystick.Joystick;
import us.ihmc.tools.inputDevices.joystick.JoystickModel;
import us.ihmc.tools.inputDevices.joystick.exceptions.JoystickNotFoundException;
import us.ihmc.yoVariables.parameters.DefaultParameterReader;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class RhinoBeetleSimpleSimulation
{
   private static final double SIMULATION_DT = 0.0001;
   private static final double CONTROLLER_DT = 0.001;
   private static final boolean SHOW_EXPORT_TORQUE_AND_SPEED = true;

   public RhinoBeetleSimpleSimulation()
   {
      RhinoBeetleModelFactory modelFactory = new RhinoBeetleModelFactory();
      FloatingRootJointRobot sdfRobot = modelFactory.createSdfRobot();
      YoRegistry registry = sdfRobot.getRobotsYoRegistry();
      FullRobotModel fullRobotModel = modelFactory.createFullRobotModel();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      
      ArrayList<String> jointsToControl = new ArrayList<>();
      
      RhinoBeetleInverseDynamicsParameters idParameters = new RhinoBeetleInverseDynamicsParameters(registry);
      RhinoBeetleVirtualModelControlParameters vmcParameters = new RhinoBeetleVirtualModelControlParameters(registry);
      HexapodSimulationController controller = new HexapodSimulationController(fullRobotModel, sdfRobot, jointsToControl, idParameters, vmcParameters, yoGraphicsListRegistry, CONTROLLER_DT);
      sdfRobot.setController(controller, (int) (CONTROLLER_DT / SIMULATION_DT));
      GroundContactModel groundContactModel = createGroundContactModel(sdfRobot);
      sdfRobot.setGroundContactModel(groundContactModel);
      

      SimulationConstructionSet scs = new SimulationConstructionSet(sdfRobot);

      SimulationOverheadPlotterFactory simulationOverheadPlotterFactory = scs.createSimulationOverheadPlotterFactory();
      simulationOverheadPlotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      SimulationOverheadPlotter overheadPlotter = simulationOverheadPlotterFactory.createOverheadPlotter();
      overheadPlotter.setXVariableToTrack((YoDouble) sdfRobot.findVariable("q_x"));
      overheadPlotter.setYVariableToTrack((YoDouble) sdfRobot.findVariable("q_y"));

      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);
      scs.setDT(SIMULATION_DT, 10);

      if (SHOW_EXPORT_TORQUE_AND_SPEED)
      {
         JButton exportTorqueAndSpeedButton = new JButton("Export Torque And Speed");
         TorqueSpeedDataExporter dataExporter = new TorqueSpeedDataExporter(scs, sdfRobot, RhinoBeetleSimpleSimulation.class, getClass().getSimpleName());
         exportTorqueAndSpeedButton.addActionListener(dataExporter);
         scs.addButton(exportTorqueAndSpeedButton);
      }
      
      DefaultParameterReader reader = new DefaultParameterReader();
      reader.readParametersInRegistry(registry);

      RhinoBeetleSimInitialSetup initialSetup = new RhinoBeetleSimInitialSetup();
      initialSetup.initializeRobot(sdfRobot, modelFactory.getJointNameMap());
      scs.startOnAThread();
      scs.simulate();
      
      try
      {
         final Joystick gamePad = new Joystick(JoystickModel.XBOX_ONE, 0);
         HexapodGamePadManager gamePadManager = new HexapodGamePadManager(gamePad, registry);
      }
      catch (JoystickNotFoundException e)
      {
         PrintTools.warn("Xbox controller 1 (for body) not connected!");
      }
   }

   private GroundContactModel createGroundContactModel(FloatingRootJointRobot sdfRobot)
   {
      FlatGroundProfile groundProfile3D = new FlatGroundProfile(0.0);

      LinearGroundContactModel groundContactModel = new LinearGroundContactModel(sdfRobot, sdfRobot.getRobotsYoRegistry());
      groundContactModel.setZStiffness(200.0);
      groundContactModel.setZDamping(250.0);
      groundContactModel.setXYStiffness(5000.0);
      groundContactModel.setXYDamping(100.0);
      groundContactModel.setGroundProfile3D(groundProfile3D);
      return groundContactModel;
   }

   public static void main(String[] args)
   {
      RhinoBeetleSimpleSimulation simulationFactory = new RhinoBeetleSimpleSimulation();
   }
}
