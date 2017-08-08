package us.ihmc.exampleSimulations.sphereICPControl;

import us.ihmc.exampleSimulations.sphereICPControl.controllers.SphereControlToolbox;
import us.ihmc.exampleSimulations.sphereICPControl.model.SphereRobot;
import us.ihmc.exampleSimulations.sphereICPControl.model.SphereRobotModel;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class SphereSimulation
{
   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;

   private final SimulationConstructionSet scs;

   public SphereSimulation()
   {
      Vector3D initialPosition = new Vector3D(0.0, 0.0, 1.0);
      SphereRobotModel sphereRobotModel = new SphereRobotModel();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      RobotTools.SCSRobotFromInverseDynamicsRobotModel sphereRobot = SphereRobot.createSphereRobot("SphereRobot", initialPosition,
            sphereRobotModel.getElevator(), yoGraphicsListRegistry, gravity);

      ExternalForcePoint externalForcePoint = sphereRobot.getAllExternalForcePoints().get(0);

      SphereControlToolbox sphereControlToolbox = new SphereControlToolbox(sphereRobotModel, controlDT, desiredHeight, gravity, sphereRobot.getYoTime(),
            sphereRobot.getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
      SphereController controller = new SphereController(sphereRobot, sphereControlToolbox, externalForcePoint);
      sphereRobot.setController(controller);

      setupGroundContactModel(sphereRobot);

      SimulationConstructionSetParameters parameters = new SimulationConstructionSetParameters();
      parameters.setDataBufferSize(16000);
      scs = new SimulationConstructionSet(sphereRobot, parameters);
      SimulationOverheadPlotterFactory plotterFactory = scs.createSimulationOverheadPlotterFactory();
      plotterFactory.setShowOnStart(true);
      plotterFactory.setVariableNameToTrack("centerOfMass");
      plotterFactory.addYoGraphicsListRegistries(yoGraphicsListRegistry);
      plotterFactory.createOverheadPlotter();

      scs.setDT(controlDT, 1);

      scs.setCameraPosition(-1.5, -2.5, 0.5);
      scs.setCameraFix(0.0, 0.0, 0.4);

      scs.setCameraTracking(false, true, true, false);
      scs.setCameraDolly(false, true, true, false);


      scs.startOnAThread();

   }

   private void setupGroundContactModel(Robot robot)
   {
      double kXY = 1000.0; //1422.0;
      double bXY = 100.0; //150.6;
      double kZ = 20.0; //50.0;
      double bZ = 50.0; //1000.0;
      GroundContactModel groundContactModel = new LinearGroundContactModel(robot, kXY, bXY, kZ, bZ, robot.getRobotsYoVariableRegistry());

      GroundProfile3D groundProfile = new FlatGroundProfile();
      groundContactModel.setGroundProfile3D(groundProfile);
      robot.setGroundContactModel(groundContactModel);

   }

   public static void main(String[] args)
   {
      new SphereSimulation();
   }
}
