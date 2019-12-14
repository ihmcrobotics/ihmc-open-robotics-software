package us.ihmc.commonWalkingControlModules.capturePoint.lqrControl;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.gui.tools.SimulationOverheadPlotterFactory;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

import java.util.ArrayList;
import java.util.List;

public class LQRMomentumControllerSimulation
{
   private final static double desiredHeight = 0.75;
   private final static double controlDT = 0.001;
   private final static double gravity = 9.81;

   private final SimulationConstructionSet scs;
   private final BasicSphereController controller;

   public LQRMomentumControllerSimulation()
   {
      Vector3D initialPosition = new Vector3D(0.0, 0.0, 1.0);
      SphereRobot sphereRobotModel = new SphereRobot();
      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      RobotTools.SCSRobotFromInverseDynamicsRobotModel sphereRobot = SphereRobot
            .createSphereRobot("SphereRobot", initialPosition, sphereRobotModel.getElevator(), yoGraphicsListRegistry, gravity);

      ExternalForcePoint externalForcePoint = sphereRobot.getAllExternalForcePoints().get(0);

      SphereControlToolbox sphereControlToolbox = new SphereControlToolbox(sphereRobotModel.getElevator(), controlDT, desiredHeight, gravity,
                                                                           sphereRobot.getYoTime(), sphereRobotModel.getTotalMass(),
                                                                           sphereRobot.getRobotsYoVariableRegistry(), yoGraphicsListRegistry);
      controller = new BasicSphereController(sphereRobot, sphereControlToolbox, externalForcePoint, yoGraphicsListRegistry);
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

   public void setTrajectories(List<? extends ContactStateProvider> contacts)
   {
      controller.solveForTrajectory(contacts);
   }

   public static void main(String[] args)
   {
      LQRMomentumControllerSimulation simulation = new LQRMomentumControllerSimulation();

      SettableContactStateProvider state1 = new SettableContactStateProvider();
      SettableContactStateProvider state2 = new SettableContactStateProvider();
      SettableContactStateProvider state3 = new SettableContactStateProvider();
      SettableContactStateProvider state4 = new SettableContactStateProvider();
      SettableContactStateProvider state5 = new SettableContactStateProvider();

      FramePoint2D contact1 = new FramePoint2D();
      FramePoint2D contact2 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 0.0, 0.15);
      FramePoint2D contact3 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 1.0, -0.15);
      FramePoint2D contact4 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 2.0, 0.15);
      FramePoint2D contact5 = new FramePoint2D(ReferenceFrame.getWorldFrame(), 2.0, 0.0);

      state1.getTimeInterval().setInterval(0.0, 0.5);
      state2.getTimeInterval().setInterval(0.5, 1.0);
      state3.getTimeInterval().setInterval(1.0, 1.5);
      state4.getTimeInterval().setInterval(1.5, 2.0);
      state5.getTimeInterval().setInterval(2.0, 2.5);

      state1.setStartCopPosition(contact1);
      state1.setEndCopPosition(contact2);

      state2.setStartCopPosition(contact2);
      state2.setEndCopPosition(contact3);

      state3.setStartCopPosition(contact3);
      state3.setEndCopPosition(contact4);

      state4.setStartCopPosition(contact4);
      state4.setEndCopPosition(contact5);

      state5.setStartCopPosition(contact5);
      state5.setEndCopPosition(contact5);

      List<ContactStateProvider> contactStateProviders = new ArrayList<>();
      contactStateProviders.add(state1);
      contactStateProviders.add(state2);
      contactStateProviders.add(state3);
      contactStateProviders.add(state4);
      contactStateProviders.add(state5);

      simulation.setTrajectories(contactStateProviders);
   }
}
