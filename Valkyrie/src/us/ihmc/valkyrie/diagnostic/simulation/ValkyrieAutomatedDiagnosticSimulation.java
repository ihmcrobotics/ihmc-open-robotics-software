package us.ihmc.valkyrie.diagnostic.simulation;

import java.io.InputStream;
import java.util.ArrayList;

import us.ihmc.avatar.diagnostics.AutomatedDiagnosticConfiguration;
import us.ihmc.avatar.diagnostics.AutomatedDiagnosticSimulationFactory;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters.DiagnosticEnvironment;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.virtualHoist.VirtualHoist;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.diagnostic.ValkyrieDiagnosticParameters;

public class ValkyrieAutomatedDiagnosticSimulation
{
   public ValkyrieAutomatedDiagnosticSimulation()
   {
      ValkyrieRobotModelWithHoist robotModel = new ValkyrieRobotModelWithHoist(RobotTarget.SCS, false);
      ValkyrieDiagnosticParameters diagnosticParameters = new ValkyrieDiagnosticParameters(DiagnosticEnvironment.RUNTIME_CONTROLLER, robotModel, false);

      AutomatedDiagnosticSimulationFactory simulationFactory = new AutomatedDiagnosticSimulationFactory(robotModel);

      InputStream gainStream = getClass().getClassLoader().getResourceAsStream("diagnostic/simulationPDGains.yaml");
      InputStream setpointStream = getClass().getClassLoader().getResourceAsStream("diagnostic/diagnosticSetPoints.yaml");

      simulationFactory.setGainStream(gainStream);
      simulationFactory.setSetpointStream(setpointStream);
      simulationFactory.setRobotInitialSetup(0.5, 0.0);
      simulationFactory.setDiagnosticParameters(diagnosticParameters);
      
      AutomatedDiagnosticConfiguration automatedDiagnosticConfiguration = simulationFactory.createDiagnosticController(true);
      automatedDiagnosticConfiguration.addJointCheckUpDiagnostic();
      automatedDiagnosticConfiguration.addPelvisIMUCheckUpDiagnostic();

      simulationFactory.startSimulation();
   }

   private class ValkyrieRobotModelWithHoist extends ValkyrieRobotModel
   {

      public ValkyrieRobotModelWithHoist(DRCRobotModel.RobotTarget target, boolean headless)
      {
         super(target, headless);
      }

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      {
         HumanoidFloatingRootJointRobot robot = super.createHumanoidFloatingRootJointRobot(createCollisionMeshes);

         Joint joint = robot.getJoint("torsoRoll");

         ArrayList<Vector3D> attachmentLocations = new ArrayList<Vector3D>();

         attachmentLocations.add(new Vector3D(0.0, 0.15, 0.412));
         attachmentLocations.add(new Vector3D(0.0, -0.15, 0.412));

         double updateDT = 0.0001;
         VirtualHoist virtualHoist = new VirtualHoist(joint, robot, attachmentLocations, updateDT);
         robot.setController(virtualHoist, 1);

         virtualHoist.turnHoistOn();
         virtualHoist.setTeepeeLocation(new Point3D(0.0, 0.0, 2.5));
         virtualHoist.setHoistStiffness(20000.0);
         virtualHoist.setHoistDamping(15000.0);

         return robot;
      }
   }

   public static void main(String[] args)
   {
      new ValkyrieAutomatedDiagnosticSimulation();
   }
}
