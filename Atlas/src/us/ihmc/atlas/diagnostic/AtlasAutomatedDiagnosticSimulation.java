package us.ihmc.atlas.diagnostic;

import java.io.InputStream;
import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.diagnostics.AutomatedDiagnosticConfiguration;
import us.ihmc.avatar.diagnostics.AutomatedDiagnosticSimulationFactory;
import us.ihmc.avatar.drcRobot.DRCRobotModel.RobotTarget;
import us.ihmc.sensorProcessing.diagnostic.DiagnosticParameters.DiagnosticEnvironment;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.virtualHoist.VirtualHoist;

public class AtlasAutomatedDiagnosticSimulation
{
   public AtlasAutomatedDiagnosticSimulation()
   {
      AtlasRobotModelWithHoist robotModel = new AtlasRobotModelWithHoist(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      AtlasDiagnosticParameters diagnosticParameters = new AtlasDiagnosticParameters(DiagnosticEnvironment.RUNTIME_CONTROLLER, robotModel, false);

      AutomatedDiagnosticSimulationFactory simulationFactory = new AutomatedDiagnosticSimulationFactory(robotModel);

      InputStream gainStream = getClass().getClassLoader().getResourceAsStream("diagnostic/simulationPDGains.yaml");
      InputStream setpointStream = getClass().getClassLoader().getResourceAsStream("diagnostic/diagnosticSetPoints.yaml");

      simulationFactory.setGainStream(gainStream);
      simulationFactory.setSetpointStream(setpointStream);
      simulationFactory.setRobotInitialSetup(0.2, 0.0);
      simulationFactory.setDiagnosticParameters(diagnosticParameters);
      
      AutomatedDiagnosticConfiguration automatedDiagnosticConfiguration = simulationFactory.createDiagnosticController(true);
      automatedDiagnosticConfiguration.addJointCheckUpDiagnostic();
      automatedDiagnosticConfiguration.addPelvisIMUCheckUpDiagnostic();

      simulationFactory.startSimulation();
   }

   private class AtlasRobotModelWithHoist extends AtlasRobotModel
   {

      public AtlasRobotModelWithHoist(AtlasRobotVersion atlasVersion, RobotTarget target, boolean headless)
      {
         super(atlasVersion, target, headless);
      }

      @Override
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      {
         HumanoidFloatingRootJointRobot robot = super.createHumanoidFloatingRootJointRobot(createCollisionMeshes);

         Joint joint = robot.getJoint("back_bkx");

         ArrayList<Vector3d> attachmentLocations = new ArrayList<Vector3d>();

         attachmentLocations.add(new Vector3d(0.05, 0.15, 0.80));
         attachmentLocations.add(new Vector3d(0.05, -0.15, 0.80));

         double updateDT = getSimulateDT();
         VirtualHoist virtualHoist = new VirtualHoist(joint, robot, attachmentLocations, updateDT);
         robot.setController(virtualHoist, 1);

         virtualHoist.turnHoistOn();
         virtualHoist.setTeepeeLocation(new Point3d(0.0, 0.0, 2.6));
         virtualHoist.setHoistStiffness(20000.0);
         virtualHoist.setHoistDamping(15000.0);

         return robot;
      }
   }

   public static void main(String[] args)
   {
      new AtlasAutomatedDiagnosticSimulation();
   }
}
