package us.ihmc.atlas.diagnostic;

import java.util.ArrayList;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.diagnostics.AutomatedDiagnosticConfiguration;
import us.ihmc.avatar.diagnostics.AutomatedDiagnosticSimulationFactory;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.virtualHoist.VirtualHoist;
import us.ihmc.simulationconstructionset.Joint;

public class AtlasAutomatedDiagnosticSimulation
{
   public AtlasAutomatedDiagnosticSimulation()
   {
      AtlasRobotModelWithHoist robotModel = new AtlasRobotModelWithHoist(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);

      AutomatedDiagnosticSimulationFactory simulationFactory = new AutomatedDiagnosticSimulationFactory(robotModel);

      simulationFactory.setRobotInitialSetup(0.2, 0.0);

      AutomatedDiagnosticConfiguration automatedDiagnosticConfiguration = simulationFactory.createDiagnosticController(true);
      automatedDiagnosticConfiguration.addJointCheckUps(AtlasDiagnosticParameters.defaultJointCheckUpConfiguration(robotModel.getJointMap()));
      automatedDiagnosticConfiguration.addPelvisIMUCheckUpDiagnostic(AtlasDiagnosticParameters.defaultPelvisIMUCheckUp(robotModel.getSensorInformation(),
                                                                                                                       robotModel.getJointMap()));

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

         ArrayList<Vector3D> attachmentLocations = new ArrayList<Vector3D>();

         attachmentLocations.add(new Vector3D(0.05, 0.15, 0.80));
         attachmentLocations.add(new Vector3D(0.05, -0.15, 0.80));

         double updateDT = getSimulateDT();
         VirtualHoist virtualHoist = new VirtualHoist(joint, robot, attachmentLocations, updateDT);
         robot.setController(virtualHoist, 1);

         virtualHoist.turnHoistOn();
         virtualHoist.setTeepeeLocation(new Point3D(0.0, 0.0, 2.6));
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
