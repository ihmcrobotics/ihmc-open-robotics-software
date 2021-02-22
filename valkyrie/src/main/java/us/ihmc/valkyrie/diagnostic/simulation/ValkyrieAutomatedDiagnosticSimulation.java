package us.ihmc.valkyrie.diagnostic.simulation;

import java.util.ArrayList;

import us.ihmc.avatar.diagnostics.AutomatedDiagnosticSimulationFactory;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.virtualHoist.VirtualHoist;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.diagnostic.ValkyrieDiagnosticParameters;
import us.ihmc.wholeBodyController.diagnostics.AutomatedDiagnosticConfiguration;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticParameters;

public class ValkyrieAutomatedDiagnosticSimulation
{
   public ValkyrieAutomatedDiagnosticSimulation()
   {
      ValkyrieRobotModelWithHoist robotModel = new ValkyrieRobotModelWithHoist(RobotTarget.SCS);

      AutomatedDiagnosticSimulationFactory simulationFactory = new AutomatedDiagnosticSimulationFactory(robotModel);
      simulationFactory.setRobotInitialSetup(0.5, 0.0);

      AutomatedDiagnosticConfiguration automatedDiagnosticConfiguration = simulationFactory.createDiagnosticController(true);
      automatedDiagnosticConfiguration.addJointCheckUps(ValkyrieDiagnosticParameters.defaultJointCheckUpConfiguration(robotModel.getJointMap()));
      automatedDiagnosticConfiguration.addPelvisIMUCheckUpDiagnostic(DiagnosticParameters.defaultPelvisIMUCheckUp(robotModel.getSensorInformation(),
                                                                                                                  robotModel.getJointMap()));

      simulationFactory.startSimulation();
   }

   private class ValkyrieRobotModelWithHoist extends ValkyrieRobotModel
   {

      public ValkyrieRobotModelWithHoist(RobotTarget target)
      {
         super(target);
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
