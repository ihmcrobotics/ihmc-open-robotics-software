package us.ihmc.valkyrie.simulation;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.valkyrie.HumanoidDiagnosticsWhenHangingSimulation;
import us.ihmc.valkyrie.ValkyrieInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;

import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.util.virtualHoist.VirtualHoist;

public class ValkyrieDiagnosticsWhenHangingSimulation
{
   public ValkyrieDiagnosticsWhenHangingSimulation()
   {
      DRCRobotModel robotModel = new ValkyrieRobotModelWithHoist(false, false);
      double groundZ = 0.0;
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = new ValkyrieInitialSetup(groundZ, initialYaw);
      
      new HumanoidDiagnosticsWhenHangingSimulation(ValkyrieConfigurationRoot.VALKYRIE_WITH_ARMS, robotModel, robotInitialSetup);
   }

   public static void main(String[] args)
   {
      new ValkyrieDiagnosticsWhenHangingSimulation();
   }
   
   
   private class ValkyrieRobotModelWithHoist extends ValkyrieRobotModel
   {

      public ValkyrieRobotModelWithHoist(boolean runningOnRealRobot, boolean headless)
      {
         super(runningOnRealRobot, headless);
      }
      
      @Override
      public SDFRobot createSdfRobot(boolean createCollisionMeshes)
      {
         SDFRobot robot = super.createSdfRobot(createCollisionMeshes);
         
         Joint joint = robot.getJoint("WaistLateralExtensor");
         
         ArrayList<Vector3d> attachmentLocations = new ArrayList<Vector3d>();
             
         attachmentLocations.add(new Vector3d(0.0, 0.15, 0.412));
         attachmentLocations.add(new Vector3d(0.0, -0.15, 0.412));
         
         double updateDT = 0.0001;
         VirtualHoist virtualHoist = new VirtualHoist(joint, robot, attachmentLocations, updateDT);
         robot.setController(virtualHoist, 1);
          
         virtualHoist.turnHoistOn();
         virtualHoist.setTeepeeLocation(new Point3d(0.0, 0.0, 2.5));
         virtualHoist.setHoistStiffness(20000.0);
         virtualHoist.setHoistDamping(5000.0);
         
         return robot;
      }

   }
}
