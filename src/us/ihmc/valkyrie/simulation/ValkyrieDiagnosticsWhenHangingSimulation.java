package us.ihmc.valkyrie.simulation;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.virtualHoist.VirtualHoist;
import us.ihmc.valkyrie.HumanoidDiagnosticsWhenHangingSimulation;
import us.ihmc.valkyrie.ValkyrieInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class ValkyrieDiagnosticsWhenHangingSimulation
{
   public ValkyrieDiagnosticsWhenHangingSimulation()
   {
      DRCRobotModel robotModel = new ValkyrieRobotModelWithHoist(false, false);
      double groundZ = 0.0;
      double initialYaw = 0.0;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = new ValkyrieInitialSetup(groundZ, initialYaw);
      
      HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation = new HumanoidDiagnosticsWhenHangingSimulation(ValkyrieConfigurationRoot.VALKYRIE_WITH_ARMS, robotModel, robotInitialSetup);
      humanoidDiagnosticsWhenHangingSimulation.rememberCorruptorVariableValues();
      
      //loadDataAndDoSomeOptimizationTests(humanoidDiagnosticsWhenHangingSimulation);
   }
   
   private void loadDataAndDoSomeOptimizationTests(HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation)
   {
      SimulationConstructionSet simulationConstructionSet = humanoidDiagnosticsWhenHangingSimulation.getSimulationConstructionSet();
      simulationConstructionSet.readData("D:/20141204_091350_Valkyrie_VerySlowArmMotionsWithMorePoses_Processed.data.gz");

      humanoidDiagnosticsWhenHangingSimulation.restoreCorruptorVariableValues();

      //      setInitialCorruptorMassValues(simulationConstructionSet);
      //      setInitialCorruptorCoMOffsetValues(simulationConstructionSet);
      setInitialCorruptorTorqueOffsetValues(simulationConstructionSet);

      //      humanoidDiagnosticsWhenHangingSimulation.setCorruptorVariableValuesToOptimizeToZero();


      String side = "Right";

      // Forearm only:
      String[] containsToOptimizeCoM = new String[]{side + "ForearmCoM"};
      String[] containsToOptimizeTorqueOffset = new String[]{side + "Elbow"};

      //    String[] containsToOptimizeCoM = new String[]{side + "ShoulderRotatorCoM", side + "ShoulderAdductorCoM", side + "ForearmCoM"};
      //    String[] containsToOptimizeTorqueOffset = new String[]{side + "Shoulder", side + "Elbow"};
      humanoidDiagnosticsWhenHangingSimulation.setVariablesToOptimize(containsToOptimizeCoM, containsToOptimizeTorqueOffset);

   }

   private void setInitialCorruptorMassValues(SimulationConstructionSet simulationConstructionSet)
   {
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderExtensorMass")).set(2.65);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderAdductorMass")).set(2.87);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderRotatorMass")).set(2.575);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightElbowExtensorMass")).set(2.367);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightForearmMass")).set(2.903);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightWristYokeMass")).set(0.1);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightPalmMass")).set(0.928);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1LeftShoulderExtensorMass")).set(2.65);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1LeftShoulderAdductorMass")).set(2.87);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1LeftShoulderRotatorMass")).set(2.575);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1LeftElbowExtensorMass")).set(2.367);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1LeftForearmMass")).set(2.903);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1LeftWristYokeMass")).set(0.1);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1LeftPalmMass")).set(0.928);
   }
   
   private void setInitialCorruptorCoMOffsetValues(SimulationConstructionSet simulationConstructionSet)
   {
      // Start from Sylvain's Numbers:
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderAdductorCoMOffsetX")).set(-0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderAdductorCoMOffsetY")).set(0.005);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderAdductorCoMOffsetZ")).set(-0.04);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderRotatorCoMOffsetX")).set(0.004);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderRotatorCoMOffsetY")).set(0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightShoulderRotatorCoMOffsetZ")).set(-0.273);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightElbowExtensorCoMOffsetX")).set(-0.027);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightElbowExtensorCoMOffsetY")).set(0.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightElbowExtensorCoMOffsetZ")).set(-0.08);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightForearmCoMOffsetX")).set(0.015);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightForearmCoMOffsetY")).set(-0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("v1RightForearmCoMOffsetZ")).set(-0.11);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_RightShoulderExtensor")).set(-1.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_RightShoulderAdductor")).set(0.7);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_RightShoulderSupinator")).set(0.3);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_RightElbowExtensor")).set(-0.1);
   }
   
   private void setInitialCorruptorTorqueOffsetValues(SimulationConstructionSet simulationConstructionSet)
   {
      // Start from Sylvain's Numbers:
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_RightShoulderExtensor")).set(-1.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_RightShoulderAdductor")).set(0.7);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_RightShoulderSupinator")).set(0.3);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_RightElbowExtensor")).set(-0.1);
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
