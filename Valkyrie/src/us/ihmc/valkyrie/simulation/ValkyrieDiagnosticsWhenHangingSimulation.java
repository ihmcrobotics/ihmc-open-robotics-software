package us.ihmc.valkyrie.simulation;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.avatar.diagnostics.HumanoidDiagnosticsWhenHangingSimulation;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.virtualHoist.VirtualHoist;
import us.ihmc.valkyrie.ValkyrieInitialSetup;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.valkyrie.configuration.ValkyrieConfigurationRoot;
import us.ihmc.valkyrieRosControl.ValkyrieTorqueOffsetPrinter;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;

public class ValkyrieDiagnosticsWhenHangingSimulation
{
   private final DiagnosticsWhenHangingController diagnosticsWhenHangingController;
   private static final boolean computeTorqueOffsetsBasedOnAverages = true;

   public ValkyrieDiagnosticsWhenHangingSimulation()
   {
      DRCRobotModel robotModel = new ValkyrieRobotModelWithHoist(DRCRobotModel.RobotTarget.SCS, false);
      double groundZ = 0.0;
      double initialYaw = 0.0;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = new ValkyrieInitialSetup(groundZ, initialYaw);
      
      HumanoidJointPoseList humanoidJointPoseList = new HumanoidJointPoseList();
//      humanoidJointPoseList.createPoseSetters();
      humanoidJointPoseList.createPoseSettersJustArms();
//      humanoidJointPoseList.createPoseSettersTuneWaist();
      
      boolean robotIsHanging = true;
      HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation = new HumanoidDiagnosticsWhenHangingSimulation(humanoidJointPoseList, ValkyrieConfigurationRoot.VALKYRIE_WITH_ARMS, robotIsHanging, robotModel, robotInitialSetup, computeTorqueOffsetsBasedOnAverages);
      humanoidDiagnosticsWhenHangingSimulation.rememberCorruptorVariableValues();

      
      diagnosticsWhenHangingController = humanoidDiagnosticsWhenHangingSimulation.getDiagnosticsWhenHangingController();
      SimulationConstructionSet simulationConstructionSet = humanoidDiagnosticsWhenHangingSimulation.getSimulationConstructionSet();
      
//      loadDataAndDoSomeOptimizationTests(humanoidDiagnosticsWhenHangingSimulation);


      String side = "right";
//    String[] containsToOptimizeCoM = new String[]{side + "ShoulderPitchLinkCoM", side + "leftShoulderRollLinkCoM", side + "ShoulderYawLinkCoM", side + "ElbowPitchLinkCoM", side + "ForearmLinkCoM"};
//    String[] containsToOptimizeTorqueScore = new String[]{"ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "ElbowPitch"};

      String[] containsToOptimizeCoM = new String[]{side + "ThighCoM", side + "ShinCoM", side + "FootCoM"};
      String[] containsToOptimizeTorqueScore = new String[]{side + "HipYaw", side + "HipRoll", side + "HipPitch", side + "KneePitch", side + "AnklePitch", side + "AnkleRoll"};

//      String[] containsToOptimizeCoM = new String[]{"pelvisCoM", "leftThighCoM", "leftShinCoM", "leftFootCoM", "rightThighCoM", "rightShinCoM", "rightFootCoM"};
//      String[] containsToOptimizeTorqueScore = new String[]{"torsoYaw", "torsoPitch", "torsoRoll", "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll", "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll"};

      humanoidDiagnosticsWhenHangingSimulation.setVariablesToOptimize(containsToOptimizeCoM, containsToOptimizeTorqueScore);
      
//      humanoidDiagnosticsWhenHangingSimulation.updateDataAndComputeTorqueOffsetsBasedOnAverages(computeTorqueOffsetsBasedOnAverages);
      
      simulationConstructionSet.addButton(new UpdateDataAndComputeTorqueOffsetsBasedOnAveragesButton(humanoidDiagnosticsWhenHangingSimulation));
//      PrintTorqueOffsetsButton printTorqueOffsetsButton = new PrintTorqueOffsetsButton();
//      simulationConstructionSet.addButton(printTorqueOffsetsButton);
   }

   private class UpdateDataAndComputeTorqueOffsetsBasedOnAveragesButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = 262981153765265286L;
      private final HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation;
      
      public UpdateDataAndComputeTorqueOffsetsBasedOnAveragesButton(HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation)
      {
         super("ComputeTorqueOffsets");
         this.humanoidDiagnosticsWhenHangingSimulation = humanoidDiagnosticsWhenHangingSimulation;
         
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
       humanoidDiagnosticsWhenHangingSimulation.updateDataAndComputeTorqueOffsetsBasedOnAverages(computeTorqueOffsetsBasedOnAverages);
      }

   }
   

   private class PrintTorqueOffsetsButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = 262981153765265286L;
      
      public PrintTorqueOffsetsButton()
      {
         super("PrintTorqueOffsets");
         
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         new ValkyrieTorqueOffsetPrinter().printTorqueOffsets(diagnosticsWhenHangingController);
//         printOffsetsForCoeffsForValkyrie();
      }

   }
   
   
   
   // Don't Delete! Fix this for Valkyrie for printing out the coeffs when the torque offsets are changed. 
   // Make it be more general, just looking at the tau_off variables...
   public void printOffsetsForCoeffsForValkyrie()
   {
      java.text.NumberFormat doubleFormat = new java.text.DecimalFormat(" 0.00;-0.00");

      System.out.println();

      ArrayList<OneDoFJoint> oneDoFJoints = diagnosticsWhenHangingController.getOneDoFJoints();
      
      for (OneDoFJoint oneDoFJoint : oneDoFJoints)
      {
         DiagnosticsWhenHangingHelper diagnosticsWhenHangingHelper = diagnosticsWhenHangingController.getDiagnosticsWhenHangingHelper(oneDoFJoint);

         if (diagnosticsWhenHangingHelper != null)
         {
            double torqueOffset = diagnosticsWhenHangingHelper.getTorqueOffset();
            double torqueOffsetSign = diagnosticsWhenHangingController.getTorqueOffsetSign(oneDoFJoint);

            double signedTorqueOffset = torqueOffset * torqueOffsetSign;

            String offsetString = doubleFormat.format(signedTorqueOffset);
            System.out.println(oneDoFJoint.getName() + " torque offset = " + offsetString);
         }
      }
   }

   private void loadDataAndDoSomeOptimizationTests(HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation)
   {
      SimulationConstructionSet simulationConstructionSet = humanoidDiagnosticsWhenHangingSimulation.getSimulationConstructionSet();
      simulationConstructionSet.readData("/home/val/Desktop/diagnosticsWhenHangingRenamed.data.gz.data");

      humanoidDiagnosticsWhenHangingSimulation.restoreCorruptorVariableValues();
//      setInitialCorruptorValues(simulationConstructionSet);


      String side = "left";

      String[] containsToOptimizeCoM = new String[]{side + "ShoulderPitchLinkCoM", side + "leftShoulderRollLinkCoM", side + "ShoulderYawLinkCoM", side + "ElbowPitchLinkCoM", side + "ForearmLinkCoM"};
      String[] containsToOptimizeTorqueScore = new String[]{"ShoulderPitch", "ShoulderRoll", "ShoulderYaw", "ElbowPitch"};

      humanoidDiagnosticsWhenHangingSimulation.setVariablesToOptimize(containsToOptimizeCoM, containsToOptimizeTorqueScore);
   }
  
   private void loadLegDataAndDoSomeOptimizationTests(HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation)
   {
      SimulationConstructionSet simulationConstructionSet = humanoidDiagnosticsWhenHangingSimulation.getSimulationConstructionSet();
//      simulationConstructionSet.readData("D:/20141205_115537_Valkyrie_VerySlowLegMotions_Processed.data.gz");
      simulationConstructionSet.readData("D:/RobotLogData/20141212_155658_Valkyrie_HangingDiagnostics_Renamed.data.gz");

      humanoidDiagnosticsWhenHangingSimulation.restoreCorruptorVariableValues();
      
      String side = ""; //"right";

//      String[] containsToOptimizeCoM = new String[]{side + "ThighCoM", side + "ShinCoM"};
      String[] containsToOptimizeCoM = new String[]{side + "ShinCoM"};
      
      side = ""; //"Right";
//      String[] containsToOptimizeTorqueScore = new String[]{"Waist", side + "Hip", side + "Knee", side + "Ankle"};
      String[] containsToOptimizeTorqueScore = new String[]{side + "Knee"};

      humanoidDiagnosticsWhenHangingSimulation.setVariablesToOptimize(containsToOptimizeCoM, containsToOptimizeTorqueScore);
   }
   
   
   private void loadUpperBodyDataAndDoSomeOptimizationTests(HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation)
   {
      SimulationConstructionSet simulationConstructionSet = humanoidDiagnosticsWhenHangingSimulation.getSimulationConstructionSet();
//      simulationConstructionSet.readData("D:/RobotLogData/20141208_115309_Valkyrie_SlowChestMotionLastSequence/20141208_115309_Valkyrie_SlowChestMotionLastSequence_Processed.data.gz");
      simulationConstructionSet.readData("D:/RobotLogData/20141212_125659_Valkyrie_ChestMotionsDiagnostic_Processed.data.gz");

      
      
      humanoidDiagnosticsWhenHangingSimulation.restoreCorruptorVariableValues();
      
//      setInitialCorruptorLegCoMOffsetValues(simulationConstructionSet);
      
//      String[] containsToOptimizeCoM = new String[]{"chestCoMOffsetX", "chestCoMOffsetZ"}; //, "ShoulderRotatorCoM", "ShoulderAdductorCoM", "ForearmCoM"};
////      String[] containsToOptimizeCoM = new String[]{"chestCoMOffset", "chestMass"}; //, "ShoulderRotatorCoM", "ShoulderAdductorCoM", "ForearmCoM"};
//      String[] containsToOptimizeTorqueScore = new String[]{"Waist"};
      
      String[] containsToOptimizeCoM = new String[]{"chestCoMOffset", "ShoulderRotatorCoM", "ShoulderAdductorCoM", "ForearmCoM"};
    String[] containsToOptimizeTorqueScore = new String[]{"Waist", "Shoulder", "Elbow"};

      humanoidDiagnosticsWhenHangingSimulation.setVariablesToOptimize(containsToOptimizeCoM, containsToOptimizeTorqueScore);
   }

   private void setInitialCorruptorArmMassValues(SimulationConstructionSet simulationConstructionSet)
   {
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderPitchMass")).set(2.65);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderRollMass")).set(2.87);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderYawMass")).set(2.575);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightElbowPitchMass")).set(2.367);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightForearmYawMass")).set(2.903);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightWristPitchMass")).set(0.1);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightWristRollMass")).set(0.928);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShoulderPitchMass")).set(2.65);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShoulderRollMass")).set(2.87);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShoulderYawMass")).set(2.575);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftElbowPitchMass")).set(2.367);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftForearmYawMass")).set(2.903);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftWristPitchMass")).set(0.1);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftWristRollMass")).set(0.928);
   }
   
   private void setInitialCorruptorArmCoMOffsetValues(SimulationConstructionSet simulationConstructionSet)
   {
      // Start from Sylvain's Numbers:
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderRollCoMOffsetX")).set(-0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderRollCoMOffsetY")).set(0.005);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderRollCoMOffsetZ")).set(-0.04);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderYawCoMOffsetX")).set(0.004);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderYawCoMOffsetY")).set(0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShoulderYawCoMOffsetZ")).set(-0.273);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightElbowPitchCoMOffsetX")).set(-0.027);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightElbowPitchCoMOffsetY")).set(0.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightElbowPitchCoMOffsetZ")).set(-0.08);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightForearmYawCoMOffsetX")).set(0.015);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightForearmYawCoMOffsetY")).set(-0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightForearmYawCoMOffsetZ")).set(-0.11);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_rightShoulderPitxh")).set(-1.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_rightShoulderRoll")).set(0.7);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_rightShoulderYaw")).set(0.3);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_rightElbowPitch")).set(-0.1);
   }
   
   private void setInitialCorruptorArmTorqueOffsetValues(SimulationConstructionSet simulationConstructionSet)
   {
      // Start from Sylvain's Numbers:
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_rightShoulderPitch")).set(-1.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_rightShoulderRoll")).set(0.7);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_rightShoulderYaw")).set(0.3);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("tau_off_rightElbowPitch")).set(-0.1);
   }
   
   private void setInitialCorruptorLegCoMOffsetValues(SimulationConstructionSet simulationConstructionSet)
   {
      // Tuned by Jerry on December 8, 2014.
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftThighCoMOffsetX")).set(-0.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftThighCoMOffsetY")).set(0.05);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftThighCoMOffsetZ")).set(-0.220);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShinCoMOffsetX")).set(0.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShinCoMOffsetY")).set(0.045);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShinCoMOffsetZ")).set(-0.185);
      
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightThighCoMOffsetX")).set(0.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightThighCoMOffsetY")).set(-0.05);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightThighCoMOffsetZ")).set(-0.220);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShinCoMOffsetX")).set(0.0);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShinCoMOffsetY")).set(-0.045);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShinCoMOffsetZ")).set(-0.185);
      

   }

   public static void main(String[] args)
   {
      new ValkyrieDiagnosticsWhenHangingSimulation();
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
