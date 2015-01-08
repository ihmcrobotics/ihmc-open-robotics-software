package us.ihmc.acsell.simulation;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.acsell.parameters.BonoRobotModel;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.diagnostics.HumanoidDiagnosticsWhenHangingSimulation;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationconstructionset.util.virtualHoist.VirtualHoist;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class StepprDiagnosticsWhenHangingSimulation
{
   private final DiagnosticsWhenHangingController diagnosticsWhenHangingController;
   
   public StepprDiagnosticsWhenHangingSimulation()
   {
      BonoRobotModelWithHoist robotModel = new BonoRobotModelWithHoist(false, false);

      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<SDFRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      double kneeAngleMultiplicationFactor = -1.0;
      HumanoidJointPoseList humanoidJointPoseList = new HumanoidJointPoseList(kneeAngleMultiplicationFactor);
      humanoidJointPoseList.createPoseSettersJustLegs();
      
      boolean robotIsHanging = true;
      boolean hasArms = false;
      HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation = new HumanoidDiagnosticsWhenHangingSimulation(humanoidJointPoseList, hasArms, robotIsHanging, robotModel, robotInitialSetup);
      humanoidDiagnosticsWhenHangingSimulation.rememberCorruptorVariableValues();

      
      diagnosticsWhenHangingController = humanoidDiagnosticsWhenHangingSimulation.getDiagnosticsWhenHangingController();
      SimulationConstructionSet simulationConstructionSet = humanoidDiagnosticsWhenHangingSimulation.getSimulationConstructionSet();
      
//      loadUpperBodyDataAndDoSomeOptimizationTests(humanoidDiagnosticsWhenHangingSimulation);
      //loadArmDataAndDoSomeOptimizationTests(humanoidDiagnosticsWhenHangingSimulation);
//      loadLegDataAndDoSomeOptimizationTests(humanoidDiagnosticsWhenHangingSimulation);
      
      
      humanoidDiagnosticsWhenHangingSimulation.updateDataAndComputeTorqueOffsetsBasedOnAverages();
      
      PrintTorqueOffsetsButton printTorqueOffsetsButton = new PrintTorqueOffsetsButton();
      simulationConstructionSet.addButton(printTorqueOffsetsButton);
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
         printOffsetsForCoeffsForSteppr();
      }

   }
   
   
   public void printOffsetsForCoeffsForSteppr()
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
      simulationConstructionSet.readData("D:/20141204_091350_Valkyrie_VerySlowArmMotionsWithMorePoses_Processed.data.gz");

      humanoidDiagnosticsWhenHangingSimulation.restoreCorruptorVariableValues();

      //      setInitialCorruptorArmMassValues(simulationConstructionSet);
      //      setInitialCorruptorArmCoMOffsetValues(simulationConstructionSet);
//      setInitialCorruptorArmTorqueOffsetValues(simulationConstructionSet);

      //      humanoidDiagnosticsWhenHangingSimulation.setCorruptorVariableValuesToOptimizeToZero();


      String side = "Right";

      // Forearm only:
      String[] containsToOptimizeCoM = new String[]{side + "ForearmCoM"};
      String[] containsToOptimizeTorqueScore = new String[]{""};

      //    String[] containsToOptimizeCoM = new String[]{side + "ShoulderRotatorCoM", side + "ShoulderAdductorCoM", side + "ForearmCoM"};
      //    String[] containsToOptimizeTorqueOffset = new String[]{side + "Shoulder", side + "Elbow"};
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
   

   private void setInitialCorruptorArmMassValues(SimulationConstructionSet simulationConstructionSet)
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
   


   public static void main(String[] args)
   {
      new StepprDiagnosticsWhenHangingSimulation();
   }
   
   
   private class BonoRobotModelWithHoist extends BonoRobotModel
   {

      public BonoRobotModelWithHoist(boolean runningOnRealRobot, boolean headless)
      {
         super(runningOnRealRobot, headless);
      }
      
      @Override
      public SDFRobot createSdfRobot(boolean createCollisionMeshes)
      {
         SDFRobot robot = super.createSdfRobot(createCollisionMeshes);
         
         Joint joint = robot.getJoint("back_ubz");
         if (joint == null) throw new RuntimeException();
         
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
