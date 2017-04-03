package us.ihmc.steppr.simulation;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;

import us.ihmc.avatar.diagnostics.HumanoidDiagnosticsWhenHangingSimulation;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.DiagnosticsWhenHangingHelper;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;
import us.ihmc.simulationConstructionSetTools.util.virtualHoist.VirtualHoist;
import us.ihmc.steppr.parameters.BonoRobotModel;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;

public class StepprDiagnosticsWhenHangingSimulation
{
   private final DiagnosticsWhenHangingController diagnosticsWhenHangingController;
   private static final boolean computeTorqueOffsetsBasedOnAverages = false;

   public StepprDiagnosticsWhenHangingSimulation()
   {
      BonoRobotModelWithHoist robotModel = new BonoRobotModelWithHoist(false, false);

      final double groundHeight = 0.0;
      GroundProfile3D groundProfile = new FlatGroundProfile(groundHeight);

      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(groundProfile, robotModel.getSimulateDT());
      scsInitialSetup.setDrawGroundProfile(true);
      scsInitialSetup.setInitializeEstimatorToActual(true);

      double initialYaw = 0.3;
      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup = robotModel.getDefaultRobotInitialSetup(groundHeight, initialYaw);

      HumanoidJointPoseList humanoidJointPoseList = new HumanoidJointPoseList();
      humanoidJointPoseList.createPoseSettersJustLegs();
      
      boolean robotIsHanging = true;
      boolean hasArms = false;
      HumanoidDiagnosticsWhenHangingSimulation humanoidDiagnosticsWhenHangingSimulation = new HumanoidDiagnosticsWhenHangingSimulation(humanoidJointPoseList, hasArms, robotIsHanging, robotModel, robotInitialSetup, computeTorqueOffsetsBasedOnAverages);
      humanoidDiagnosticsWhenHangingSimulation.rememberCorruptorVariableValues();

      
      diagnosticsWhenHangingController = humanoidDiagnosticsWhenHangingSimulation.getDiagnosticsWhenHangingController();
      SimulationConstructionSet simulationConstructionSet = humanoidDiagnosticsWhenHangingSimulation.getSimulationConstructionSet();
      
//      loadDataAndDoSomeOptimizationTests(humanoidDiagnosticsWhenHangingSimulation);
      
      humanoidDiagnosticsWhenHangingSimulation.updateDataAndComputeTorqueOffsetsBasedOnAverages(computeTorqueOffsetsBasedOnAverages);
      
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
//      simulationConstructionSet.readData("D:/RobotLogData/20150108_212041_StepprDiagAfterKtScaling_Processed.data.gz");
//      simulationConstructionSet.readData("D:/RobotLogData/20150109_120549_StepprDiagSplineTime10s_Processed.data.gz");
      simulationConstructionSet.readData("D:/RobotLogData/20150116_150646_Steppr_Diagnosis_Processed.data.gz");

      humanoidDiagnosticsWhenHangingSimulation.restoreCorruptorVariableValues();
//      setInitialCorruptorValues(simulationConstructionSet);


      //      humanoidDiagnosticsWhenHangingSimulation.setCorruptorVariableValuesToOptimizeToZero();

      String side = "left";

      // Forearm only:
//      String[] containsToOptimizeCoM = new String[]{side + "ShinCoM"};
      String[] containsToOptimizeCoM = new String[]{side + "ThighCoM", side + "ShinCoM", side + "FootCoM"};
//      String[] containsToOptimizeCoM = new String[]{side + "ThighCoM", side + "ShinCoM"};
      String[] containsToOptimizeTorqueScore = new String[]{"leg_uhz", "leg_lhy", "leg_mhx", "leg_kny", "leg_uay", "leg_lax"};
//      String[] containsToOptimizeTorqueScore = new String[]{"kny"};
//      String[] containsToOptimizeTorqueScore = new String[]{"uay", "lax"};

      humanoidDiagnosticsWhenHangingSimulation.setVariablesToOptimize(containsToOptimizeCoM, containsToOptimizeTorqueScore);
   }
  
   

   private void setInitialCorruptorValues(SimulationConstructionSet simulationConstructionSet)
   {
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftThighCoMOffsetX")).set(-0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftThighCoMOffsetY")).set(0.0525);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftThighCoMOffsetZ")).set(-0.218); 
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShinCoMOffsetX")).set(0.013); 
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShinCoMOffsetY")).set(-0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftShinCoMOffsetZ")).set(-0.06); 
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftFootCoMOffsetX")).set(0.121);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftFootCoMOffsetY")).set(0.0); 
      ((DoubleYoVariable) simulationConstructionSet.getVariable("leftFootCoMOffsetZ")).set(0.0);
      
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightThighCoMOffsetX")).set(-0.02); 
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightThighCoMOffsetY")).set(-0.0525);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightThighCoMOffsetZ")).set(-0.218);
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShinCoMOffsetX")).set(0.013);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShinCoMOffsetY")).set(0.02);
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightShinCoMOffsetZ")).set(-0.06); 
      
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightFootCoMOffsetX")).set(0.121); 
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightFootCoMOffsetY")).set(0.0); 
      ((DoubleYoVariable) simulationConstructionSet.getVariable("rightFootCoMOffsetZ")).set(0.0); 
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
      public HumanoidFloatingRootJointRobot createHumanoidFloatingRootJointRobot(boolean createCollisionMeshes)
      {
         HumanoidFloatingRootJointRobot robot = super.createHumanoidFloatingRootJointRobot(createCollisionMeshes);
         
         Joint joint = robot.getJoint("back_ubz");
         if (joint == null) throw new RuntimeException();
         
         ArrayList<Vector3D> attachmentLocations = new ArrayList<Vector3D>();
             
         attachmentLocations.add(new Vector3D(0.0, 0.15, 0.412));
         attachmentLocations.add(new Vector3D(0.0, -0.15, 0.412));
         
         double updateDT = 0.0001;
         VirtualHoist virtualHoist = new VirtualHoist(joint, robot, attachmentLocations, updateDT);
         robot.setController(virtualHoist, 1);
          
         virtualHoist.turnHoistOn();
         virtualHoist.setTeepeeLocation(new Point3D(0.0, 0.0, 2.5));
         virtualHoist.setHoistStiffness(20000.0);
         virtualHoist.setHoistDamping(5000.0);
         
         return robot;
      }

   }
}
