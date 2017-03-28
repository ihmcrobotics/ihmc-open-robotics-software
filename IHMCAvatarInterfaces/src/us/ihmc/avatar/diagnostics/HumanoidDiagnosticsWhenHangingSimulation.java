package us.ihmc.avatar.diagnostics;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.SwingWorker;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.factory.AvatarSimulation;
import us.ihmc.avatar.factory.AvatarSimulationFactory;
import us.ihmc.avatar.initialSetup.DRCGuiInitialSetup;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.initialSetup.DRCSCSInitialSetup;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.commonWalkingControlModules.instantaneousCapturePoint.icpOptimization.ICPOptimizationParameters;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelState;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.sensorProcessing.parameters.DRCRobotSensorInformation;
import us.ihmc.simulationconstructionset.DataProcessingFunction;
import us.ihmc.simulationconstructionset.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController.DiagnosticsWhenHangingState;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingControllerFactory;
import us.ihmc.wholeBodyController.diagnostics.HumanoidDiagnosticsWhenHangingAnalyzer;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;

public class HumanoidDiagnosticsWhenHangingSimulation
{
   private final SimulationConstructionSet simulationConstructionSet;
   private final HumanoidDiagnosticsWhenHangingAnalyzer analyzer;
   private final DiagnosticsWhenHangingController controller;
   private final boolean computeTorqueOffsetsBasedOnAverages;

   public HumanoidDiagnosticsWhenHangingSimulation(HumanoidJointPoseList humanoidJointPoseList, boolean useArms, boolean robotIsHanging, DRCRobotModel model, DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> robotInitialSetup, boolean computeTorqueOffsetsBasedOnAverages)
   {
      this.computeTorqueOffsetsBasedOnAverages = computeTorqueOffsetsBasedOnAverages;

      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(environment, model.getSimulateDT());
      scsInitialSetup.setRunMultiThreaded(false);

      scsInitialSetup.setInitializeEstimatorToActual(true);
      robotInitialSetup.setInitialGroundHeight(0.0);

      ContactableBodiesFactory contactableBodiesFactory = model.getContactPointParameters().getContactableBodiesFactory();
      DRCRobotSensorInformation sensorInformation = model.getSensorInformation();
      SideDependentList<String> footSensorNames = sensorInformation.getFeetForceSensorNames();
      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = model.getCapturePointPlannerParameters();
      ICPOptimizationParameters icpOptimizationParameters = model.getICPOptimizationParameters();
      SideDependentList<String> feetContactSensorNames = sensorInformation.getFeetContactSensorNames();
      SideDependentList<String> wristForceSensorNames = sensorInformation.getWristForceSensorNames();
      MomentumBasedControllerFactory momentumBasedControllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, footSensorNames,
            feetContactSensorNames, wristForceSensorNames, walkingControllerParameters, capturePointPlannerParameters, HighLevelState.DO_NOTHING_BEHAVIOR);
      DiagnosticsWhenHangingControllerFactory diagnosticsWhenHangingControllerFactory = new DiagnosticsWhenHangingControllerFactory(humanoidJointPoseList, useArms, robotIsHanging, null);
      diagnosticsWhenHangingControllerFactory.setTransitionRequested(true);
      momentumBasedControllerFactory.addHighLevelBehaviorFactory(diagnosticsWhenHangingControllerFactory);
      momentumBasedControllerFactory.setICPOptimizationControllerParameters(icpOptimizationParameters);

      HumanoidGlobalDataProducer globalDataProducer = null;

      AvatarSimulationFactory avatarSimulationFactory = new AvatarSimulationFactory();
      avatarSimulationFactory.setRobotModel(model);
      avatarSimulationFactory.setMomentumBasedControllerFactory(momentumBasedControllerFactory);
      avatarSimulationFactory.setCommonAvatarEnvironment(environment);
      avatarSimulationFactory.setRobotInitialSetup(robotInitialSetup);
      avatarSimulationFactory.setSCSInitialSetup(scsInitialSetup);
      avatarSimulationFactory.setGuiInitialSetup(guiInitialSetup);
      avatarSimulationFactory.setHumanoidGlobalDataProducer(globalDataProducer);
      AvatarSimulation avatarSimulation = avatarSimulationFactory.createAvatarSimulation();

      simulationConstructionSet = avatarSimulation.getSimulationConstructionSet();

      avatarSimulation.start();
//      drcSimulation.simulate();

//      if (DRCSimulationFactory.RUN_MULTI_THREADED)
//      {
//         throw new RuntimeException("This only works with single threaded right now. Change DRCSimulationFactory.RUN_MULTI_THREADED to false!");
//      }

      FullRobotModelCorruptor fullRobotModelCorruptor = avatarSimulation.getFullRobotModelCorruptor();
      if(fullRobotModelCorruptor == null)
    	  throw new RuntimeException("This only works with model corruption on. Change DRCControllerThread.ALLOW_MODEL_CORRUPTION to true!");

      controller = diagnosticsWhenHangingControllerFactory.getController();
      analyzer = new HumanoidDiagnosticsWhenHangingAnalyzer(simulationConstructionSet, controller, fullRobotModelCorruptor);


      UpdateDiagnosticsWhenHangingHelpersButton updateDiagnosticsWhenHangingHelpersButton = new UpdateDiagnosticsWhenHangingHelpersButton(analyzer);
      simulationConstructionSet.addButton(updateDiagnosticsWhenHangingHelpersButton);

      OptimizeDiagnosticsWhenHangingHelpersButton optimizeDiagnosticsWhenHangingHelpersButton = new OptimizeDiagnosticsWhenHangingHelpersButton(analyzer);
      simulationConstructionSet.addButton(optimizeDiagnosticsWhenHangingHelpersButton);

      CutBufferToDiagnosticsStateButton cutBufferButton = new CutBufferToDiagnosticsStateButton(simulationConstructionSet);
      simulationConstructionSet.addButton(cutBufferButton);

      CopyMeasuredTorqueToAppliedTorqueButton copyMeasuredTorqueToAppliedTorqueButton = new CopyMeasuredTorqueToAppliedTorqueButton(analyzer);
      simulationConstructionSet.addButton(copyMeasuredTorqueToAppliedTorqueButton);


      analyzer.printOutAllCorruptorVariables();
   }

   public SimulationConstructionSet getSimulationConstructionSet()
   {
      return simulationConstructionSet;
   }

   public DiagnosticsWhenHangingController getDiagnosticsWhenHangingController()
   {
      return controller;
   }

   public void updateDataAndComputeTorqueOffsetsBasedOnAverages(boolean computeTorqueOffsetsBasedOnAverages)
   {
      analyzer.updateDataAndComputeTorqueOffsetsBasedOnAverages(computeTorqueOffsetsBasedOnAverages);
   }

   public void setVariablesToOptimize(String[] containsToOptimizeCoM, String[] containsToOptimizeTorqueScores)
   {
      analyzer.setVariablesToOptimize(containsToOptimizeCoM, containsToOptimizeTorqueScores);
   }

   public void rememberCorruptorVariableValues()
   {
      analyzer.rememberCorruptorVariableValues();
   }

   public void restoreCorruptorVariableValues()
   {
      analyzer.restoreCorruptorVariableValues();
   }

   public void setCorruptorVariableValuesToOptimizeToZero()
   {
      analyzer.setCorruptorVariableValuesToOptimizeToZero();
   }

   private class CutBufferToDiagnosticsStateButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -2047087705497963648L;
      private final SimulationConstructionSet simulationConstructionSet;
      private final EnumYoVariable<DiagnosticsWhenHangingState> diagnosticsState;

      public CutBufferToDiagnosticsStateButton(SimulationConstructionSet simulationConstructionSet)
      {
         super("Cut Buffer");

         this.simulationConstructionSet = simulationConstructionSet;

         diagnosticsState = (EnumYoVariable<DiagnosticsWhenHangingState>) simulationConstructionSet.getVariable("DiagnosticsState");
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         simulationConstructionSet.cropBuffer();

         while(true)
         {
            simulationConstructionSet.gotoInPointNow();

            while(true)
            {
               if (diagnosticsState.getEnumValue() != DiagnosticsWhenHangingState.CHECK_DIAGNOSTICS)
               {
                  simulationConstructionSet.setInPoint();
                  break;
               }
               if (simulationConstructionSet.getIndex() == simulationConstructionSet.getOutPoint())
               {
                  return;
               }

               simulationConstructionSet.tick(1);
            }

            while(true)
            {
               if (diagnosticsState.getEnumValue() == DiagnosticsWhenHangingState.CHECK_DIAGNOSTICS)
               {
                  simulationConstructionSet.setOutPoint();
                  simulationConstructionSet.cutBuffer();
                  break;
               }
               if (simulationConstructionSet.getIndex() == simulationConstructionSet.getOutPoint())
               {
                  simulationConstructionSet.cutBuffer();
                  break;
               }

               simulationConstructionSet.tick(1);
            }

            simulationConstructionSet.setInOutPointFullBuffer();
         }
      }

   }

   private class CopyMeasuredTorqueToAppliedTorqueButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -8720505122426008775L;
      private final HumanoidDiagnosticsWhenHangingAnalyzer analyzer;

      public CopyMeasuredTorqueToAppliedTorqueButton(HumanoidDiagnosticsWhenHangingAnalyzer analyzer)
      {
         super("CopyMeasuredTorque");
         this.analyzer = analyzer;
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         DataProcessingFunction dataProcessingFunction = new DataProcessingFunction()
         {
            @Override
            public void processData()
            {
               analyzer.copyMeasuredTorqueToAppliedTorque();
            }

            @Override
            public void initializeProcessing()
            {
            }
         };

         simulationConstructionSet.applyDataProcessingFunction(dataProcessingFunction);
      }
   }

   private class UpdateDiagnosticsWhenHangingHelpersButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -112620995090732618L;

      private final HumanoidDiagnosticsWhenHangingAnalyzer analyzer;

      public UpdateDiagnosticsWhenHangingHelpersButton(HumanoidDiagnosticsWhenHangingAnalyzer analyzer)
      {
         super("UpdateDiagnostics");
         this.analyzer = analyzer;
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
//         analyzer.updateCorruptorAndAnalyzeDataInBuffer();
         analyzer.updateDataAndComputeTorqueOffsetsBasedOnAverages(computeTorqueOffsetsBasedOnAverages);
      }
   }

   private class OptimizeDiagnosticsWhenHangingHelpersButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = -112620995090732618L;

      private final HumanoidDiagnosticsWhenHangingAnalyzer analyzer;

      public OptimizeDiagnosticsWhenHangingHelpersButton(HumanoidDiagnosticsWhenHangingAnalyzer analyzer)
      {
         super("Optimize");
         this.analyzer = analyzer;
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         if (this.getText().equals("Optimize"))
         {
            this.setText("StopOptimize");

            SwingWorker worker = new SwingWorker()
            {
               @Override
               protected Object doInBackground() throws Exception
               {
                  analyzer.optimizeCorruptorValues(computeTorqueOffsetsBasedOnAverages);
                  setText("Optimize");
                  return null;
               }};

               worker.execute();
         }
         else
         {
            analyzer.stopOptimization();
         }
      }
   }


}

