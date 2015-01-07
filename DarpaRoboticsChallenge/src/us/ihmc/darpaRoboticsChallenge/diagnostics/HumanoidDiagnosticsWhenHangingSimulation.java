package us.ihmc.darpaRoboticsChallenge.diagnostics;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.SwingWorker;

import us.ihmc.SdfLoader.SDFRobot;
import us.ihmc.commonWalkingControlModules.configurations.ArmControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.CapturePointPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.corruptors.FullRobotModelCorruptor;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.MomentumBasedControllerFactory;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.communication.streamingData.GlobalDataProducer;
import us.ihmc.darpaRoboticsChallenge.DRCGuiInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSCSInitialSetup;
import us.ihmc.darpaRoboticsChallenge.DRCSimulationFactory;
import us.ihmc.darpaRoboticsChallenge.drcRobot.DRCRobotModel;
import us.ihmc.darpaRoboticsChallenge.drcRobot.FlatGroundEnvironment;
import us.ihmc.darpaRoboticsChallenge.initialSetup.DRCRobotInitialSetup;
import us.ihmc.simulationconstructionset.DataProcessingFunction;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.utilities.robotSide.SideDependentList;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingControllerFactory;
import us.ihmc.wholeBodyController.diagnostics.HumanoidDiagnosticsWhenHangingAnalyzer;
import us.ihmc.wholeBodyController.diagnostics.HumanoidJointPoseList;
import us.ihmc.wholeBodyController.diagnostics.DiagnosticsWhenHangingController.DiagnosticsWhenHangingState;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;

public class HumanoidDiagnosticsWhenHangingSimulation
{
   private final SimulationConstructionSet simulationConstructionSet;
   private final HumanoidDiagnosticsWhenHangingAnalyzer analyzer;
   private final DiagnosticsWhenHangingController controller;
   
   public HumanoidDiagnosticsWhenHangingSimulation(HumanoidJointPoseList humanoidJointPoseList, boolean useArms, boolean robotIsHanging, DRCRobotModel model, DRCRobotInitialSetup<SDFRobot> robotInitialSetup)
   {
      FlatGroundEnvironment environment = new FlatGroundEnvironment();
      DRCGuiInitialSetup guiInitialSetup = new DRCGuiInitialSetup(false, false);
      DRCSCSInitialSetup scsInitialSetup = new DRCSCSInitialSetup(environment, model.getSimulateDT());
      scsInitialSetup.setInitializeEstimatorToActual(true);
      robotInitialSetup.setInitialGroundHeight(0.0);

      ContactableBodiesFactory contactableBodiesFactory = model.getContactPointParameters().getContactableBodiesFactory();
      SideDependentList<String> footSensorNames = model.getSensorInformation().getFeetForceSensorNames();
      WalkingControllerParameters walkingControllerParameters = model.getWalkingControllerParameters();
      ArmControllerParameters armControllerParameters = model.getArmControllerParameters();
      CapturePointPlannerParameters capturePointPlannerParameters = model.getCapturePointPlannerParameters();
      MomentumBasedControllerFactory momentumBasedControllerFactory = new MomentumBasedControllerFactory(contactableBodiesFactory, footSensorNames, walkingControllerParameters, armControllerParameters, capturePointPlannerParameters, HighLevelState.DO_NOTHING_BEHAVIOR);
      DiagnosticsWhenHangingControllerFactory diagnosticsWhenHangingControllerFactory = new DiagnosticsWhenHangingControllerFactory(humanoidJointPoseList, useArms, robotIsHanging);
      diagnosticsWhenHangingControllerFactory.setTransitionRequested(true);
      momentumBasedControllerFactory.addHighLevelBehaviorFactory(diagnosticsWhenHangingControllerFactory);

      GlobalDataProducer globalDataProducer = null;

      DRCSimulationFactory drcSimulation = new DRCSimulationFactory(model, momentumBasedControllerFactory, environment, robotInitialSetup, scsInitialSetup,
            guiInitialSetup, globalDataProducer);

      simulationConstructionSet = drcSimulation.getSimulationConstructionSet();

      drcSimulation.start();
//      drcSimulation.simulate();
       
      if (DRCSimulationFactory.RUN_MULTI_THREADED)
      {
         throw new RuntimeException("This only works with single threaded right now. Change DRCSimulationFactory.RUN_MULTI_THREADED to false!");
      }
      
      FullRobotModelCorruptor fullRobotModelCorruptor = drcSimulation.getFullRobotModelCorruptor();
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
      
      ThinBufferButton thinBufferButton = new ThinBufferButton(simulationConstructionSet);
      simulationConstructionSet.addButton(thinBufferButton);
      
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
   
   public void updateDataAndComputeTorqueOffsetsBasedOnAverages()
   {
      analyzer.updateDataAndComputeTorqueOffsetsBasedOnAverages();
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
   
   private class ThinBufferButton extends JButton implements ActionListener
   {
      private static final long serialVersionUID = 4260526727108638954L;
      private final SimulationConstructionSet simulationConstructionSet;
      
      public ThinBufferButton(SimulationConstructionSet simulationConstructionSet)
      {
         super("Thin Buffer");
         
         this.simulationConstructionSet = simulationConstructionSet;
         this.addActionListener(this);
      }

      @Override
      public void actionPerformed(ActionEvent e)
      {
         simulationConstructionSet.thinBuffer(2);
      }

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
         analyzer.updateDataAndComputeTorqueOffsetsBasedOnAverages();
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
                  analyzer.optimizeCorruptorValues();
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

