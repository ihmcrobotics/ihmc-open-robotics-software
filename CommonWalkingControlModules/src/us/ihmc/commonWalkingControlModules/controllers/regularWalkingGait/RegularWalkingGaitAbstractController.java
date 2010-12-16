package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import java.awt.Container;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.commonWalkingControlModules.RobotSide;
import us.ihmc.commonWalkingControlModules.filters.TorqueTransitionFilter;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.RobotSpecificJointNames;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.IntYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.statemachines.State;
import com.yobotics.simulationconstructionset.util.statemachines.StateChangedListener;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;
import com.yobotics.simulationconstructionset.util.statemachines.StateMachinesJPanel;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransition;
import com.yobotics.simulationconstructionset.util.statemachines.StateTransitionCondition;

public abstract class RegularWalkingGaitAbstractController
{
   protected final RobotSpecificJointNames robotJointNames;

   protected final DoEveryTickSubController doEveryTickSubController;
   protected final StanceSubController stanceSubController;
   protected final SwingSubController swingSubController;
   protected final UpperBodySubController upperBodySubController;

   protected final LowerBodyTorques lowerBodyTorques;
   protected final UpperBodyTorques upperBodyTorques = new UpperBodyTorques();
   protected final ProcessedOutputsInterface processedOutputs;

   private final TorqueTransitionFilter torqueTransitionFilter;
   
   protected final YoVariableRegistry controllerRegistry;
   protected final YoVariableRegistry childRegistry = new YoVariableRegistry("GoAndSuch");

   protected final BooleanYoVariable go = new BooleanYoVariable("go", "Starts and stops the walking", childRegistry);

   protected final BooleanYoVariable resetSteps = new BooleanYoVariable("resetSteps", childRegistry);
   protected final IntYoVariable stepsTaken = new IntYoVariable("stepsTaken", childRegistry);
   protected final IntYoVariable stepsToTake = new IntYoVariable("stepsToTake", childRegistry);
   protected final BooleanYoVariable onFinalStep = new BooleanYoVariable("onFinalStep", childRegistry);   

   protected final StateMachine walkingStateMachine;
   protected final EnumYoVariable<RobotSide> supportLegYoVariable = new EnumYoVariable<RobotSide>("supportLegForWalkingCtrlr", "Current support leg. Null if double support", childRegistry, RobotSide.class);
   protected final EnumYoVariable<RobotSide> swingLegYoVariable = EnumYoVariable.create("swingLeg", "Current support leg. Null if double support", RobotSide.class, childRegistry);

   private final String name;
   
   public RegularWalkingGaitAbstractController(String name,
         RobotSpecificJointNames robotJointNames,
         DoubleYoVariable time,
         ProcessedOutputsInterface processedOutputs, 
         DoEveryTickSubController doEveryTickSubController,
         StanceSubController stanceSubController,
         SwingSubController swingSubController,
         UpperBodySubController upperBodySubController,
         YoVariableRegistry controllerRegistry
   )
   {
      this.name = name;
      this.robotJointNames = robotJointNames;
      this.processedOutputs = processedOutputs;      
      
      this.doEveryTickSubController = doEveryTickSubController;
      this.stanceSubController = stanceSubController;
      this.swingSubController = swingSubController;
      this.upperBodySubController = upperBodySubController;
      
      this.controllerRegistry = controllerRegistry;
      controllerRegistry.addChild(childRegistry);
      
      torqueTransitionFilter = new TorqueTransitionFilter(robotJointNames, processedOutputs, childRegistry);

      lowerBodyTorques = new LowerBodyTorques(robotJointNames);
      
      walkingStateMachine = new StateMachine("walkingState", "switchTime", RegularWalkingState.class, time, childRegistry);
      setupStateMachine();
   
      stepsToTake.set(16000);
      stepsTaken.set(0);
      onFinalStep.set(false);
   }
   
   public void doControl()
   {
   // Remember the torques to filter them in a bit:
      torqueTransitionFilter.rememberPreviousTorques();
      
      doEveryTickSubController.doEveryControlTick(supportLegYoVariable.getEnumValue());
      walkingStateMachine.doAction();
      walkingStateMachine.checkTransitionConditions();
      
   // Filter torques:
      torqueTransitionFilter.updateTorques(walkingStateMachine.timeInCurrentState());
   }
   
   public enum RegularWalkingState
   {
      StartWalkingDoubleSupportState, 
      TransferAllLoadToLeftLegForWalking, 
      LeftLoadingRightPreSwingA,LeftLoadingRightPreSwingB, LeftLoadingRightPreSwingC, 
      LeftEarlyStanceRightInitialSwing, LeftLateStanceRightMidSwing, LeftTerminalStanceRightTerminalSwing,
      TransferAllLoadToRightLegForWalking,
      RightLoadingLeftPreSwingA, RightLoadingLeftPreSwingB, RightLoadingLeftPreSwingC, 
      RightEarlyStanceLeftInitialSwing, RightLateStanceLeftMidSwing, RightTerminalStanceLeftTerminalSwing,
      stopWalkingLeftLoadingState, stopWalkingRightLoadingState
      ;

//      public RobotSide getSupportLeg()
//      {
//         if ((this == LeftLoadingRightPreSwingC) || (this == LeftEarlyStanceRightInitialSwing) || (this == LeftLateStanceRightMidSwing)
//                 || (this == LeftTerminalStanceRightTerminalSwing))
//         {
//            return RobotSide.LEFT;
//         }
//
//         else if ((this == RightLoadingLeftPreSwingC) || (this == RightEarlyStanceLeftInitialSwing) || (this == RightLateStanceLeftMidSwing)
//                  || (this == RightTerminalStanceLeftTerminalSwing))
//         {
//            return RobotSide.RIGHT;
//         }
//
//         else if ((this == StartWalkingDoubleSupportState) || (this == UnloadRightLegToTransferIntoWalking) || (this == UnloadLeftLegToTransferIntoWalking)
//                  || (this == RightLoadingLeftPreSwingA) || (this == RightLoadingLeftPreSwingB) || (this == LeftLoadingRightPreSwingA)
//                  || (this == LeftLoadingRightPreSwingB))
//         {
//            return null;
//         }
//
//         else
//            throw new RuntimeException("Not implemented for " + this);
//      }
   }
   
   
   
   protected void setProcessedOutputsBodyTorques()
   {
      processedOutputs.setLowerBodyTorques(lowerBodyTorques);
      processedOutputs.setUpperBodyTorques(upperBodyTorques);
   }
   

   protected void setLowerBodyTorquesToZero()
   {
      lowerBodyTorques.setLowerBodyTorquesToZero();
   }
   
   
   public YoVariableRegistry getYoVariableRegistry()
   {
      return controllerRegistry;
   }
  
   
   public void setupParametersForR2()
   {
//      torqueTransitionFilter.setTauFilterTime(0.05);
      torqueTransitionFilter.setTauFilterTime(0.005);
   }
   
   public void setupParametersForM2V2()
   {
      torqueTransitionFilter.setTauFilterTime(0.15);
   }
   
   protected class StartWalkingDoubleSupportState extends State
   {
      private final RobotSide loadingLeg;

      public StartWalkingDoubleSupportState(RegularWalkingState stateName, RobotSide loadingLeg)
      {
         super(stateName);
         this.loadingLeg = loadingLeg;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         stanceSubController.doStartWalkingDoubleSupport(lowerBodyTorques, null, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

//         if (stanceSubController.isDoneStartStopWalkingDoubleSupport(loadingLeg, walkingStateMachine.timeInCurrentState()))
//         {
//            this.transitionToDefaultNextState();
//         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(null);
         swingLegYoVariable.set(null);

         stanceSubController.doTransitionIntoStartStopWalkingDoubleSupport(loadingLeg);
      }

      public void doTransitionOutOfAction()
      {
         if (resetSteps.getBooleanValue())
         {
            stepsTaken.set(0);
            onFinalStep.set(false);

            resetSteps.set(false);
         }
      }
   }
   
   
   protected class TransferAllLoadToLegForWalkingState extends State
   {
      private final RobotSide upcomingSupportSide;

      public TransferAllLoadToLegForWalkingState(RegularWalkingState stateName, RobotSide upcomingSupportSide)
      {
         super(stateName);
         this.upcomingSupportSide = upcomingSupportSide;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         stanceSubController.doUnloadLegToTransferIntoWalking(lowerBodyTorques, upcomingSupportSide, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (stanceSubController.isDoneUnloadLegToTransferIntoWalking(upcomingSupportSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(null);
         swingLegYoVariable.set(null);

         stanceSubController.doTransitionIntoUnloadLegToTransferIntoWalking(upcomingSupportSide);
      }

      public void doTransitionOutOfAction()
      {
         supportLegYoVariable.set(upcomingSupportSide);
         stanceSubController.doTransitionOutOfUnloadLegToTransferIntoWalking(upcomingSupportSide);
      }
   }

   
   
   protected class LoadingPreSwingAState extends State
   {
      private final RobotSide loadingLeg;

      public LoadingPreSwingAState(RegularWalkingState stateName, RobotSide loadingLeg)
      {
         super(stateName);
         this.loadingLeg = loadingLeg;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         stanceSubController.doLoadingPreSwingA(lowerBodyTorques, loadingLeg, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (stanceSubController.isDoneWithLoadingPreSwingA(loadingLeg, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(null);
         swingLegYoVariable.set(null);
         tookAnotherStep();
         stanceSubController.doTransitionIntoLoadingPreSwingA(loadingLeg);
      }

      public void doTransitionOutOfAction()
      {
         stanceSubController.doTransitionOutOfLoadingPreSwingA(loadingLeg);
      }
   }

   
   private void tookAnotherStep()
   {
      stepsTaken.set(stepsTaken.getIntegerValue() + 1);
      onFinalStep.set(stepsTaken.getIntegerValue() >= stepsToTake.getIntegerValue());
   }
   
   
   protected class LoadingPreSwingBState extends State
   {
      private final RobotSide loadingLeg;

      public LoadingPreSwingBState(RegularWalkingState stateEnum, RobotSide upcomingSupportLeg)
      {
         super(stateEnum);
         this.loadingLeg = upcomingSupportLeg;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         stanceSubController.doLoadingPreSwingB(lowerBodyTorques, loadingLeg, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (stanceSubController.isDoneWithLoadingPreSwingB(loadingLeg, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }
         
         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(null);
         swingLegYoVariable.set(null);

         stanceSubController.doTransitionIntoLoadingPreSwingB(loadingLeg);
      }

      public void doTransitionOutOfAction()
      {
         stanceSubController.doTransitionOutOfLoadingPreSwingB(loadingLeg);
      }
   }


   protected class LoadingPreSwingCState extends State
   {
      private final RobotSide supportLeg, swingLeg;

      public LoadingPreSwingCState(RegularWalkingState stateEnum, RobotSide supportLeg)
      {
         super(stateEnum);
         this.supportLeg = supportLeg;
         this.swingLeg = supportLeg.getOppositeSide();
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doPreSwing(lowerBodyTorques.getLegTorques(swingLeg), walkingStateMachine.timeInCurrentState());
         stanceSubController.doLoadingPreSwingC(lowerBodyTorques.getLegTorques(supportLeg), supportLeg, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (swingSubController.isDoneWithPreSwing(swingLeg, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }
         
         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         swingLegYoVariable.set(swingLeg);
         supportLegYoVariable.set(supportLeg);

         stanceSubController.doTransitionIntoLoadingPreSwingC(supportLeg);
         swingSubController.doTransitionIntoPreSwing(swingLeg);
      }

      public void doTransitionOutOfAction()
      {
         swingSubController.doTransitionOutOfPreSwing(swingLeg);
         stanceSubController.doTransitionOutOfLoadingPreSwingC(supportLeg);
      }
   }
   
   
   protected class EarlyStanceInitialSwingState extends State
   {
      private final RobotSide stanceSide, swingSide;

      public EarlyStanceInitialSwingState(RegularWalkingState stateName, RobotSide stanceSide)
      {
         super(stateName);
         this.stanceSide = stanceSide;
         this.swingSide = stanceSide.getOppositeSide();
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();
         
         swingSubController.doInitialSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());
         stanceSubController.doEarlyStance(lowerBodyTorques.getLegTorques(stanceSide), walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (swingSubController.isDoneWithInitialSwing(swingSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(stanceSide);
         swingLegYoVariable.set(swingSide);

         stanceSubController.doTransitionIntoEarlyStance(stanceSide);
         swingSubController.doTransitionIntoInitialSwing(swingSide);
      }

      public void doTransitionOutOfAction()
      {
         stanceSubController.doTransitionOutOfEarlyStance(stanceSide);
         swingSubController.doTransitionOutOfInitialSwing(swingSide);
      }

   }


   protected class LateStanceMidSwingState extends State
   {
      private final RobotSide stanceSide, swingSide;

      public LateStanceMidSwingState(RegularWalkingState stateName, RobotSide stanceSide)
      {
         super(stateName);
         this.stanceSide = stanceSide;
         this.swingSide = stanceSide.getOppositeSide();

      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doMidSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());
         stanceSubController.doLateStance(lowerBodyTorques.getLegTorques(stanceSide), walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (swingSubController.isDoneWithMidSwing(swingSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(stanceSide);
         swingLegYoVariable.set(swingSide);

         stanceSubController.doTransitionIntoLateStance(stanceSide);
         swingSubController.doTransitionIntoMidSwing(swingSide);
      }

      public void doTransitionOutOfAction()
      {
         stanceSubController.doTransitionOutOfLateStance(stanceSide);
         swingSubController.doTransitionOutOfMidSwing(swingSide);
      }
   }


   protected class TerminalStanceTerminalSwingState extends State
   {
      private final RobotSide stanceSide, swingSide;

      public TerminalStanceTerminalSwingState(RegularWalkingState stateName, RobotSide stanceSide)
      {
         super(stateName);
         this.stanceSide = stanceSide;
         this.swingSide = stanceSide.getOppositeSide();
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doTerminalSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());
         stanceSubController.doTerminalStance(lowerBodyTorques.getLegTorques(stanceSide), walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (swingSubController.isDoneWithTerminalSwing(swingSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(stanceSide);
         swingLegYoVariable.set(swingSide);

         stanceSubController.doTransitionIntoTerminalStance(stanceSide);
         swingSubController.doTransitionIntoTerminalSwing(swingSide);
      }

      public void doTransitionOutOfAction()
      {
         stanceSubController.doTransitionOutOfTerminalStance(stanceSide);
         swingSubController.doTransitionOutOfTerminalSwing(swingSide);
      }

   }

   
   protected class StopWalkingDoubleSupportState extends State
   {
      private final RobotSide loadingLeg;

      public StopWalkingDoubleSupportState(RegularWalkingState stateName, RobotSide loadingLeg)
      {
         super(stateName);
         this.loadingLeg = loadingLeg;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

//         stanceSubController.doDoubleSupportStanceControl(lowerBodyTorques, loadingLeg);
         stanceSubController.doStopWalkingDoubleSupport(lowerBodyTorques, null, walkingStateMachine.timeInCurrentState());

         upperBodySubController.doUpperBodyControl(upperBodyTorques);

//         if (stanceSubController.isDoneStartStopWalkingDoubleSupport(loadingLeg, walkingStateMachine.timeInCurrentState()))
//         {
////          this.transitionToDefaultNextState();
//         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(null);    // null implies Double support
         swingLegYoVariable.set(null);    // null implies Double support
      }

      public void doTransitionOutOfAction()
      {
         supportLegYoVariable.set(loadingLeg);
      }
   }
   
   private void setupStateMachine()
   {
      // Create states
      StartWalkingDoubleSupportState startWalkingDoubleSupportState = new StartWalkingDoubleSupportState(RegularWalkingState.StartWalkingDoubleSupportState,
            RobotSide.RIGHT);

      TransferAllLoadToLegForWalkingState transferAllLoadToLeftLegForWalkingState = new TransferAllLoadToLegForWalkingState(
            RegularWalkingState.TransferAllLoadToLeftLegForWalking, RobotSide.LEFT);
      TransferAllLoadToLegForWalkingState transferAllLoadToRightLegForWalkingState = new TransferAllLoadToLegForWalkingState(
            RegularWalkingState.TransferAllLoadToRightLegForWalking, RobotSide.RIGHT);

      LoadingPreSwingAState leftLoadingRightPreSwingAState = new LoadingPreSwingAState(RegularWalkingState.LeftLoadingRightPreSwingA, RobotSide.LEFT);
      LoadingPreSwingAState rightLoadingLeftPreSwingAState = new LoadingPreSwingAState(RegularWalkingState.RightLoadingLeftPreSwingA, RobotSide.RIGHT);

      LoadingPreSwingBState leftLoadingRightPreSwingBState = new LoadingPreSwingBState(RegularWalkingState.LeftLoadingRightPreSwingB, RobotSide.LEFT);
      LoadingPreSwingBState rightLoadingLeftPreSwingBState = new LoadingPreSwingBState(RegularWalkingState.RightLoadingLeftPreSwingB, RobotSide.RIGHT);

      LoadingPreSwingCState leftLoadingRightPreSwingCState = new LoadingPreSwingCState(RegularWalkingState.LeftLoadingRightPreSwingC, RobotSide.LEFT);
      LoadingPreSwingCState rightLoadingLeftPreSwingCState = new LoadingPreSwingCState(RegularWalkingState.RightLoadingLeftPreSwingC, RobotSide.RIGHT);

      EarlyStanceInitialSwingState leftEarlyStanceRightInitialSwingState = new EarlyStanceInitialSwingState(
            RegularWalkingState.LeftEarlyStanceRightInitialSwing, RobotSide.LEFT);
      EarlyStanceInitialSwingState rightEarlyStanceLeftInitialSwingState = new EarlyStanceInitialSwingState(
            RegularWalkingState.RightEarlyStanceLeftInitialSwing, RobotSide.RIGHT);

      LateStanceMidSwingState leftLateStanceRightMidSwingState = new LateStanceMidSwingState(RegularWalkingState.LeftLateStanceRightMidSwing, RobotSide.LEFT);
      LateStanceMidSwingState rightLateStanceLeftMidSwingState = new LateStanceMidSwingState(RegularWalkingState.RightLateStanceLeftMidSwing, RobotSide.RIGHT);

      TerminalStanceTerminalSwingState leftTerminalStanceRightTerminalSwingState = new TerminalStanceTerminalSwingState(
            RegularWalkingState.LeftTerminalStanceRightTerminalSwing, RobotSide.LEFT);
      TerminalStanceTerminalSwingState rightTerminalStanceLeftTerminalSwingState = new TerminalStanceTerminalSwingState(
            RegularWalkingState.RightTerminalStanceLeftTerminalSwing, RobotSide.RIGHT);

      StopWalkingDoubleSupportState stopWalkingLeftLoadingState = new StopWalkingDoubleSupportState(RegularWalkingState.stopWalkingLeftLoadingState,
            RobotSide.LEFT);
      StopWalkingDoubleSupportState stopWalkingRightLoadingState = new StopWalkingDoubleSupportState(RegularWalkingState.stopWalkingRightLoadingState,
            RobotSide.RIGHT);

      // Begin: State transition conditions
      StateTransitionCondition startWalkingCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            RobotSide loadingLeg = RobotSide.LEFT;
            boolean readyToWalk = stanceSubController.isReadyToStartStopWalkingDoubleSupport(loadingLeg, walkingStateMachine.timeInCurrentState());
            return (go.getBooleanValue() && readyToWalk);
         }
      };
      
      StateTransitionCondition stopWalkingCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            if (((go.getBooleanValue() == false) || (onFinalStep.getBooleanValue())) && swingSubController.canWeStopNowSwingSubController() && stanceSubController.canWeStopNowStanceSubController())
            {
               return true;
            }
            
            return false;
         }
      };

      // End: State transition conditions

      // Begin: create state transitions
      StateTransition toTransferAllLoadToLeftLegForWalkingState = new StateTransition(transferAllLoadToLeftLegForWalkingState.getStateEnum(),
            startWalkingCondition);
      StateTransition toTransferAllLoadToRightLegForWalkingState = new StateTransition(transferAllLoadToRightLegForWalkingState.getStateEnum(),
            startWalkingCondition);

      StateTransition toStopWalkingLeftLoadingState = new StateTransition(stopWalkingLeftLoadingState.getStateEnum(), stopWalkingCondition);
      StateTransition toStopWalkingRightLoadingState = new StateTransition(stopWalkingRightLoadingState.getStateEnum(), stopWalkingCondition);

      // End: create state transitions

      // Begin: Add transitions
      // Start Walking
      startWalkingDoubleSupportState.addStateTransition(toTransferAllLoadToLeftLegForWalkingState);
 
      // Stop walking
      leftLoadingRightPreSwingAState.addStateTransition(toStopWalkingLeftLoadingState);
      rightLoadingLeftPreSwingAState.addStateTransition(toStopWalkingRightLoadingState);

      // Restart walking
      stopWalkingLeftLoadingState.addStateTransition(toTransferAllLoadToLeftLegForWalkingState);
      stopWalkingRightLoadingState.addStateTransition(toTransferAllLoadToRightLegForWalkingState);

      // Add default state transition last
//      startWalkingDoubleSupportState.setDefaultNextState(transferAllLoadToRightLegForWalkingState.getStateEnum());
      transferAllLoadToLeftLegForWalkingState.setDefaultNextState(leftLoadingRightPreSwingCState.getStateEnum());
      transferAllLoadToRightLegForWalkingState.setDefaultNextState(rightLoadingLeftPreSwingCState.getStateEnum());

      rightLoadingLeftPreSwingAState.setDefaultNextState(rightLoadingLeftPreSwingBState.getStateEnum());
      rightLoadingLeftPreSwingBState.setDefaultNextState(rightLoadingLeftPreSwingCState.getStateEnum());
      rightLoadingLeftPreSwingCState.setDefaultNextState(rightEarlyStanceLeftInitialSwingState.getStateEnum());
      rightEarlyStanceLeftInitialSwingState.setDefaultNextState(rightLateStanceLeftMidSwingState.getStateEnum());
      rightLateStanceLeftMidSwingState.setDefaultNextState(rightTerminalStanceLeftTerminalSwingState.getStateEnum());
      rightTerminalStanceLeftTerminalSwingState.setDefaultNextState(leftLoadingRightPreSwingAState.getStateEnum());

      leftLoadingRightPreSwingAState.setDefaultNextState(leftLoadingRightPreSwingBState.getStateEnum());
      leftLoadingRightPreSwingBState.setDefaultNextState(leftLoadingRightPreSwingCState.getStateEnum());
      leftLoadingRightPreSwingCState.setDefaultNextState(leftEarlyStanceRightInitialSwingState.getStateEnum());
      leftEarlyStanceRightInitialSwingState.setDefaultNextState(leftLateStanceRightMidSwingState.getStateEnum());
      leftLateStanceRightMidSwingState.setDefaultNextState(leftTerminalStanceRightTerminalSwingState.getStateEnum());
      leftTerminalStanceRightTerminalSwingState.setDefaultNextState(rightLoadingLeftPreSwingAState.getStateEnum());

      // End: Add transitions

      // Begin: Add states
      walkingStateMachine.addState(startWalkingDoubleSupportState);

      walkingStateMachine.addState(transferAllLoadToLeftLegForWalkingState);
      walkingStateMachine.addState(leftLoadingRightPreSwingAState);
      walkingStateMachine.addState(leftLoadingRightPreSwingBState);
      walkingStateMachine.addState(leftLoadingRightPreSwingCState);
      walkingStateMachine.addState(leftEarlyStanceRightInitialSwingState);
      walkingStateMachine.addState(leftLateStanceRightMidSwingState);
      walkingStateMachine.addState(leftTerminalStanceRightTerminalSwingState);

      walkingStateMachine.addState(transferAllLoadToRightLegForWalkingState);
      walkingStateMachine.addState(rightLoadingLeftPreSwingAState);
      walkingStateMachine.addState(rightLoadingLeftPreSwingBState);
      walkingStateMachine.addState(rightLoadingLeftPreSwingCState);
      walkingStateMachine.addState(rightEarlyStanceLeftInitialSwingState);
      walkingStateMachine.addState(rightLateStanceLeftMidSwingState);
      walkingStateMachine.addState(rightTerminalStanceLeftTerminalSwingState);

      walkingStateMachine.addState(stopWalkingLeftLoadingState);
      walkingStateMachine.addState(stopWalkingRightLoadingState);

      // End: Add states
   }

   public String getName()
   {
      return name;
   }
   
   public SwingSubController getSwingSubController()
   {
      return swingSubController;
   }

   public StanceSubController getStanceSubController()
   {
      return stanceSubController;
   }

   public UpperBodySubController getUpperBodySubController()
   {
      return upperBodySubController;
   }

   public State getCurrentWalkingState()
   {
      return walkingStateMachine.getCurrentState();
   }
   
   
   public void attachStateChangedListener(StateChangedListener listener)
   {
      walkingStateMachine.attachStateChangedListener(listener);
   }
   
   
   public JPanel createStateMachineWindow(boolean inJFrame)
   {
      StateMachinesJPanel walkingStatePanel = new StateMachinesJPanel(walkingStateMachine);
      if (inJFrame)
      {
         JFrame jFrame = new JFrame("Walking State Window");
         Container contentPane = jFrame.getContentPane();
         contentPane.setLayout(new BoxLayout(contentPane, BoxLayout.X_AXIS));

         jFrame.getContentPane().add(walkingStatePanel);
         jFrame.pack();
         jFrame.setSize(450, 300);
         jFrame.setAlwaysOnTop(true);
         jFrame.setVisible(true);
      }

      // Doing the following will cause redraw when the state changes, but not
      // during replay or rewind:
      walkingStateMachine.attachStateChangedListener(walkingStatePanel);

      // Doing this will cause redraw every specified miliseconds:
      // walkingStatePanel.createUpdaterThread(5000);

      return walkingStatePanel;

   }
}
