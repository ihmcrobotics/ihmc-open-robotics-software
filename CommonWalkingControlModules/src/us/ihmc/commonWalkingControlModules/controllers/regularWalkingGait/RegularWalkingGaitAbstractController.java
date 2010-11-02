package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import us.ihmc.commonWalkingControlModules.RobotSide;
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
import com.yobotics.simulationconstructionset.util.statemachines.StateMachine;

public class RegularWalkingGaitAbstractController
{
   protected final RobotSpecificJointNames robotJointNames;

   protected final DoEveryTickSubController doEveryTickSubController;
   protected final StanceSubController stanceSubController;
   protected final SwingSubController swingSubController;
   protected final UpperBodySubController upperBodySubController;

   protected final LowerBodyTorques lowerBodyTorques;
   protected final UpperBodyTorques upperBodyTorques = new UpperBodyTorques();
   protected final ProcessedOutputsInterface processedOutputs;

   protected final YoVariableRegistry controllerRegistry;
   protected final YoVariableRegistry childRegistry = new YoVariableRegistry("GoAndSuch");

   protected final BooleanYoVariable go = new BooleanYoVariable("go", "Starts and stops the walking", childRegistry);

   protected final BooleanYoVariable resetSteps = new BooleanYoVariable("resetSteps", childRegistry);
   protected final IntYoVariable stepsTaken = new IntYoVariable("stepsTaken", childRegistry);
   protected final IntYoVariable stepsToTake = new IntYoVariable("stepsToTake", childRegistry);
   protected final BooleanYoVariable onFinalStep = new BooleanYoVariable("onFinalStep", childRegistry);   
   
   protected final StateMachine walkingStateMachine;
   protected final EnumYoVariable<RobotSide> supportLegYoVariable = new EnumYoVariable<RobotSide>("supportLeg", "Current support leg. Null if double support", childRegistry, RobotSide.class);
   protected final EnumYoVariable<RobotSide> swingLegYoVariable = EnumYoVariable.create("swingLeg", "Current support leg. Null if double support", RobotSide.class, childRegistry);

   
   public RegularWalkingGaitAbstractController(
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
      this.robotJointNames = robotJointNames;
      this.processedOutputs = processedOutputs;      
      
      this.doEveryTickSubController = doEveryTickSubController;
      this.stanceSubController = stanceSubController;
      this.swingSubController = swingSubController;
      this.upperBodySubController = upperBodySubController;
      
      this.controllerRegistry = controllerRegistry;
      controllerRegistry.addChild(childRegistry);
      
      lowerBodyTorques = new LowerBodyTorques(robotJointNames);
      
      walkingStateMachine = new StateMachine("walkingState", "switchTime", RegularWalkingState.class, time, childRegistry);
   }
   
   
   public enum RegularWalkingState
   {
      StartStopWalkingDoubleSupportState, UnloadRightLegToTransferIntoWalking, UnloadLeftLegToTransferIntoWalking, 
      LeftLoadingRightPreSwingA,LeftLoadingRightPreSwingB, LeftLoadingRightPreSwingC, 
      LeftEarlyStanceRightInitialSwing, LeftLateStanceRightMidSwing, LeftTerminalStanceRightTerminalSwing,
      RightLoadingLeftPreSwingA, RightLoadingLeftPreSwingB, RightLoadingLeftPreSwingC, 
      RightEarlyStanceLeftInitialSwing, RightLateStanceLeftMidSwing, RightTerminalStanceLeftTerminalSwing,
      
      StartWalkingDoubleSupportState, TransferAllLoadToRightLegForWalking,
      TransferAllLoadToLeftLegForWalking, stopWalkingLeftLoadingState, stopWalkingRightLoadingState
      ;

      public RobotSide getSupportLeg()
      {
         if ((this == LeftLoadingRightPreSwingC) || (this == LeftEarlyStanceRightInitialSwing) || (this == LeftLateStanceRightMidSwing)
                 || (this == LeftTerminalStanceRightTerminalSwing))
         {
            return RobotSide.LEFT;
         }

         else if ((this == RightLoadingLeftPreSwingC) || (this == RightEarlyStanceLeftInitialSwing) || (this == RightLateStanceLeftMidSwing)
                  || (this == RightTerminalStanceLeftTerminalSwing))
         {
            return RobotSide.RIGHT;
         }

         else if ((this == StartStopWalkingDoubleSupportState) || (this == UnloadRightLegToTransferIntoWalking) || (this == UnloadLeftLegToTransferIntoWalking)
                  || (this == RightLoadingLeftPreSwingA) || (this == RightLoadingLeftPreSwingB) || (this == LeftLoadingRightPreSwingA)
                  || (this == LeftLoadingRightPreSwingB))
         {
            return null;
         }

         else
            throw new RuntimeException("Not implemented for " + this);
      }
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

         stanceSubController.doStartStopWalkingDoubleSupport(lowerBodyTorques, null, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (stanceSubController.isDoneStartStopWalkingDoubleSupport(loadingLeg, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

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

         stanceSubController.doEarlyStance(lowerBodyTorques.getLegTorques(stanceSide), walkingStateMachine.timeInCurrentState());
         swingSubController.doInitialSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());
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

         stanceSubController.doLateStance(lowerBodyTorques.getLegTorques(stanceSide), walkingStateMachine.timeInCurrentState());
         swingSubController.doMidSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());
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

         stanceSubController.doTerminalStance(lowerBodyTorques.getLegTorques(stanceSide), walkingStateMachine.timeInCurrentState());
         swingSubController.doTerminalSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());
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

         stanceSubController.doDoubleSupportStanceControl(lowerBodyTorques, loadingLeg);
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (stanceSubController.isDoneStartStopWalkingDoubleSupport(loadingLeg, walkingStateMachine.timeInCurrentState()))
         {
//          this.transitionToDefaultNextState();
         }

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

   
}
