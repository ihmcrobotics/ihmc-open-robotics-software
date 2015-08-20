package us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait;

import java.awt.Container;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Random;

import javax.swing.BoxLayout;
import javax.swing.JFrame;
import javax.swing.JPanel;

import us.ihmc.SdfLoader.models.FullHumanoidRobotModel;
import us.ihmc.SdfLoader.partNames.ArmJointName;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.RobotSpecificJointNames;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.commonWalkingControlModules.configurations.BalanceOnOneLegConfiguration;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.ArmTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LegTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.NeckTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.SpineTorques;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.UpperBodyTorques;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.IntegerYoVariable;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateChangedListener;
import us.ihmc.yoUtilities.stateMachines.StateMachine;
import us.ihmc.yoUtilities.stateMachines.StateMachinesJPanel;
import us.ihmc.yoUtilities.stateMachines.StateTransition;
import us.ihmc.yoUtilities.stateMachines.StateTransitionAction;
import us.ihmc.yoUtilities.stateMachines.StateTransitionCondition;

public abstract class RegularWalkingGaitAbstractController implements RobotController
{
   public static boolean DO_FORWARD_BACKWARD_SWING_ON_ONE_LEG = false;
   
   protected final RobotSpecificJointNames robotJointNames;

   protected final DoEveryTickSubController doEveryTickSubController;
   protected final StanceSubController stanceSubController;
   protected final SwingSubController swingSubController;
   protected final UpperBodySubController upperBodySubController;
   
   protected final CommonHumanoidReferenceFrames referenceFrames;

   protected final LowerBodyTorques lowerBodyTorques;
   protected final UpperBodyTorques upperBodyTorques = new UpperBodyTorques();
   protected final ProcessedOutputsInterface processedOutputs;

//   private final TorqueTransitionFilter torqueTransitionFilter;

   protected final YoVariableRegistry controllerRegistry;
   protected final YoVariableRegistry childRegistry = new YoVariableRegistry("GoAndSuch");

   protected final BooleanYoVariable go = new BooleanYoVariable("go", "Starts and stops the walking", childRegistry);
   protected final BooleanYoVariable balanceOnOneLeg = new BooleanYoVariable("balanceOnOneLeg", "Starts and stops balancing on one leg", childRegistry);
   protected final BooleanYoVariable backToDoubleSupport = new BooleanYoVariable("backToDoubleSupport", childRegistry);
   protected final BooleanYoVariable swingInAir = new BooleanYoVariable("swingInAir", "Starts and stops swinging to a new position in the air", childRegistry);

   protected final BooleanYoVariable resetSteps = new BooleanYoVariable("resetSteps", childRegistry);
   protected final IntegerYoVariable stepsTaken = new IntegerYoVariable("stepsTaken", childRegistry);
   protected final IntegerYoVariable stepsToTake = new IntegerYoVariable("stepsToTake", childRegistry);
   protected final BooleanYoVariable onFinalStep = new BooleanYoVariable("onFinalStep", childRegistry);

//   protected boolean doTorqueTransitionFilterInThisModule;
   
   protected final StateMachine walkingStateMachine;
   protected final EnumYoVariable<RobotSide> supportLegYoVariable = new EnumYoVariable<RobotSide>("supportLegForWalkingCtrlr",
                                                                       "Current support leg. Null if double support", childRegistry, RobotSide.class, true);
   protected final EnumYoVariable<RobotSide> oneLegBalanceSide = new EnumYoVariable<RobotSide>("oneLegBalanceSide", childRegistry, RobotSide.class);
   
   private final DoubleYoVariable timeInSwing = new DoubleYoVariable("timeInSwing", childRegistry);
   private final DoubleYoVariable timeInPreSwing = new DoubleYoVariable("timeInPreSwing", childRegistry);
   private final DoubleYoVariable timeInInitialSwing = new DoubleYoVariable("timeInInitialSwing", childRegistry);
   private final DoubleYoVariable timeInMidSwing = new DoubleYoVariable("timeInMidSwing", childRegistry);

   private final String name;
   private final FullHumanoidRobotModel fullRobotModel;

   public RegularWalkingGaitAbstractController(String name, RobotSpecificJointNames robotJointNames, FullHumanoidRobotModel fullRobotModel, DoubleYoVariable time,
           ProcessedOutputsInterface processedOutputs, DoEveryTickSubController doEveryTickSubController, StanceSubController stanceSubController,
           SwingSubController swingSubController, UpperBodySubController upperBodySubController, CommonHumanoidReferenceFrames referenceFrames,
           YoVariableRegistry controllerRegistry, RobotSide initialLoadingLeg)
   {
      this.name = name;
      this.fullRobotModel = fullRobotModel;
      this.robotJointNames = robotJointNames;
      this.processedOutputs = processedOutputs;

      this.doEveryTickSubController = doEveryTickSubController;
      this.stanceSubController = stanceSubController;
      this.swingSubController = swingSubController;
      this.upperBodySubController = upperBodySubController;
      this.referenceFrames = referenceFrames;
      this.controllerRegistry = controllerRegistry;
      controllerRegistry.addChild(childRegistry);

//      torqueTransitionFilter = new TorqueTransitionFilter(robotJointNames, processedOutputs, childRegistry);

      lowerBodyTorques = new LowerBodyTorques(robotJointNames);

      walkingStateMachine = new StateMachine("walkingState", "switchTime", RegularWalkingState.class, time, childRegistry);
      setupStateMachine(initialLoadingLeg);

      stepsToTake.set(16000);
      stepsTaken.set(0);
      onFinalStep.set(false);
//      createStateMachineWindow(true);
     
   }
   
   public void initialize()
   {
      doEveryTickSubController.initialize();
      swingSubController.initialize();
      stanceSubController.initialize();
   }

   public void doControl()
   {
//      // Remember the torques to filter them in a bit:
//      if(doTorqueTransitionFilterInThisModule)
//      {
//         torqueTransitionFilter.rememberPreviousTorques();
//      }
      
      doEveryTickSubController.doEveryControlTick(supportLegYoVariable.getEnumValue());
      walkingStateMachine.doAction();
      walkingStateMachine.checkTransitionConditions();

      // Filter torques:
      processedOutputs.incrementProcessedOutputsWhiteBoardIndex();
      
//      if(doTorqueTransitionFilterInThisModule)
//      {
//         torqueTransitionFilter.updateTorques(walkingStateMachine.timeInCurrentState());
//      }
   }

   public enum RegularWalkingState
   {
      StartWalkingDoubleSupportState, TransferAllLoadToLeftLegForWalking, LeftLoadingRightPreSwingA, LeftLoadingRightPreSwingB, LeftLoadingRightPreSwingC,
      LeftEarlyStanceRightInitialSwing, LeftLateStanceRightMidSwing, LeftTerminalStanceRightTerminalSwing, TransferAllLoadToRightLegForWalking,
      RightLoadingLeftPreSwingA, RightLoadingLeftPreSwingB, RightLoadingLeftPreSwingC, RightEarlyStanceLeftInitialSwing, RightLateStanceLeftMidSwing,
      RightTerminalStanceLeftTerminalSwing, stopWalkingLeftLoadingState, stopWalkingRightLoadingState, leftLoadingForSingleLegBalance,
      rightLoadingForSingleLegBalance, leftSingleLegBalanceRightPreSwingInAir, rightSingleLegBalanceRightPreSwingInAir, leftSingleLegBalanceRightSwingInAir, rightSingleLegBalanceRightSwingInAir, 
      blankState
      ;

//    public RobotSide getSupportLeg()
//    {
//       if ((this == LeftLoadingRightPreSwingC) || (this == LeftEarlyStanceRightInitialSwing) || (this == LeftLateStanceRightMidSwing)
//               || (this == LeftTerminalStanceRightTerminalSwing))
//       {
//          return RobotSide.LEFT;
//       }
//
//       else if ((this == RightLoadingLeftPreSwingC) || (this == RightEarlyStanceLeftInitialSwing) || (this == RightLateStanceLeftMidSwing)
//                || (this == RightTerminalStanceLeftTerminalSwing))
//       {
//          return RobotSide.RIGHT;
//       }
//
//       else if ((this == StartWalkingDoubleSupportState) || (this == UnloadRightLegToTransferIntoWalking) || (this == UnloadLeftLegToTransferIntoWalking)
//                || (this == RightLoadingLeftPreSwingA) || (this == RightLoadingLeftPreSwingB) || (this == LeftLoadingRightPreSwingA)
//                || (this == LeftLoadingRightPreSwingB))
//       {
//          return null;
//       }
//
//       else
//          throw new RuntimeException("Not implemented for " + this);
//    }
   }



   protected void setProcessedOutputsBodyTorques()
   {
      processedOutputs.setLowerBodyTorques(lowerBodyTorques);
      processedOutputs.setUpperBodyTorques(upperBodyTorques);
      
      for (RobotSide robotSide : RobotSide.values)
      {
         LegTorques legTorques = lowerBodyTorques.getLegTorques(robotSide);
         for (LegJointName legJointName : legTorques.getLegJointNames())
         {
            double tau = lowerBodyTorques.getLegTorques(robotSide).getTorque(legJointName);
            fullRobotModel.getLegJoint(robotSide, legJointName).setTau(tau);
         }
         
         ArmTorques armTorques = upperBodyTorques.getArmTorques(robotSide);
         for (ArmJointName armJointName : robotJointNames.getArmJointNames())
         {
            double tau = armTorques.getTorque(armJointName);
            fullRobotModel.getArmJoint(robotSide, armJointName).setTau(tau);
         }
      }
      
      SpineTorques spineTorques = upperBodyTorques.getSpineTorques();
      for (SpineJointName spineJointName : robotJointNames.getSpineJointNames())
      {
         double tau = spineTorques.getTorque(spineJointName);
         fullRobotModel.getSpineJoint(spineJointName).setTau(tau);
      }
      
      NeckTorques neckTorques = upperBodyTorques.getNeckTorques();
      for (NeckJointName neckJointName : robotJointNames.getNeckJointNames())
      {
         double tau = neckTorques.getTorque(neckJointName);
         fullRobotModel.getNeckJoint(neckJointName).setTau(tau);
      }
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
      
//      doTorqueTransitionFilterInThisModule = true;
////    torqueTransitionFilter.setTauFilterTime(0.05);
//      torqueTransitionFilter.setTauFilterTime(0.005);
   }

   public void setupParametersForM2V2()
   {
//      doTorqueTransitionFilterInThisModule = false;
      
//      torqueTransitionFilter.setTauFilterTime(0.15);
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

//       if (stanceSubController.isDoneStartStopWalkingDoubleSupport(loadingLeg, walkingStateMachine.timeInCurrentState()))
//       {
//          this.transitionToDefaultNextState();
//       }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(null);
         backToDoubleSupport.set(false);
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
         
         stanceSubController.doTransitionOutOfStartStopWalkingDoubleSupport(loadingLeg);
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
         
         timeInPreSwing.set(walkingStateMachine.timeInCurrentState());
         timeInSwing.set(timeInPreSwing.getDoubleValue());

         if (swingSubController.isDoneWithPreSwingC(swingLeg, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
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
         
         timeInInitialSwing.set(walkingStateMachine.timeInCurrentState());
         timeInSwing.set(timeInPreSwing.getDoubleValue() + timeInInitialSwing.getDoubleValue());

         if (swingSubController.isDoneWithInitialSwing(swingSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }
         

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(stanceSide);

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
         
         timeInMidSwing.set(walkingStateMachine.timeInCurrentState());
         timeInSwing.set(timeInPreSwing.getDoubleValue() + timeInInitialSwing.getDoubleValue() + timeInMidSwing.getDoubleValue());

         if (swingSubController.isDoneWithMidSwing(swingSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }
         

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(stanceSide);

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

         
         timeInSwing.set(timeInPreSwing.getDoubleValue() + timeInInitialSwing.getDoubleValue() + timeInMidSwing.getDoubleValue() + walkingStateMachine.timeInCurrentState());
         
         if (swingSubController.isDoneWithTerminalSwing(swingSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }
         
         

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(stanceSide);

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

//       stanceSubController.doDoubleSupportStanceControl(lowerBodyTorques, loadingLeg);
         stanceSubController.doStopWalkingDoubleSupport(lowerBodyTorques, null, walkingStateMachine.timeInCurrentState());

         upperBodySubController.doUpperBodyControl(upperBodyTorques);

//       if (stanceSubController.isDoneStartStopWalkingDoubleSupport(loadingLeg, walkingStateMachine.timeInCurrentState()))
//       {
////        this.transitionToDefaultNextState();
//       }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(null);    // null implies Double support
         stanceSubController.doTransitionIntoStartStopWalkingDoubleSupport(loadingLeg);
      }

      public void doTransitionOutOfAction()
      {
         supportLegYoVariable.set(loadingLeg);
         stanceSubController.doTransitionOutOfStartStopWalkingDoubleSupport(loadingLeg);
      }
   }


   protected class LoadingForSingleLegBalanceState extends State
   {
      private final RobotSide upcomingSupportSide;

      public LoadingForSingleLegBalanceState(RegularWalkingState stateName, RobotSide upcomingSupportSide)
      {
         super(stateName);
         this.upcomingSupportSide = upcomingSupportSide;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         stanceSubController.doLoadingForSingleLegBalance(lowerBodyTorques, upcomingSupportSide, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (stanceSubController.isDoneLoadingForSingleLegBalance(upcomingSupportSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         supportLegYoVariable.set(null);

         stanceSubController.doTransitionIntoLoadingForSingleLegBalance(upcomingSupportSide);
      }

      public void doTransitionOutOfAction()
      {
         supportLegYoVariable.set(upcomingSupportSide);
         stanceSubController.doTransitionOutOfLoadingForSingleLegBalance(upcomingSupportSide);
      }
   }

   protected class SingleLegBalancePreSwingInAirState extends State
   {
      private final RobotSide supportLeg;
      private final RobotSide swingLeg;
      private BalanceOnOneLegConfiguration currentConfiguration;


      public SingleLegBalancePreSwingInAirState(RegularWalkingState stateName, RobotSide supportLeg)
      {
         super(stateName);

         this.supportLeg = supportLeg;
         this.swingLeg = supportLeg.getOppositeSide();
         ReferenceFrame supportLegAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(supportLeg);

         FramePoint defaultDesiredCapturePoint = new FramePoint(supportLegAnkleZUpFrame);
         
         double defaultKneeBendSupportLeg = 0.0;

         this.currentConfiguration = new BalanceOnOneLegConfiguration(new double[3], defaultDesiredCapturePoint, new FramePoint(supportLegAnkleZUpFrame),
               defaultKneeBendSupportLeg);
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doPreSwingInAir(lowerBodyTorques.getLegTorques(swingLeg), walkingStateMachine.timeInCurrentState());
         stanceSubController.doSingleLegBalance(lowerBodyTorques.getLegTorques(supportLeg), supportLeg, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         if (swingSubController.isDoneWithPreSwingInAir(swingLeg, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }


         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {

         supportLegYoVariable.set(supportLeg);

         stanceSubController.doTransitionIntoSingleLegBalance(supportLeg, currentConfiguration.getYawPitchRoll());
         swingSubController.doTransitionIntoPreSwingInAir(swingLeg);
      }

      public void doTransitionOutOfAction()
      {
         stanceSubController.doTransitionOutOfSingleLegBalance(supportLeg);
         swingSubController.doTransitionOutOfPreSwingInAir(swingLeg);
      }

   }


   protected class SingleLegBalanceSwingInAirState extends State
   {
      private final RobotSide supportLeg;
      private final RobotSide swingLeg;
      private final BalanceOnOneLegConfiguration homeConfiguration;
      private final ArrayList<BalanceOnOneLegConfiguration> balanceOnOneLegConfigurations;
      private final BalanceOnOneLegConfiguration backToDoubleSupportConfiguration;
      private BalanceOnOneLegConfiguration currentConfiguration;
      private int randomConfigurationIndex = 0;

      public SingleLegBalanceSwingInAirState(RegularWalkingState stateName, RobotSide supportLeg)
      {
         super(stateName);

         this.supportLeg = supportLeg;
         this.swingLeg = supportLeg.getOppositeSide();

         ReferenceFrame supportLegAnkleZUpFrame = referenceFrames.getAnkleZUpFrame(supportLeg);

         double[] defaultYawPitchRoll = {0.0, 0.0, 0.0};
         FramePoint defaultDesiredCapturePoint = new FramePoint(supportLegAnkleZUpFrame);
         double defaultSwingFootY = supportLeg.negateIfLeftSide(0.25);
         double defaultKneeBendSupportLeg = 0.0;

         double homeSwingFootZ = 0.1;
         FramePoint homeDesiredSwingFootPosition = new FramePoint(supportLegAnkleZUpFrame, 0.0, defaultSwingFootY, homeSwingFootZ);
         this.homeConfiguration = new BalanceOnOneLegConfiguration(defaultYawPitchRoll, defaultDesiredCapturePoint, homeDesiredSwingFootPosition,
                 defaultKneeBendSupportLeg);

         double backToDoubleSupportSwingFootZ = 0.0;
         FramePoint backToDoubleSupportDesiredSwingFootPosition = new FramePoint(supportLegAnkleZUpFrame, 0.0, defaultSwingFootY,
                                                                     backToDoubleSupportSwingFootZ);
         this.backToDoubleSupportConfiguration = new BalanceOnOneLegConfiguration(defaultYawPitchRoll, defaultDesiredCapturePoint,
                 backToDoubleSupportDesiredSwingFootPosition, defaultKneeBendSupportLeg);

         if (DO_FORWARD_BACKWARD_SWING_ON_ONE_LEG)
         {
            this.balanceOnOneLegConfigurations = BalanceOnOneLegConfiguration.generateForwardBackward(supportLeg, supportLegAnkleZUpFrame);
         }
         else
         {
            int nConfigurations = 10;
            this.balanceOnOneLegConfigurations = BalanceOnOneLegConfiguration.generateABunch(nConfigurations, supportLeg, supportLegAnkleZUpFrame, false);
            Random random = new Random(108L);
            Collections.shuffle(balanceOnOneLegConfigurations, random);
         }
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doSwingInAir(lowerBodyTorques.getLegTorques(swingLeg), walkingStateMachine.timeInCurrentState());
         stanceSubController.doSingleLegBalance(lowerBodyTorques.getLegTorques(supportLeg), supportLeg, walkingStateMachine.timeInCurrentState());
         upperBodySubController.doUpperBodyControl(upperBodyTorques);

         // no default next state

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         determineCurrentConfiguration();

         supportLegYoVariable.set(supportLeg);

         stanceSubController.doTransitionIntoSingleLegBalance(supportLeg, currentConfiguration.getYawPitchRoll());
         swingSubController.doTransitionIntoSwingInAir(swingLeg, currentConfiguration);
      }

      public void doTransitionOutOfAction()
      {
         stanceSubController.doTransitionOutOfSingleLegBalance(supportLeg);
         swingSubController.doTransitionOutOfSwingInAir(swingLeg);
      }

      private void determineCurrentConfiguration()
      {
         if (backToDoubleSupport.getBooleanValue())
            currentConfiguration = backToDoubleSupportConfiguration;
         else if (!swingInAir.getBooleanValue())
            currentConfiguration = homeConfiguration;
         else
         {
            currentConfiguration = balanceOnOneLegConfigurations.get(randomConfigurationIndex);
            randomConfigurationIndex++;
            randomConfigurationIndex %= balanceOnOneLegConfigurations.size();
         }
      }
   }

   private void setupStateMachine(RobotSide initialLoadingLeg)
   {
      // Create states
      StartWalkingDoubleSupportState startWalkingDoubleSupportState = new StartWalkingDoubleSupportState(RegularWalkingState.StartWalkingDoubleSupportState,
                                                                         initialLoadingLeg);

      TransferAllLoadToLegForWalkingState transferAllLoadToLeftLegForWalkingState =
         new TransferAllLoadToLegForWalkingState(RegularWalkingState.TransferAllLoadToLeftLegForWalking, RobotSide.LEFT);
      TransferAllLoadToLegForWalkingState transferAllLoadToRightLegForWalkingState =
         new TransferAllLoadToLegForWalkingState(RegularWalkingState.TransferAllLoadToRightLegForWalking, RobotSide.RIGHT);

      LoadingPreSwingAState leftLoadingRightPreSwingAState = new LoadingPreSwingAState(RegularWalkingState.LeftLoadingRightPreSwingA, RobotSide.LEFT);
      LoadingPreSwingAState rightLoadingLeftPreSwingAState = new LoadingPreSwingAState(RegularWalkingState.RightLoadingLeftPreSwingA, RobotSide.RIGHT);

      LoadingPreSwingBState leftLoadingRightPreSwingBState = new LoadingPreSwingBState(RegularWalkingState.LeftLoadingRightPreSwingB, RobotSide.LEFT);
      LoadingPreSwingBState rightLoadingLeftPreSwingBState = new LoadingPreSwingBState(RegularWalkingState.RightLoadingLeftPreSwingB, RobotSide.RIGHT);

      LoadingPreSwingCState leftLoadingRightPreSwingCState = new LoadingPreSwingCState(RegularWalkingState.LeftLoadingRightPreSwingC, RobotSide.LEFT);
      LoadingPreSwingCState rightLoadingLeftPreSwingCState = new LoadingPreSwingCState(RegularWalkingState.RightLoadingLeftPreSwingC, RobotSide.RIGHT);

      EarlyStanceInitialSwingState leftEarlyStanceRightInitialSwingState =
         new EarlyStanceInitialSwingState(RegularWalkingState.LeftEarlyStanceRightInitialSwing, RobotSide.LEFT);
      EarlyStanceInitialSwingState rightEarlyStanceLeftInitialSwingState =
         new EarlyStanceInitialSwingState(RegularWalkingState.RightEarlyStanceLeftInitialSwing, RobotSide.RIGHT);

      LateStanceMidSwingState leftLateStanceRightMidSwingState = new LateStanceMidSwingState(RegularWalkingState.LeftLateStanceRightMidSwing, RobotSide.LEFT);
      LateStanceMidSwingState rightLateStanceLeftMidSwingState = new LateStanceMidSwingState(RegularWalkingState.RightLateStanceLeftMidSwing, RobotSide.RIGHT);

      TerminalStanceTerminalSwingState leftTerminalStanceRightTerminalSwingState =
         new TerminalStanceTerminalSwingState(RegularWalkingState.LeftTerminalStanceRightTerminalSwing, RobotSide.LEFT);
      TerminalStanceTerminalSwingState rightTerminalStanceLeftTerminalSwingState =
         new TerminalStanceTerminalSwingState(RegularWalkingState.RightTerminalStanceLeftTerminalSwing, RobotSide.RIGHT);

      StopWalkingDoubleSupportState stopWalkingLeftLoadingState = new StopWalkingDoubleSupportState(RegularWalkingState.stopWalkingLeftLoadingState,
                                                                     RobotSide.LEFT);
      StopWalkingDoubleSupportState stopWalkingRightLoadingState = new StopWalkingDoubleSupportState(RegularWalkingState.stopWalkingRightLoadingState,
                                                                      RobotSide.RIGHT);

      LoadingForSingleLegBalanceState leftLoadingForSingleLegBalanceState =
         new LoadingForSingleLegBalanceState(RegularWalkingState.leftLoadingForSingleLegBalance, RobotSide.LEFT);
      LoadingForSingleLegBalanceState rightLoadingForSingleLegBalanceState =
         new LoadingForSingleLegBalanceState(RegularWalkingState.rightLoadingForSingleLegBalance, RobotSide.RIGHT);

      SingleLegBalancePreSwingInAirState leftSingleLegBalanceRightPreSwingInAirState =
            new SingleLegBalancePreSwingInAirState(RegularWalkingState.leftSingleLegBalanceRightPreSwingInAir, RobotSide.LEFT);
         SingleLegBalancePreSwingInAirState rightSingleLegBalanceLeftPreSwingInAirState =
            new SingleLegBalancePreSwingInAirState(RegularWalkingState.rightSingleLegBalanceRightPreSwingInAir, RobotSide.RIGHT);
      
      SingleLegBalanceSwingInAirState leftSingleLegBalanceRightSwingInAirState =
         new SingleLegBalanceSwingInAirState(RegularWalkingState.leftSingleLegBalanceRightSwingInAir, RobotSide.LEFT);
      SingleLegBalanceSwingInAirState rightSingleLegBalanceLeftSwingInAirState =
         new SingleLegBalanceSwingInAirState(RegularWalkingState.rightSingleLegBalanceRightSwingInAir, RobotSide.RIGHT);
      
      
      
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
            boolean commandedToStop = (!go.getBooleanValue()) || (onFinalStep.getBooleanValue());
            boolean canWeStopNow = swingSubController.canWeStopNow() && stanceSubController.canWeStopNow();

            return commandedToStop && canWeStopNow;
         }
      };

      StateTransitionCondition toLeftSingleLegBalanceCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return balanceOnOneLeg.getBooleanValue() && (oneLegBalanceSide.getEnumValue() == RobotSide.LEFT);
         }
      };

      StateTransitionCondition toRightSingleLegBalanceCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return balanceOnOneLeg.getBooleanValue() && (oneLegBalanceSide.getEnumValue() == RobotSide.RIGHT);
         }
      };

      StateTransitionCondition swingInAirAgainCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            if (swingSubController.isDoneWithSwingInAir(oneLegBalanceSide.getEnumValue().getOppositeSide(), walkingStateMachine.timeInCurrentState()))
            {
               boolean commandedToSwingAgain = swingInAir.getBooleanValue();
               boolean endingSingleSupport = backToDoubleSupport.getBooleanValue() &&!swingSubController.isReadyForDoubleSupport(supportLegYoVariable.getEnumValue().getOppositeSide());

               return commandedToSwingAgain || endingSingleSupport;
            }

            return false;
         }
      };

      StateTransitionCondition returnToDoubleSupportFromSingleSupportBalanceCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return (backToDoubleSupport.getBooleanValue() && swingSubController.isReadyForDoubleSupport(supportLegYoVariable.getEnumValue().getOppositeSide()));
         }
      };

      StateTransitionCondition toLeftEarlyStanceRightInitialSwingFromSwingInAirCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return stanceSubController.needToTakeAStep(RobotSide.LEFT);
         }
      };
      
      StateTransitionCondition toRightEarlyStanceLeftInitialSwingFromSwingInAirCondition = new StateTransitionCondition()
      {
         public boolean checkCondition()
         {
            return stanceSubController.needToTakeAStep(RobotSide.RIGHT);
         }
      };
      
           
           
      // End: State transition conditions
      
      // Begin: create state transition actions
      StateTransitionAction switchBalanceSideAction = new StateTransitionAction()
      {
         
         public void doTransitionAction()
         {
            oneLegBalanceSide.set(oneLegBalanceSide.getEnumValue().getOppositeSide());
         }
      };
      // End: create state transition actions

      // Begin: create state transitions
      StateTransition toTransferAllLoadToLeftLegForWalkingState = new StateTransition(transferAllLoadToLeftLegForWalkingState.getStateEnum(),
                                                                     startWalkingCondition);
      StateTransition toTransferAllLoadToRightLegForWalkingState = new StateTransition(transferAllLoadToRightLegForWalkingState.getStateEnum(),
                                                                      startWalkingCondition);

      StateTransition toStopWalkingLeftLoadingState = new StateTransition(stopWalkingLeftLoadingState.getStateEnum(), stopWalkingCondition);
      StateTransition toStopWalkingRightLoadingState = new StateTransition(stopWalkingRightLoadingState.getStateEnum(), stopWalkingCondition);

      StateTransition toLeftLoadingForSingleLegBalanceState = new StateTransition(leftLoadingForSingleLegBalanceState.getStateEnum(),
                                                                 toLeftSingleLegBalanceCondition);
      StateTransition toRightLoadingForSingleLegBalanceState = new StateTransition(rightLoadingForSingleLegBalanceState.getStateEnum(),
                                                                  toRightSingleLegBalanceCondition);

      StateTransition toLeftSingleLegBalanceRightSwingInAirState = new StateTransition(leftSingleLegBalanceRightSwingInAirState.getStateEnum(),
                                                                      swingInAirAgainCondition);
      StateTransition toRightSingleLegBalanceLeftSwingInAirState = new StateTransition(rightSingleLegBalanceLeftSwingInAirState.getStateEnum(),
                                                                      swingInAirAgainCondition);

      StateTransition toStartWalkingDoubleSupportStateFromSingleLegBalance = new StateTransition(startWalkingDoubleSupportState.getStateEnum(),
                                                                                returnToDoubleSupportFromSingleSupportBalanceCondition);
      
      StateTransition toLeftEarlyStanceRightInitialSwingFromSwingInAir = new StateTransition(leftEarlyStanceRightInitialSwingState.getStateEnum(), toLeftEarlyStanceRightInitialSwingFromSwingInAirCondition, switchBalanceSideAction);
      StateTransition toRightEarlyStanceLeftInitialSwingFromSwingInAir = new StateTransition(rightEarlyStanceLeftInitialSwingState.getStateEnum(), toRightEarlyStanceLeftInitialSwingFromSwingInAirCondition, switchBalanceSideAction);
      
      // End: create state transitions

      // Begin: Add transitions
      // Start Walking
      if (initialLoadingLeg == RobotSide.LEFT)
         startWalkingDoubleSupportState.addStateTransition(toTransferAllLoadToLeftLegForWalkingState);
      else
         startWalkingDoubleSupportState.addStateTransition(toTransferAllLoadToRightLegForWalkingState);

//      startWalkingDoubleSupportState.addStateTransition(toTransferAllLoadToRightLegForWalkingState);

      // Stop walking
      leftLoadingRightPreSwingAState.addStateTransition(toStopWalkingLeftLoadingState);
      rightLoadingLeftPreSwingAState.addStateTransition(toStopWalkingRightLoadingState);

      // Restart walking
      stopWalkingLeftLoadingState.addStateTransition(toTransferAllLoadToLeftLegForWalkingState);
      stopWalkingRightLoadingState.addStateTransition(toTransferAllLoadToRightLegForWalkingState);

      // Go to single leg balance
      startWalkingDoubleSupportState.addStateTransition(toLeftLoadingForSingleLegBalanceState);
      startWalkingDoubleSupportState.addStateTransition(toRightLoadingForSingleLegBalanceState);
      stopWalkingLeftLoadingState.addStateTransition(toLeftLoadingForSingleLegBalanceState);
      stopWalkingLeftLoadingState.addStateTransition(toRightLoadingForSingleLegBalanceState);
      stopWalkingRightLoadingState.addStateTransition(toLeftLoadingForSingleLegBalanceState);
      stopWalkingRightLoadingState.addStateTransition(toRightLoadingForSingleLegBalanceState);

      // Swing again in single leg support
      leftSingleLegBalanceRightSwingInAirState.addStateTransition(toLeftSingleLegBalanceRightSwingInAirState);
      rightSingleLegBalanceLeftSwingInAirState.addStateTransition(toRightSingleLegBalanceLeftSwingInAirState);

      // Return to double support from single support
      leftSingleLegBalanceRightSwingInAirState.addStateTransition(toStartWalkingDoubleSupportStateFromSingleLegBalance);
      rightSingleLegBalanceLeftSwingInAirState.addStateTransition(toStartWalkingDoubleSupportStateFromSingleLegBalance);
      
      // Go to initial swing from swing in air (for push recovery)
      leftSingleLegBalanceRightSwingInAirState.addStateTransition(toLeftEarlyStanceRightInitialSwingFromSwingInAir);
      rightSingleLegBalanceLeftSwingInAirState.addStateTransition(toRightEarlyStanceLeftInitialSwingFromSwingInAir);

      
      
      // Add default state transition last
//    startWalkingDoubleSupportState.setDefaultNextState(transferAllLoadToRightLegForWalkingState.getStateEnum());
      transferAllLoadToLeftLegForWalkingState.setDefaultNextState(leftLoadingRightPreSwingCState.getStateEnum());
      transferAllLoadToRightLegForWalkingState.setDefaultNextState(rightLoadingLeftPreSwingCState.getStateEnum());

      leftLoadingRightPreSwingAState.setDefaultNextState(leftLoadingRightPreSwingBState.getStateEnum());
      leftLoadingRightPreSwingBState.setDefaultNextState(leftLoadingRightPreSwingCState.getStateEnum());
      leftLoadingRightPreSwingCState.setDefaultNextState(leftEarlyStanceRightInitialSwingState.getStateEnum());
      leftEarlyStanceRightInitialSwingState.setDefaultNextState(leftLateStanceRightMidSwingState.getStateEnum());
      leftLateStanceRightMidSwingState.setDefaultNextState(leftTerminalStanceRightTerminalSwingState.getStateEnum());
      leftTerminalStanceRightTerminalSwingState.setDefaultNextState(rightLoadingLeftPreSwingAState.getStateEnum());
      leftLoadingForSingleLegBalanceState.setDefaultNextState(leftSingleLegBalanceRightPreSwingInAirState.getStateEnum());
      leftSingleLegBalanceRightPreSwingInAirState.setDefaultNextState(leftSingleLegBalanceRightSwingInAirState.getStateEnum());

      rightLoadingLeftPreSwingAState.setDefaultNextState(rightLoadingLeftPreSwingBState.getStateEnum());
      rightLoadingLeftPreSwingBState.setDefaultNextState(rightLoadingLeftPreSwingCState.getStateEnum());
      rightLoadingLeftPreSwingCState.setDefaultNextState(rightEarlyStanceLeftInitialSwingState.getStateEnum());
      rightEarlyStanceLeftInitialSwingState.setDefaultNextState(rightLateStanceLeftMidSwingState.getStateEnum());
      rightLateStanceLeftMidSwingState.setDefaultNextState(rightTerminalStanceLeftTerminalSwingState.getStateEnum());
      rightTerminalStanceLeftTerminalSwingState.setDefaultNextState(leftLoadingRightPreSwingAState.getStateEnum());
      rightLoadingForSingleLegBalanceState.setDefaultNextState(rightSingleLegBalanceLeftPreSwingInAirState.getStateEnum());
      rightSingleLegBalanceLeftPreSwingInAirState.setDefaultNextState(rightSingleLegBalanceLeftSwingInAirState.getStateEnum());

      // End: Add transitions

      // Begin: Add states
      // walkingStateMachine.addState(new BlankState(RegularWalkingState.blankState));

      walkingStateMachine.addState(startWalkingDoubleSupportState);

      walkingStateMachine.addState(transferAllLoadToLeftLegForWalkingState);
      walkingStateMachine.addState(leftLoadingRightPreSwingAState);
      walkingStateMachine.addState(leftLoadingRightPreSwingBState);
      walkingStateMachine.addState(leftLoadingRightPreSwingCState);
      walkingStateMachine.addState(leftEarlyStanceRightInitialSwingState);
      walkingStateMachine.addState(leftLateStanceRightMidSwingState);
      walkingStateMachine.addState(leftTerminalStanceRightTerminalSwingState);
      walkingStateMachine.addState(leftLoadingForSingleLegBalanceState);
      walkingStateMachine.addState(leftSingleLegBalanceRightPreSwingInAirState);
      walkingStateMachine.addState(leftSingleLegBalanceRightSwingInAirState);

      walkingStateMachine.addState(transferAllLoadToRightLegForWalkingState);
      walkingStateMachine.addState(rightLoadingLeftPreSwingAState);
      walkingStateMachine.addState(rightLoadingLeftPreSwingBState);
      walkingStateMachine.addState(rightLoadingLeftPreSwingCState);
      walkingStateMachine.addState(rightEarlyStanceLeftInitialSwingState);
      walkingStateMachine.addState(rightLateStanceLeftMidSwingState);
      walkingStateMachine.addState(rightTerminalStanceLeftTerminalSwingState);
      walkingStateMachine.addState(rightLoadingForSingleLegBalanceState);
      walkingStateMachine.addState(rightSingleLegBalanceLeftPreSwingInAirState);
      walkingStateMachine.addState(rightSingleLegBalanceLeftSwingInAirState);

      walkingStateMachine.addState(stopWalkingLeftLoadingState);
      walkingStateMachine.addState(stopWalkingRightLoadingState);
      

      // End: Add states
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return getName();
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

         //ScrollPane pane = new ScrollPane();
         //pane.add(walkingStatePanel);
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
