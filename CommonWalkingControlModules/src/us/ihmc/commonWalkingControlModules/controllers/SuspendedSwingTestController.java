package us.ihmc.commonWalkingControlModules.controllers;

import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.DoEveryTickSubController;
import us.ihmc.commonWalkingControlModules.controllers.regularWalkingGait.SwingSubController;
import us.ihmc.commonWalkingControlModules.outputs.ProcessedOutputsInterface;
import us.ihmc.commonWalkingControlModules.partNamesAndTorques.LowerBodyTorques;
import us.ihmc.simulationconstructionset.robotController.RobotController;
import us.ihmc.robotics.humanoidRobot.frames.CommonHumanoidReferenceFrames;
import us.ihmc.robotics.humanoidRobot.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.stateMachines.State;
import us.ihmc.yoUtilities.stateMachines.StateMachine;

public class SuspendedSwingTestController implements RobotController
{
   private final String name = getClass().getSimpleName();

   protected final RobotSpecificJointNames robotJointNames;

   protected final DoEveryTickSubController doEveryTickSubController;
   protected final SwingSubController swingSubController;

   protected final CommonHumanoidReferenceFrames referenceFrames;

   protected final LowerBodyTorques lowerBodyTorques;
   protected final ProcessedOutputsInterface processedOutputs;

   protected final YoVariableRegistry controllerRegistry;
   protected final YoVariableRegistry childRegistry = new YoVariableRegistry("GoAndSuch");

   protected final BooleanYoVariable go = new BooleanYoVariable("go", "Starts and stops the walking", childRegistry);

   protected final StateMachine walkingStateMachine;
   private final DoubleYoVariable timeInSwing = new DoubleYoVariable("timeInSwing", childRegistry);
   private final DoubleYoVariable timeInPreSwing = new DoubleYoVariable("timeInPreSwing", childRegistry);
   private final DoubleYoVariable timeInInitialSwing = new DoubleYoVariable("timeInInitialSwing", childRegistry);
   private final DoubleYoVariable timeInMidSwing = new DoubleYoVariable("timeInMidSwing", childRegistry);

   //   protected final EnumYoVariable<RobotSide> legToTest = new EnumYoVariable<RobotSide>("legToTest", childRegistry, RobotSide.class);

   public enum SwingState
   {
      restState, initialSwing, midSwing, terminalSwing, preSwing
   }

   public SuspendedSwingTestController(RobotSpecificJointNames robotJointNames, DoubleYoVariable time, ProcessedOutputsInterface processedOutputs,
         DoEveryTickSubController doEveryTickSubController, SwingSubController swingSubController, CommonHumanoidReferenceFrames referenceFrames,
         YoVariableRegistry controllerRegistry)
   {
      this.robotJointNames = robotJointNames;
      this.processedOutputs = processedOutputs;

      this.doEveryTickSubController = doEveryTickSubController;
      this.swingSubController = swingSubController;
      this.referenceFrames = referenceFrames;
      this.controllerRegistry = controllerRegistry;
      controllerRegistry.addChild(childRegistry);
      

      lowerBodyTorques = new LowerBodyTorques(robotJointNames);

      walkingStateMachine = new StateMachine("swingState", "switchTime", SwingState.class, time, childRegistry);
      setupStateMachine();
      


   }

   public void initialize()
   {
      doEveryTickSubController.initialize();
      swingSubController.initialize();
   }

   protected void setProcessedOutputsBodyTorques()
   {
      processedOutputs.setLowerBodyTorques(lowerBodyTorques);
   }

   protected void setLowerBodyTorquesToZero()
   {
      lowerBodyTorques.setLowerBodyTorquesToZero();
   }

   protected class RestState extends State
   {

      public RestState(SwingState stateName)
      {
         super(stateName);
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         if (go.getBooleanValue())
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {

      }

      public void doTransitionOutOfAction()
      {

      }
   }
   
   protected class PreSwingState extends State
   {
      private final RobotSide swingLeg;

      public PreSwingState(SwingState stateEnum, RobotSide swingLeg)
      {
         super(stateEnum);
         this.swingLeg = swingLeg;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doPreSwing(lowerBodyTorques.getLegTorques(swingLeg), walkingStateMachine.timeInCurrentState());


         if (swingSubController.isDoneWithPreSwingC(swingLeg, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }

         timeInPreSwing.set(walkingStateMachine.timeInCurrentState());
         timeInSwing.set(timeInPreSwing.getDoubleValue());
         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         swingSubController.doTransitionIntoPreSwing(swingLeg);
      }

      public void doTransitionOutOfAction()
      {
         swingSubController.doTransitionOutOfPreSwing(swingLeg);
      }
   }

   protected class InitialSwingState extends State
   {
      private final RobotSide swingSide;

      public InitialSwingState(SwingState stateName, RobotSide swingSide)
      {
         super(stateName);
         this.swingSide = swingSide;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doInitialSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());

         if (swingSubController.isDoneWithInitialSwing(swingSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }
         timeInInitialSwing.set(walkingStateMachine.timeInCurrentState());
         timeInSwing.set(timeInPreSwing.getDoubleValue() + timeInInitialSwing.getDoubleValue());
         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {

         swingSubController.doTransitionIntoInitialSwing(swingSide);
      }

      public void doTransitionOutOfAction()
      {
         swingSubController.doTransitionOutOfInitialSwing(swingSide);
      }

   }

   protected class MidSwingState extends State
   {
      private final RobotSide swingSide;

      public MidSwingState(SwingState stateName, RobotSide swingSide)
      {
         super(stateName);
         this.swingSide = swingSide;

      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doMidSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());

         if (swingSubController.isDoneWithMidSwing(swingSide, walkingStateMachine.timeInCurrentState()))
         {
            this.transitionToDefaultNextState();
         }
         
         timeInMidSwing.set(walkingStateMachine.timeInCurrentState());
         timeInSwing.set(timeInPreSwing.getDoubleValue() + timeInInitialSwing.getDoubleValue() + timeInMidSwing.getDoubleValue());
         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         swingSubController.doTransitionIntoMidSwing(swingSide);
      }

      public void doTransitionOutOfAction()
      {
         swingSubController.doTransitionOutOfMidSwing(swingSide);
      }
   }

   protected class TerminalSwingState extends State
   {
      private final RobotSide swingSide;

      public TerminalSwingState(SwingState stateName, RobotSide swingSide)
      {
         super(stateName);
         this.swingSide = swingSide;
      }

      public void doAction()
      {
         setLowerBodyTorquesToZero();

         swingSubController.doTerminalSwing(lowerBodyTorques.getLegTorques(swingSide), walkingStateMachine.timeInCurrentState());

//         if (swingSubController.isDoneWithTerminalSwing(swingSide, walkingStateMachine.timeInCurrentState()))
//         {
//            this.transitionToDefaultNextState();
//         }
         timeInSwing.set(timeInPreSwing.getDoubleValue() + timeInInitialSwing.getDoubleValue() + timeInMidSwing.getDoubleValue() + walkingStateMachine.timeInCurrentState());
         if(timeInSwing.getDoubleValue() > 0.7)
         {
            this.transitionToDefaultNextState();
         }

         setProcessedOutputsBodyTorques();
      }

      public void doTransitionIntoAction()
      {
         swingSubController.doTransitionIntoTerminalSwing(swingSide);
      }

      public void doTransitionOutOfAction()
      {
         swingSubController.doTransitionOutOfTerminalSwing(swingSide);
      }

   }

   private void setupStateMachine()
   {
      RestState restState = new RestState(SwingState.restState);
      PreSwingState preSwingState = new PreSwingState(SwingState.preSwing, RobotSide.LEFT);
      InitialSwingState initialSwingState = new InitialSwingState(SwingState.initialSwing, RobotSide.LEFT);
      MidSwingState midSwingState = new MidSwingState(SwingState.midSwing, RobotSide.LEFT);
      TerminalSwingState terminalSwingState = new TerminalSwingState(SwingState.terminalSwing, RobotSide.LEFT);

      restState.setDefaultNextState(preSwingState.getStateEnum());
      preSwingState.setDefaultNextState(initialSwingState.getStateEnum());
      initialSwingState.setDefaultNextState(midSwingState.getStateEnum());
      midSwingState.setDefaultNextState(terminalSwingState.getStateEnum());
      terminalSwingState.setDefaultNextState(restState.getStateEnum());

      walkingStateMachine.addState(restState);
      walkingStateMachine.addState(preSwingState);
      walkingStateMachine.addState(initialSwingState);
      walkingStateMachine.addState(midSwingState);
      walkingStateMachine.addState(terminalSwingState);

      walkingStateMachine.setCurrentState(restState.getStateEnum());

   }

   public YoVariableRegistry getYoVariableRegistry()
   {
      return controllerRegistry;
   }

   public String getName()
   {
      return name;
   }

   public String getDescription()
   {
      return name;
   }

   public void doControl()
   {

      doEveryTickSubController.doEveryControlTick(RobotSide.RIGHT);
      walkingStateMachine.doAction();
      walkingStateMachine.checkTransitionConditions();

      // Filter torques:
      processedOutputs.incrementProcessedOutputsWhiteBoardIndex();

   }

}
