package us.ihmc.humanoidBehaviors.behaviors.examples;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.examples.ExampleComplexBehaviorStateMachine.ExampleStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class ExampleComplexBehaviorStateMachine extends StateMachineBehavior<ExampleStates>
{
   public enum ExampleStates
   {
      STEP1, STEP2, STEP3, STEP4,
   }

   private final AtlasPrimitiveActions atlasPrimitiveActions;

   private final SleepBehavior sleepBehavior;
   private final SimpleArmMotionBehavior simpleArmMotionBehavior;
   private final ResetRobotBehavior resetRobotBehavior;

   public ExampleComplexBehaviorStateMachine(String robotName, Ros2Node ros2Node, YoDouble yoTime, AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, "ExampleStateMachine", ExampleStates.class, yoTime, ros2Node);
      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);

      this.atlasPrimitiveActions = atlasPrimitiveActions;

      //create your behaviors
      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);
      simpleArmMotionBehavior = new SimpleArmMotionBehavior(robotName, yoTime, atlasPrimitiveActions.referenceFrames, ros2Node, atlasPrimitiveActions);

      // FIXME
      //      statemachine.getStateYoVariable().addVariableChangedListener(new VariableChangedListener()
      //      {
      //
      //         @Override
      //         public void notifyOfVariableChange(YoVariable<?> v)
      //         {
      //            System.out.println("ExampleComplexBehaviorStateMachine: Changing state to " + statemachine.getCurrentState());
      //         }
      //      });
      setupStateMachine();
   }

   @Override
   public void onBehaviorEntered()
   {
      publishTextToSpeech("Starting Example Behavior");
      super.onBehaviorEntered();
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Ending Example Behavior");
   }

   @Override
   protected ExampleStates configureStateMachineAndReturnInitialKey(StateMachineFactory<ExampleStates, BehaviorAction> factory)
   {
      //TODO setup search for ball behavior

      BehaviorAction sleep = new BehaviorAction(sleepBehavior) // ExampleStates.ENABLE_LIDAR
      {
         @Override
         public void onEntry()
         {
            publishTextToSpeech("entering 3 second sleep behavior");
            super.onEntry();
         }
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("sleeping for 3 seconds");
            sleepBehavior.setSleepTime(3);
         }
         @Override
         public void onExit()
         {
            publishTextToSpeech("sleeping for 3 seconds complete");
            super.onExit();
         }
      };

      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior) // ExampleStates.RESET_ROBOT_PIPELINE_EXAMPLE
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Resetting Robot");
            super.setBehaviorInput();
         }
         @Override
         public void onExit()
         {
            publishTextToSpeech("reset robot complete");
            super.onExit();
         }
      };

      BehaviorAction setupRobot = new BehaviorAction(simpleArmMotionBehavior) // ExampleStates.SETUP_ROBOT_PARALLEL_STATEMACHINE_EXAMPLE
      {
         @Override
         protected void setBehaviorInput()
         {
            publishTextToSpeech("Setting Up Robot Pose");
         }
      };

     


      factory.addStateAndDoneTransition(ExampleStates.STEP1, setupRobot, ExampleStates.STEP3);
      factory.addStateAndDoneTransition(ExampleStates.STEP3, sleep, ExampleStates.STEP4);
      factory.addState(ExampleStates.STEP4, resetRobot);
      

      return ExampleStates.STEP1;
   }
   
}
