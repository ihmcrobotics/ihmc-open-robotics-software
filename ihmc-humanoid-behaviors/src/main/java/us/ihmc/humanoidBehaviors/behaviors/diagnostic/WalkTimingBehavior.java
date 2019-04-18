package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import us.ihmc.humanoidBehaviors.behaviors.diagnostic.SQLBehaviorDatabaseManager.Operator;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.SQLBehaviorDatabaseManager.Run;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.SQLBehaviorDatabaseManager.Task;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.WalkTimingBehavior.WalkTimingBehaviorStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.TimingBehaviorHelper;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class WalkTimingBehavior extends StateMachineBehavior<WalkTimingBehaviorStates>
{

   public boolean operatorInControl = false;
   public boolean doorIsOpen = false;
   public double armTrajectoryTime = 0;
   protected TimingBehaviorHelper timingBehavior;

   private YoDouble totalTimePlanning;
   private YoDouble totalTimeWalking;

   public SleepBehavior sleepBehavior;

   private Operator operator;
   private Task currentTask;
   private Run currentRun;

   public enum WalkTimingBehaviorStates
   {
      PLANNING,
      WALKING,
    }

   public WalkTimingBehavior(String robotName, YoDouble yoTime, Ros2Node ros2Node, boolean userControlled)
   {
      super(robotName, "WalkTimingBehaviorStates", WalkTimingBehaviorStates.class, yoTime, ros2Node);
      this.operatorInControl = userControlled;

      timingBehavior = new TimingBehaviorHelper(robotName, ros2Node);

      totalTimePlanning = new YoDouble("totalTimePlanning", registry);

      totalTimeWalking = new YoDouble("totalTimeWalking", registry);

      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);

      setupStateMachine();
   }

   @Override
   protected WalkTimingBehaviorStates configureStateMachineAndReturnInitialKey(StateMachineFactory<WalkTimingBehaviorStates, BehaviorAction> factory)
   {

     // done once a step is requested
      BehaviorAction planning = new BehaviorAction()
      {

         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();


         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimePlanning.add(getStateMachine().getTimeInCurrentState());

               timingBehavior.saveEvent(currentRun.runID, WalkTimingBehaviorStates.PLANNING.toString(), getStateMachine().getTimeInCurrentState());

               publishTextToSpeech("Adding time to total planning: " + (getStateMachine().getTimeInCurrentState()));
               return true;
            }

            return false;
         }

         @Override
         public void doPostBehaviorCleanup()
         {
            timingBehavior.clean();
            super.doPostBehaviorCleanup();
         }
      };

      //starts with first foot step and is done once the walk is complete
      BehaviorAction walking = new BehaviorAction()
      {
         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();
            publishTextToSpeech("Timer Entering Walking State");

         }

         @Override
         public boolean isDone(double timeInState)
         {
            if (timingBehavior.walkingStatusMessage.get() != null)
            {
               switch (WalkingStatus.fromByte(timingBehavior.walkingStatusMessage.get().getWalkingStatus()))
               {
               case ABORT_REQUESTED:
               {
                  totalTimeWalking.add(getStateMachine().getTimeInCurrentState());
                  timingBehavior.saveEvent(currentRun.runID, WalkTimingBehaviorStates.WALKING.toString(), getStateMachine().getTimeInCurrentState());
                  publishTextToSpeech("Adding time to total Walking: " + (getStateMachine().getTimeInCurrentState()));
                  return true;
               }
               case COMPLETED:
               {
                  totalTimeWalking.add(getStateMachine().getTimeInCurrentState());
                  timingBehavior.saveEvent(currentRun.runID, WalkTimingBehaviorStates.WALKING.toString(), getStateMachine().getTimeInCurrentState());
                  publishTextToSpeech("Adding time to total Walking: " + (getStateMachine().getTimeInCurrentState()));
                  return true;
               }
               case PAUSED:
               {
                  totalTimeWalking.add(getStateMachine().getTimeInCurrentState());
                  timingBehavior.saveEvent(currentRun.runID, WalkTimingBehaviorStates.WALKING.toString(), getStateMachine().getTimeInCurrentState());
                  publishTextToSpeech("Adding time to total Walking: " + (getStateMachine().getTimeInCurrentState()));
                  return true;
               }
               default:
                  return false;
               }
            }
            return false;
         }

         @Override
         public void doPostBehaviorCleanup()
         {
            timingBehavior.clean();
            super.doPostBehaviorCleanup();
         }
      };

      //if this is the operator start here, 
      //the timer stops when a footstep is sent to the controller
      factory.addStateAndDoneTransition(WalkTimingBehaviorStates.PLANNING, planning, WalkTimingBehaviorStates.WALKING);
      //timer stops when foot steps are paused, completed or aborted
      factory.addStateAndDoneTransition(WalkTimingBehaviorStates.WALKING, walking, WalkTimingBehaviorStates.PLANNING);
      
      return WalkTimingBehaviorStates.PLANNING;
   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
      publishTextToSpeech("Entering walk timing behavior");
      if (operatorInControl)
      {
         operator = timingBehavior.dataBase.saveOperator("Human1");

         currentTask = timingBehavior.dataBase.saveTask("Manual Walk");

      }
      else
      {

         operator = timingBehavior.dataBase.saveOperator("Auto_Behavior");

         currentTask = timingBehavior.dataBase.saveTask("Behavior Walk");

      }
      System.out.println("********************************************** "+currentTask.taskID);
      currentRun = timingBehavior.dataBase.saveRun(timingBehavior.dataBase.new Run(operator.operatorID, currentTask.taskID));

      //save start time
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("leaving walk timing behavior");
      // publishTextToSpeech("Total Planning Time:" + totalPlanningTime.getDoubleValue() + " Total Walking Time:" + totalWalkTime.getDoubleValue());
   }

}
