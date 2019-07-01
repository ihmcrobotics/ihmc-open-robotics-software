package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import us.ihmc.humanoidBehaviors.behaviors.diagnostic.RoughTerrainTimingBehavior.RoughTerrainOperatorTimingBehaviorStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.TimingBehaviorHelper;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class RoughTerrainTimingBehavior extends StateMachineBehavior<RoughTerrainOperatorTimingBehaviorStates>
{

   protected TimingBehaviorHelper timingBehavior;

   private YoDouble totalPlanningTime;;
   private YoDouble totalWalkTime;;

   public enum RoughTerrainOperatorTimingBehaviorStates
   {
      PLANNING, WALKING
   }

   public RoughTerrainTimingBehavior(String robotName, YoDouble yoTime, Ros2Node ros2Node)
   {
      super(robotName, "RoughTerrainOperatorTimingBehavior", RoughTerrainOperatorTimingBehaviorStates.class, yoTime, ros2Node);
      timingBehavior = new TimingBehaviorHelper(robotName, ros2Node);
      totalPlanningTime = new YoDouble("totalPlanningTime", registry);
      totalWalkTime = new YoDouble("totalWalkTime", registry);

      setupStateMachine();
   }

   @Override
   protected RoughTerrainOperatorTimingBehaviorStates configureStateMachineAndReturnInitialKey(StateMachineFactory<RoughTerrainOperatorTimingBehaviorStates, BehaviorAction> factory)
   {
      BehaviorAction planning = new BehaviorAction()
      {
         @Override
         public void onEntry()
         {
            super.onEntry();
            publishTextToSpeech("Timer Entering Planning State");

         }
         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.walkingStatusMessage.get() != null)
            {
               switch (WalkingStatus.fromByte(timingBehavior.walkingStatusMessage.getAndSet(null).getWalkingStatus()))
               {
               case STARTED:
               {
                  addTimeToPlanning();
                  return true;
               }
               case RESUMED:
               {
                  addTimeToPlanning();
                  return true;
               }
               default:
                  return false;
               }
            }

            return false;
         }
      };
      BehaviorAction walking = new BehaviorAction()
      {
         @Override
         public void onEntry()
         {
            super.onEntry();
            publishTextToSpeech("Timer Entering Walking State");

         }
         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.walkingStatusMessage.get() != null)
            {

               switch (WalkingStatus.fromByte(timingBehavior.walkingStatusMessage.getAndSet(null).getWalkingStatus()))
               {
               case ABORT_REQUESTED:
               {
                  addTimeToWalking();
                  return true;
               }
               case COMPLETED:
               {
                  addTimeToWalking();
                  return true;
               }
               case PAUSED:
               {
                  addTimeToWalking();
                  return true;
               }
               default:
                  return false;
               }
            }

            return false;
         }
      };

      factory.addStateAndDoneTransition(RoughTerrainOperatorTimingBehaviorStates.PLANNING, planning, RoughTerrainOperatorTimingBehaviorStates.WALKING);
      factory.addStateAndDoneTransition(RoughTerrainOperatorTimingBehaviorStates.WALKING, walking, RoughTerrainOperatorTimingBehaviorStates.PLANNING);

      return RoughTerrainOperatorTimingBehaviorStates.PLANNING;
   }

   private void addTimeToPlanning()
   {
      totalPlanningTime.add(getStateMachine().getTimeInCurrentState());
      publishTextToSpeech("Adding time to total planning: "+getStateMachine().getTimeInCurrentState());

   }
   private void addTimeToWalking()
   {
      totalWalkTime.add(getStateMachine().getTimeInCurrentState());
      publishTextToSpeech("Adding time to total walking: "+getStateMachine().getTimeInCurrentState());

   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
      publishTextToSpeech("Starting timer for walk task");
      //save start time
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Stopping timer for walk task");
      publishTextToSpeech("Total Planning Time:"+totalPlanningTime.getDoubleValue()+" Total Walking Time:"+totalWalkTime.getDoubleValue());
   }

}
