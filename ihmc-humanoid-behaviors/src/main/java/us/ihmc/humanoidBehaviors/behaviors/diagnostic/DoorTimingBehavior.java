package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DoorTimingBehavior.DoorTimingBehaviorStates;
import us.ihmc.humanoidBehaviors.behaviors.primitives.BasicTimingBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.WalkToLocationPlannedBehavior.WalkToLocationStates;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoorTimingBehavior extends StateMachineBehavior<DoorTimingBehaviorStates>
{

   public boolean operatorInControl = false;
   public boolean doorIsOpen = false;
   protected BasicTimingBehavior timingBehavior;

   private YoDouble totalTimeFindingDoor;
   private YoDouble totalTimePlanning;
   private YoDouble totalTimeApproach;
   private YoDouble totalTimeLocateHandle;
   private YoDouble totalTimeOpenDoor;
   private YoDouble totalTimePrepareToEnter;
   private YoDouble totalTimePlanningThroughDoor;
   private YoDouble totalTimeGoThroughDoor;

   public boolean transitionToWalking = false;
   public boolean transitionToManipulation = false;
   public boolean transitionToPlanning = false;

   public enum DoorTimingBehaviorStates
   {
      LOCATE_DOOR, PLANNING, APPROACH, TRANSITION, LOCATE_HANDLE, OPEN_DOOR, PREPARE_TO_ENTER_DOOR, PLANNING_THROUGH_DOOR, GO_THROUGH_DOOR, 
   }

   public DoorTimingBehavior(String robotName, YoDouble yoTime, Ros2Node ros2Node, boolean userControlled)
   {
      super(robotName, "DoorTimingBehaviorStates", DoorTimingBehaviorStates.class, yoTime, ros2Node);
      this.operatorInControl = userControlled;
      timingBehavior = new BasicTimingBehavior(robotName, ros2Node);

      totalTimeFindingDoor = new YoDouble("totalTimeFindingDoor", registry);

      totalTimePlanning = new YoDouble("totalTimePlanning", registry);

      totalTimeApproach = new YoDouble("totalTimeApproach", registry);

      totalTimeLocateHandle = new YoDouble("totalTimeLocateHandle", registry);

      totalTimeOpenDoor = new YoDouble("totalTimeOpenDoor", registry);

      totalTimePrepareToEnter = new YoDouble("totalTimePrepareToEnter", registry);

      totalTimePlanningThroughDoor = new YoDouble("totalTimePlanningThroughDoor", registry);
      
      totalTimeGoThroughDoor = new YoDouble("totalTimeGoThroughDoor", registry);


      setupStateMachine();
   }

   @Override
   protected DoorTimingBehaviorStates configureStateMachineAndReturnInitialKey(StateMachineFactory<DoorTimingBehaviorStates, BehaviorAction> factory)
   {

      //is done once a door location is recieved
      BehaviorAction locateDoor = new BehaviorAction()
      {
         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();
            publishTextToSpeech("Timer Entering Locating Door State");

         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.doorLocationMessage.get() != null)
            {
               totalTimeFindingDoor.add(getStateMachine().getTimeInCurrentState());
               publishTextToSpeech("Adding time to locating door: " + getStateMachine().getTimeInCurrentState());
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
      // done once a step is requested
      BehaviorAction planningToDoor = new BehaviorAction()
      {

         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();
            publishTextToSpeech("Timer Entering Planning To Door State");

         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimePlanning.add(getStateMachine().getTimeInCurrentState());
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
      BehaviorAction walkingToDoor = new BehaviorAction()
      {
         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();
            publishTextToSpeech("Timer Entering Walking To Door State");

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
                  totalTimeApproach.add(getStateMachine().getTimeInCurrentState());
                  publishTextToSpeech("Adding time to total Walking: " + (getStateMachine().getTimeInCurrentState()));

                  return true;
               }
               case COMPLETED:
               {
                  totalTimeApproach.add(getStateMachine().getTimeInCurrentState());
                  publishTextToSpeech("Adding time to total Walking: " + (getStateMachine().getTimeInCurrentState()));

                  return true;
               }
               case PAUSED:
               {
                  totalTimeApproach.add(getStateMachine().getTimeInCurrentState());
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
      //      if you have finished walking you are either going to walk again, move your arms, or search for the handle.
      BehaviorAction transitionState = new BehaviorAction()
      {

         @Override
         public void doTransitionIntoAction()
         {
            transitionToWalking = false;
            transitionToManipulation = false;
            transitionToPlanning = false;
            super.doTransitionIntoAction();
         }

         @Override
         public void doAction()
         {

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimePlanning.add(getStateMachine().getTimeInCurrentState());
               publishTextToSpeech("Adding time to total planning: " + (getStateMachine().getTimeInCurrentState()));
               transitionToWalking = true;
            }
            else if (timingBehavior.armTrajectoryMessage.get() != null)
            {
               totalTimeOpenDoor.add(getStateMachine().getTimeInCurrentState());
               publishTextToSpeech("Adding time to openingDoor handle: " + getStateMachine().getTimeInCurrentState());
               transitionToManipulation = true;
            }
            else if (timingBehavior.doorLocationMessage.get() != null)
            {
               transitionToPlanning = true;
               totalTimeLocateHandle.add(getStateMachine().getTimeInCurrentState());
               publishTextToSpeech("Adding time to locating handle: " + getStateMachine().getTimeInCurrentState());
            }

            super.doAction();
         }

         @Override
         public boolean isDone(double timeInState)
         {
            return false;
         }

         @Override
         public void doPostBehaviorCleanup()
         {
            timingBehavior.clean();
            super.doPostBehaviorCleanup();
         }
      };

      //is done once a arm joint angle is sent signaling getting ready to walk through the door
      BehaviorAction openDoor = new BehaviorAction()
      {
         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();
            timingBehavior.clean();
            publishTextToSpeech("Timer Entering OpeningDoor State");
         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.armTrajectoryMessage.get() != null && !operatorInControl)
            {
               System.out.println("here1");
               totalTimeOpenDoor.add(getStateMachine().getTimeInCurrentState());
               publishTextToSpeech("Adding time to openingDoor handle: " + getStateMachine().getTimeInCurrentState());
               return true;
            }

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimeOpenDoor.add(getStateMachine().getTimeInCurrentState());
               publishTextToSpeech("Adding time to openingDoor handle: " + getStateMachine().getTimeInCurrentState());
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
      //is done once a arm joint angle is sent signaling getting ready to walk through the door
      BehaviorAction prepairToWalk = new BehaviorAction()
      {
         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();
            publishTextToSpeech("Timer Entering prepair To Walk State");
         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               doorIsOpen = true;
               totalTimePrepareToEnter.add(getStateMachine().getTimeInCurrentState());
               publishTextToSpeech("Adding time to openingDoor handle: " + getStateMachine().getTimeInCurrentState());
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
      BehaviorAction planningThroughDoor = new BehaviorAction()
      {

         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();
            publishTextToSpeech("Timer Entering Planning To Door State");

         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimePlanningThroughDoor.add(getStateMachine().getTimeInCurrentState());
               publishTextToSpeech("Adding time to total planning through door: " + (getStateMachine().getTimeInCurrentState()));
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
      BehaviorAction walkingThroughDoor = new BehaviorAction()
      {
         @Override
         public void doTransitionIntoAction()
         {
            super.doTransitionIntoAction();
            publishTextToSpeech("Timer Entering Walking Through Door State");

         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.walkingStatusMessage.get() != null)
            {
               System.out.println("WalkingStatus "+WalkingStatus.fromByte(timingBehavior.walkingStatusMessage.get().getWalkingStatus()));
               switch (WalkingStatus.fromByte(timingBehavior.walkingStatusMessage.get().getWalkingStatus()))
               {
               case ABORT_REQUESTED:
               {
                  totalTimeGoThroughDoor.add(getStateMachine().getTimeInCurrentState());
                  publishTextToSpeech("Adding time to total Walking Through Door: " + (getStateMachine().getTimeInCurrentState()));

                  return true;
               }
               case COMPLETED:
               {
                  totalTimeGoThroughDoor.add(getStateMachine().getTimeInCurrentState());
                  publishTextToSpeech("Adding time to total Walking Through Door: " + (getStateMachine().getTimeInCurrentState()));

                  return true;
               }
               case PAUSED:
               {
                  totalTimeGoThroughDoor.add(getStateMachine().getTimeInCurrentState());
                  publishTextToSpeech("Adding time to total Walking Through Door: " + (getStateMachine().getTimeInCurrentState()));

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
      
      

      // if this is the behavior start here, timer starts and stops when the door is lcoated
      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.LOCATE_DOOR, locateDoor, DoorTimingBehaviorStates.PLANNING);

      //if this is the operator start here, 
      //the timer stops when a footstep is sent to the controller
      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.PLANNING, planningToDoor, DoorTimingBehaviorStates.APPROACH);
      //timer stops when foot steps are paused, completed or aborted
      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.APPROACH, walkingToDoor, DoorTimingBehaviorStates.TRANSITION);

      //from transition, if walking go back to approach, if searching go to open door, if manipulate, go to open door
      factory.addState(DoorTimingBehaviorStates.TRANSITION, transitionState);
      factory.addTransition(DoorTimingBehaviorStates.TRANSITION, DoorTimingBehaviorStates.APPROACH, t -> transitionToWalking);
      factory.addTransition(DoorTimingBehaviorStates.TRANSITION, DoorTimingBehaviorStates.OPEN_DOOR, t -> transitionToManipulation);
      factory.addTransition(DoorTimingBehaviorStates.TRANSITION, DoorTimingBehaviorStates.PLANNING, t -> transitionToPlanning);

      //if operator transition from open door to go through door else transition to prepair to enter door
      factory.addState(DoorTimingBehaviorStates.OPEN_DOOR, openDoor);
      factory.addTransition(DoorTimingBehaviorStates.OPEN_DOOR, DoorTimingBehaviorStates.GO_THROUGH_DOOR, t -> openDoor.isDone() && operatorInControl);
      factory.addTransition(DoorTimingBehaviorStates.OPEN_DOOR, DoorTimingBehaviorStates.PREPARE_TO_ENTER_DOOR, t -> openDoor.isDone() && !operatorInControl);

      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.PREPARE_TO_ENTER_DOOR, prepairToWalk, DoorTimingBehaviorStates.GO_THROUGH_DOOR);
      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.PLANNING_THROUGH_DOOR, planningThroughDoor, DoorTimingBehaviorStates.GO_THROUGH_DOOR);
      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.GO_THROUGH_DOOR, walkingThroughDoor,DoorTimingBehaviorStates.PLANNING_THROUGH_DOOR);

      if (operatorInControl)
         return DoorTimingBehaviorStates.PLANNING;
      else
         return DoorTimingBehaviorStates.LOCATE_DOOR;
   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
      publishTextToSpeech("Starting timer for door task");
      //save start time
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("Stopping timer for door task");
      // publishTextToSpeech("Total Planning Time:" + totalPlanningTime.getDoubleValue() + " Total Walking Time:" + totalWalkTime.getDoubleValue());
   }

}
