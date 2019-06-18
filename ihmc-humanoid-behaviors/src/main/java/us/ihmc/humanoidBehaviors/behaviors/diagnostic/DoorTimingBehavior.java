package us.ihmc.humanoidBehaviors.behaviors.diagnostic;

import us.ihmc.humanoidBehaviors.behaviors.diagnostic.DoorTimingBehavior.DoorTimingBehaviorStates;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.SQLBehaviorDatabaseManager.Operator;
import us.ihmc.humanoidBehaviors.behaviors.diagnostic.SQLBehaviorDatabaseManager.Task;
import us.ihmc.humanoidBehaviors.behaviors.primitives.TimingBehaviorHelper;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.SleepBehavior;
import us.ihmc.humanoidBehaviors.stateMachine.StateMachineBehavior;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.stateMachine.factories.StateMachineFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoDouble;

public class DoorTimingBehavior extends StateMachineBehavior<DoorTimingBehaviorStates>
{

   public boolean operatorInControl = false;
   public boolean doorIsOpen = false;
   public double armTrajectoryTime = 0;
   protected TimingBehaviorHelper timingBehavior;

   private YoDouble totalTimeFindingDoor;
   private YoDouble totalTimePlanning;
   private YoDouble totalTimeApproach;
   private YoDouble totalTimeLocateHandle;
   private YoDouble totalTimeMovingArmToOpenDoor;
   private YoDouble totalTimePlanningArmMotionsToOpenDoor;

   private YoDouble totalTimePrepareToEnter;
   private YoDouble totalTimePlanningThroughDoor;
   private YoDouble totalTimeGoThroughDoor;

   public boolean transitionToWalking = false;
   public boolean transitionToManipulation = false;

   public SleepBehavior sleepBehavior;

   //database varaibles

   private Operator operator;
   private Task currentTask;
   private Run currentRun;

   public enum DoorTimingBehaviorStates
   {
      LOCATE_DOOR,
      PLANNING_TO_DOOR,
      APPROACH,
      TRANSITION_ON_APPROACH,
      TRANSITION_THROUGH_DOOR,
      PLANNING_OPEN_DOOR_ARM_MOTION,
      OPEN_DOOR_ARM_MOTION,
      PLANNING_THROUGH_DOOR,
      GO_THROUGH_DOOR
   }

   public DoorTimingBehavior(String robotName, YoDouble yoTime, Ros2Node ros2Node, boolean userControlled)
   {
      super(robotName, "DoorTimingBehaviorStates", DoorTimingBehaviorStates.class, yoTime, ros2Node);
      this.operatorInControl = userControlled;

      timingBehavior = new TimingBehaviorHelper(robotName, ros2Node);

      totalTimeFindingDoor = new YoDouble("totalTimeFindingDoor", registry);

      totalTimePlanning = new YoDouble("totalTimePlanning", registry);

      totalTimeApproach = new YoDouble("totalTimeApproach", registry);

      totalTimeLocateHandle = new YoDouble("totalTimeLocateHandle", registry);

      totalTimeMovingArmToOpenDoor = new YoDouble("totalTimeMovingArmToOpenDoor", registry);
      totalTimePlanningArmMotionsToOpenDoor = new YoDouble("totalTimePlanningArmMotionsToOpenDoor", registry);

      totalTimePrepareToEnter = new YoDouble("totalTimePrepareToEnter", registry);

      totalTimePlanningThroughDoor = new YoDouble("totalTimePlanningThroughDoor", registry);

      totalTimeGoThroughDoor = new YoDouble("totalTimeGoThroughDoor", registry);

      sleepBehavior = new SleepBehavior(robotName, ros2Node, yoTime);

      setupStateMachine();
   }

   private double getArmTrajectoryTime()
   {
      if (timingBehavior.armTrajectoryMessage.get() != null)
         return timingBehavior.armTrajectoryMessage.get().jointspace_trajectory_.getJointTrajectoryMessages().getLast().getTrajectoryPoints().getLast()
                                                                                .getTime();
      else if (timingBehavior.handTrajectoryMessage.get() != null)
         return timingBehavior.handTrajectoryMessage.get().getSe3Trajectory().getTaskspaceTrajectoryPoints().getLast().getTime();
      return 0;

   }

   @Override
   protected DoorTimingBehaviorStates configureStateMachineAndReturnInitialKey(StateMachineFactory<DoorTimingBehaviorStates, BehaviorAction> factory)
   {

      //is done once a door location is recieved
      BehaviorAction locateDoor = new BehaviorAction()
      {
         @Override
         public void onEntry()
         {
            super.onEntry();
            publishTextToSpeech("Timer Entering Locating Door State");

         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.doorLocationMessage.get() != null)
            {
               totalTimeFindingDoor.add(getStateMachine().getTimeInCurrentState());

               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.LOCATE_DOOR.toString(), getStateMachine().getTimeInCurrentState());

               publishTextToSpeech("Adding time to locating door: " + getStateMachine().getTimeInCurrentState());
               return true;
            }
            if (operatorInControl)
               return true;

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
         public void onEntry()
         {
            super.onEntry();

            publishTextToSpeech("Timer Entering Planning To Door State");

         }

         @Override
         public boolean isDone(double timeInState)
         {

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimePlanning.add(getStateMachine().getTimeInCurrentState());

               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.PLANNING_TO_DOOR.toString(), getStateMachine().getTimeInCurrentState());

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
         public void onEntry()
         {
            super.onEntry();
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
                  timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.APPROACH.toString(), getStateMachine().getTimeInCurrentState());

                  publishTextToSpeech("Adding time to total Walking: " + (getStateMachine().getTimeInCurrentState()));

                  return true;
               }
               case COMPLETED:
               {
                  totalTimeApproach.add(getStateMachine().getTimeInCurrentState());
                  timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.APPROACH.toString(), getStateMachine().getTimeInCurrentState());

                  publishTextToSpeech("Adding time to total Walking: " + (getStateMachine().getTimeInCurrentState()));

                  return true;
               }
               case PAUSED:
               {
                  totalTimeApproach.add(getStateMachine().getTimeInCurrentState());
                  timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.APPROACH.toString(), getStateMachine().getTimeInCurrentState());

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
         public void onEntry()
         {
            transitionToWalking = false;
            transitionToManipulation = false;
            super.onEntry();
         }

         @Override
         public void doAction(double timeInState)
         {

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimePlanning.add(getStateMachine().getTimeInCurrentState());
               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.PLANNING_TO_DOOR.toString(), getStateMachine().getTimeInCurrentState());

               publishTextToSpeech("Adding time to total planning: " + (getStateMachine().getTimeInCurrentState()));
               transitionToWalking = true;
            }
            else if (timingBehavior.armTrajectoryMessage.get() != null || timingBehavior.handTrajectoryMessage.get() != null)
            {
               armTrajectoryTime = getArmTrajectoryTime();
               totalTimePlanningArmMotionsToOpenDoor.add(getStateMachine().getTimeInCurrentState());
               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.PLANNING_OPEN_DOOR_ARM_MOTION.toString(),
                                        getStateMachine().getTimeInCurrentState());

               publishTextToSpeech("Adding time to openDoor Planning : " + getStateMachine().getTimeInCurrentState());
               transitionToManipulation = true;
            }

            super.doAction(timeInState);
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
      BehaviorAction movingArm = new BehaviorAction(sleepBehavior)
      {
         @Override
         public void onEntry()
         {
            super.onEntry();
            sleepBehavior.setSleepTime(armTrajectoryTime-0.5);
            timingBehavior.clean();
            publishTextToSpeech("Timer Entering OpeningDoor State");

         }

         @Override
         public boolean isDone(double timeInState)
         {

            //if you got another arm motion command
            if ((timingBehavior.armTrajectoryMessage.get() != null || timingBehavior.handTrajectoryMessage.get() != null))
            {
               armTrajectoryTime = getArmTrajectoryTime();

               totalTimeMovingArmToOpenDoor.add(getStateMachine().getTimeInCurrentState());
               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.OPEN_DOOR_ARM_MOTION.toString(), getStateMachine().getTimeInCurrentState());

               publishTextToSpeech("Adding time to openingDoor handle: " + getStateMachine().getTimeInCurrentState());
               return true;
            }

            //if a footstep command was sent durring an arm motion
            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimeMovingArmToOpenDoor.add(getStateMachine().getTimeInCurrentState());
               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.PLANNING_THROUGH_DOOR.toString(), getStateMachine().getTimeInCurrentState());

               publishTextToSpeech("Adding time to walkPlanning handle: " + getStateMachine().getTimeInCurrentState());
               return true;
            }

            //if the arm motion is complete
            if (sleepBehavior.isDone())
            {
               totalTimeMovingArmToOpenDoor.add(getStateMachine().getTimeInCurrentState());
               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.OPEN_DOOR_ARM_MOTION.toString(), getStateMachine().getTimeInCurrentState());

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

      //      if you have finished walking you are either going to walk again, move your arms, or search for the handle.
      BehaviorAction transitionThroughDoorState = new BehaviorAction()
      {

         @Override
         public void onEntry()
         {
            transitionToWalking = false;
            transitionToManipulation = false;
            super.onEntry();
         }

         @Override
         public void doAction(double timeInState)
         {

            if (timingBehavior.footstepDataListMessage.get() != null)
            {
               totalTimePlanning.add(getStateMachine().getTimeInCurrentState());
               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.PLANNING_THROUGH_DOOR.toString(), getStateMachine().getTimeInCurrentState());

               publishTextToSpeech("Adding time to total planning: " + (getStateMachine().getTimeInCurrentState()));
               transitionToWalking = true;
            }
            else if (timingBehavior.armTrajectoryMessage.get() != null || timingBehavior.handTrajectoryMessage.get() != null)
            {
               armTrajectoryTime = getArmTrajectoryTime();

               totalTimeMovingArmToOpenDoor.add(getStateMachine().getTimeInCurrentState());
               timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.PLANNING_OPEN_DOOR_ARM_MOTION.toString(),
                                        getStateMachine().getTimeInCurrentState());

               publishTextToSpeech("Adding time to Planning opening Door: " + getStateMachine().getTimeInCurrentState());
               transitionToManipulation = true;
            }

            super.doAction(timeInState);
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

      BehaviorAction walkingThroughDoor = new BehaviorAction()
      {
         @Override
         public void onEntry()
         {
            super.onEntry();
            publishTextToSpeech("Timer Entering Walking Through Door State");

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
      
                  return true;
               }
               case COMPLETED:
               {
 
                  return true;
               }
               case PAUSED:
               {
                  

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
            totalTimeGoThroughDoor.add(getStateMachine().getTimeInCurrentState());
            timingBehavior.saveEvent(currentRun.getRunID(), DoorTimingBehaviorStates.GO_THROUGH_DOOR.toString(), getStateMachine().getTimeInCurrentState());

            publishTextToSpeech("Adding time to total Walking Through Door: " + (getStateMachine().getTimeInCurrentState()));
            
            timingBehavior.clean();

            super.doPostBehaviorCleanup();

            currentRun.setSuccessful(true);
            timingBehavior.dataBase.updateRun(currentRun);
            publishTextToSpeech("Total number of run events added for door task: " + ( timingBehavior.dataBase.getNumberOfRunEventsAddedForRun(currentRun.getRunID())));
           
         }
      };

      // if this is the behavior start here, timer starts and stops when the door is lcoated
      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.LOCATE_DOOR, locateDoor, DoorTimingBehaviorStates.PLANNING_TO_DOOR);

      //if this is the operator start here, 
      //the timer stops when a footstep is sent to the controller
      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.PLANNING_TO_DOOR, planningToDoor, DoorTimingBehaviorStates.APPROACH);
      //timer stops when foot steps are paused, completed or aborted
      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.APPROACH, walkingToDoor, DoorTimingBehaviorStates.TRANSITION_ON_APPROACH);

      //from transition, if walking go back to approach, if searching go to open door, if manipulate, go to open door
      factory.addState(DoorTimingBehaviorStates.TRANSITION_ON_APPROACH, transitionState);
      factory.addTransition(DoorTimingBehaviorStates.TRANSITION_ON_APPROACH, DoorTimingBehaviorStates.APPROACH, t -> transitionToWalking);
      factory.addTransition(DoorTimingBehaviorStates.TRANSITION_ON_APPROACH, DoorTimingBehaviorStates.OPEN_DOOR_ARM_MOTION, t -> transitionToManipulation);

      factory.addState(DoorTimingBehaviorStates.TRANSITION_THROUGH_DOOR, transitionThroughDoorState);
      factory.addTransition(DoorTimingBehaviorStates.TRANSITION_THROUGH_DOOR, DoorTimingBehaviorStates.GO_THROUGH_DOOR, t -> transitionToWalking);
      factory.addTransition(DoorTimingBehaviorStates.TRANSITION_THROUGH_DOOR, DoorTimingBehaviorStates.OPEN_DOOR_ARM_MOTION, t -> transitionToManipulation);

      //if operator transition from open door to go through door else transition to prepair to enter door
      factory.addState(DoorTimingBehaviorStates.OPEN_DOOR_ARM_MOTION, movingArm);
      factory.addTransition(DoorTimingBehaviorStates.OPEN_DOOR_ARM_MOTION, DoorTimingBehaviorStates.TRANSITION_THROUGH_DOOR, t -> movingArm.isDone(Double.NaN));

      factory.addStateAndDoneTransition(DoorTimingBehaviorStates.GO_THROUGH_DOOR, walkingThroughDoor, DoorTimingBehaviorStates.TRANSITION_THROUGH_DOOR);

      return DoorTimingBehaviorStates.LOCATE_DOOR;
   }

   @Override
   public void onBehaviorEntered()
   {
      super.onBehaviorEntered();
      publishTextToSpeech("Entering door timing behavior");
      if (operatorInControl)
      {
         operator = timingBehavior.dataBase.saveOperator("Human1");

         currentTask = timingBehavior.dataBase.saveTask("Manual Walk Through Door");

      }
      else
      {

         operator = timingBehavior.dataBase.saveOperator("Auto_Behavior");

         currentTask = timingBehavior.dataBase.saveTask("Walk Through Door Behavior");

      }

      Run run = new Run(operator.operatorID, currentTask.taskID);
      currentRun = timingBehavior.dataBase.saveRun(run);

      //save start time
   }

   @Override
   public void onBehaviorExited()
   {
      publishTextToSpeech("leaving door timing behavior");
      // publishTextToSpeech("Total Planning Time:" + totalPlanningTime.getDoubleValue() + " Total Walking Time:" + totalWalkTime.getDoubleValue());
   }

}
