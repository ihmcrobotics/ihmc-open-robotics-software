package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import controller_msgs.msg.dds.PauseWalkingMessage;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.behaviors.activeMapping.TerrainPlanningDebugger;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2PublisherBasics;

public class DoNothingState implements State
{
   private final ContinuousPlanner continuousPlanner;
   private final TerrainPlanningDebugger debugger;

   private final ROS2PublisherBasics<PauseWalkingMessage> pauseWalkingPublisher;

   /**
    * This state exists for when the state machine isn't doing anything, if we have gone back to this state after running the state machine, we reset a few
    * things like visuals and some initialization booleans.
    * When we leave this state we re-initialize the continuous planner as this can only mean we are starting things up.
    */
   public DoNothingState(ROS2Helper ros2Helper, String simpleRobotName, ContinuousPlanner continuousPlanner, TerrainPlanningDebugger debugger)
   {
      this.continuousPlanner = continuousPlanner;
      this.debugger = debugger;

      pauseWalkingPublisher = ros2Helper.getROS2NodeInterface().createPublisher(HumanoidControllerAPI.getTopic(PauseWalkingMessage.class, simpleRobotName));
   }

   @Override
   public void onEntry()
   {
   }

   @Override
   public void doAction(double timeInState)
   {
      PauseWalkingMessage message = new PauseWalkingMessage();

      // This state can be entered when we want to stop Continuous Hiking, if that's the cause, pause walking so the robot stops
      if (continuousPlanner.isInitialized())
      {
         message.setPause(true);
         message.setClearRemainingFootstepQueue(true);
         continuousPlanner.setLatestFootstepPlan(null);
         pauseWalkingPublisher.publish(message);
         debugger.resetVisualizationForUIPublisher();
      }

      continuousPlanner.setInitialized(false);
      continuousPlanner.requestMonteCarloPlannerReset();
   }

   @Override
   public void onExit(double timeInState)
   {
      // We are leaving this state, and going to create a footstep plan to use, initialize the continuous planner here
      // This gets initialized here because then it only happens once when we start walking, allowing us to know where we started from
      // THis should not happen at the beginning of the ready to plan state because we may enter that state often when re-planning, but we haven't ever stopped walking
      continuousPlanner.initialize();
      continuousPlanner.setPlanAvailable(false);
   }
}
