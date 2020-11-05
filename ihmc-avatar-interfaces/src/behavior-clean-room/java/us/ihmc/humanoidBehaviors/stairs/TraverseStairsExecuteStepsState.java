package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

public class TraverseStairsExecuteStepsState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final Supplier<FootstepPlannerOutput> footstepPlannerOutput;
   private final RemoteHumanoidRobotInterface robotInterface;

   private final AtomicBoolean walkingComplete = new AtomicBoolean();

   public TraverseStairsExecuteStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters, Supplier<FootstepPlannerOutput> footstepPlannerOutput)
   {
      this.helper = helper;
      this.parameters = parameters;
      this.footstepPlannerOutput = footstepPlannerOutput;
      this.robotInterface = helper.getOrCreateRobotInterface();

      helper.createROS2ControllerCallback(WalkingStatusMessage.class, message ->
      {
         if (message.getWalkingStatus() == WalkingStatus.COMPLETED.toByte())
         {
            walkingComplete.set(true);
         }
      });
   }

   @Override
   public void onEntry()
   {
      clearWalkingCompleteFlag();

      FootstepPlannerOutput footstepPlannerOutput = this.footstepPlannerOutput.get();
      if (footstepPlannerOutput == null)
      {
         throw new RuntimeException("Footstep planner output is null");
      }

      FootstepPlan footstepPlan = footstepPlannerOutput.getFootstepPlan();
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, -1.0, -1.0);
      robotInterface.requestWalk(footstepDataListMessage);
   }

   @Override
   public void doAction(double timeInState)
   {

   }

   @Override
   public void onExit(double timeInState)
   {
      clearWalkingCompleteFlag();
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return walkingIsComplete() && !planEndsAtGoal();
   }

   public void clearWalkingCompleteFlag()
   {
      walkingComplete.set(false);
   }

   boolean planEndsAtGoal()
   {
      return footstepPlannerOutput.get() != null && footstepPlannerOutput.get().getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION;
   }

   boolean walkingIsComplete()
   {
      return walkingComplete.get();
   }
}
