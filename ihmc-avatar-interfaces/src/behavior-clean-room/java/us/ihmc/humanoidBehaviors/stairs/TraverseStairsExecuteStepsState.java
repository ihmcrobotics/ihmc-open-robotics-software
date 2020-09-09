package us.ihmc.humanoidBehaviors.stairs;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.footstepPlanning.FootstepDataMessageConverter;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepStatus;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.Supplier;

public class TraverseStairsExecuteStepsState implements State
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final Supplier<FootstepPlannerOutput> footstepPlannerOutput;
   private final RemoteHumanoidRobotInterface robotInterface;

   private int numberOfStepsInPlan;
   private final AtomicInteger numberOfCompletedSteps = new AtomicInteger();

   public TraverseStairsExecuteStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters, Supplier<FootstepPlannerOutput> footstepPlannerOutput)
   {
      this.helper = helper;
      this.parameters = parameters;
      this.footstepPlannerOutput = footstepPlannerOutput;

      robotInterface = helper.getOrCreateRobotInterface();

      helper.createROS2ControllerCallback(FootstepStatusMessage.class, message ->
      {
         if (message.getFootstepStatus() == FootstepStatus.COMPLETED.toByte())
         {
            numberOfCompletedSteps.incrementAndGet();
         }
      });
   }

   @Override
   public void onEntry()
   {
      FootstepPlannerOutput footstepPlannerOutput = this.footstepPlannerOutput.get();
      if (footstepPlannerOutput == null)
      {
         throw new RuntimeException("Footstep planner output is null");
      }

      FootstepPlan footstepPlan = footstepPlannerOutput.getFootstepPlan();
      FootstepDataListMessage footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan, -1.0, -1.0);
      numberOfStepsInPlan = footstepPlan.getNumberOfSteps();
      robotInterface.requestWalk(footstepDataListMessage);
   }

   @Override
   public void doAction(double timeInState)
   {

   }

   @Override
   public boolean isDone(double timeInState)
   {
      return numberOfCompletedSteps.get() >= numberOfStepsInPlan;
   }
}
