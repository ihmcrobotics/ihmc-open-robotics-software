package us.ihmc.behaviors.stairs;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.*;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidRobotics.communication.packets.walking.WalkingStatus;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class TraverseStairsExecuteStepsState extends TraverseStairsState
{
   private final BehaviorHelper helper;
   private final TraverseStairsBehaviorParameters parameters;
   private final Supplier<FootstepPlannerOutput> footstepPlannerOutput;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final AtomicReference<Pose3D> goalInput = new AtomicReference<>();
   private final StatusLogger statusLogger;

   private final AtomicBoolean walkingComplete = new AtomicBoolean();

   public TraverseStairsExecuteStepsState(BehaviorHelper helper, TraverseStairsBehaviorParameters parameters, Supplier<FootstepPlannerOutput> footstepPlannerOutput)
   {
      this.helper = helper;
      this.parameters = parameters;
      this.footstepPlannerOutput = footstepPlannerOutput;
      this.robotInterface = helper.getOrCreateRobotInterface();
      this.statusLogger = helper.getOrCreateStatusLogger();

      helper.subscribeToControllerViaCallback(WalkingStatusMessage.class, message ->
      {
         if (message.getWalkingStatus() == WalkingStatus.COMPLETED.toByte())
         {
            walkingComplete.set(true);
         }
      });

      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.GOAL_INPUT, goalPose ->
      {
         LogTools.info("Received goal input: " + goalPose);
         goalInput.set(goalPose);
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
      FootstepPlannerOutput output = this.footstepPlannerOutput.get();
      boolean hasOutput = output != null;
      if (!hasOutput)
      {
         return false;
      }

      return output.getFootstepPlanningResult() == FootstepPlanningResult.FOUND_SOLUTION;
   }

   boolean walkingIsComplete()
   {
      return walkingComplete.get();
   }
}
