package us.ihmc.behaviors.buildingExploration;

import us.ihmc.behaviors.BehaviorModule;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.concurrent.atomic.AtomicBoolean;

class BuildingExplorationBehaviorTraverseStairsState implements State
{
   private final BehaviorHelper helper;
   private final AtomicBoolean isDone = new AtomicBoolean();
   private final Pose3DReadOnly bombPose;

   public BuildingExplorationBehaviorTraverseStairsState(BehaviorHelper helper, Pose3DReadOnly bombPose)
   {
      this.helper = helper;
      this.bombPose = bombPose;

      helper.subscribeViaCallback(TraverseStairsBehaviorAPI.COMPLETED, completed -> isDone.set(true));
   }

   @Override
   public void onEntry()
   {
      LogTools.info("Entering " + getClass().getSimpleName());

      isDone.set(false);

      String behaviorName = TraverseStairsBehavior.DEFINITION.getName();
      helper.publish(BehaviorModule.API.BehaviorSelection, behaviorName);
      ThreadTools.sleep(100);

      helper.publish(TraverseStairsBehaviorAPI.GOAL_INPUT, new Pose3D(bombPose));
      ThreadTools.sleep(100);

      helper.publish(TraverseStairsBehaviorAPI.START);
   }

   @Override
   public void doAction(double timeInState)
   {
   }

   @Override
   public void onExit(double timeInState)
   {
      LogTools.info("Exiting " + getClass().getSimpleName());
      helper.publish(BehaviorModule.API.BehaviorSelection, "null");
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return isDone.get();
   }
}
