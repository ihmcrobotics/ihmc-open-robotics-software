package us.ihmc.behaviors.demo;

import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.behaviors.BehaviorDefinition;
import us.ihmc.behaviors.BehaviorInterface;
import us.ihmc.behaviors.door.DoorBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.tools.behaviorTree.ResettingNode;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.Goal;
import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.GoalForUI;
import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorTools.NAN_POSE;
import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI.REACHED_GOAL;

public class BuildingExplorationBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Building Exploration",
                                                                              BuildingExplorationBehavior::new,
                                                                              BuildingExplorationBehaviorAPI.API);
   private final BehaviorHelper helper;
   private final LookAndStepBehavior lookAndStepBehavior;
   private final DoorBehavior doorBehavior;
   private final RemoteSyncedRobotModel syncedRobot;
   private final AtomicReference<Pose3D> goal = new AtomicReference<>(NAN_POSE);
   private final TraverseStairsBehavior traverseStairsBehavior;

   public BuildingExplorationBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      LogTools.info("Constructing");
      syncedRobot = helper.newSyncedRobot();
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      addChild(lookAndStepBehavior);
      doorBehavior = new DoorBehavior(helper);
      doorBehavior.setSyncedRobot(syncedRobot);
      addChild(doorBehavior);
      traverseStairsBehavior = new TraverseStairsBehavior(helper);
      addChild(traverseStairsBehavior);
      helper.subscribeViaCallback(Goal, this::setGoal);
      helper.subscribeViaCallback(REACHED_GOAL, () -> setGoal(NAN_POSE));
   }

   private void setGoal(Pose3D newGoal)
   {
      goal.set(newGoal);
      if (!newGoal.containsNaN())
      {
         lookAndStepBehavior.acceptGoal(newGoal);
      }
      helper.publish(GoalForUI, goal.get());
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      syncedRobot.update();

      if (!goal.get().containsNaN())
      {
         if (doorBehavior.getDistanceToDoor() < 2.0)
         {
            return doorBehavior.tick();
         }
         else
         {
            if (lookAndStepBehavior.isReset())
               lookAndStepBehavior.acceptGoal(goal.get());
            return lookAndStepBehavior.tick();
         }
      }

      return BehaviorTreeNodeStatus.RUNNING;
   }

   @Override
   public void reset()
   {

   }

   @Override
   public void setEnabled(boolean enabled)
   {
      helper.setCommunicationCallbacksEnabled(enabled);
      if (!enabled)
         lookAndStepBehavior.setEnabled(false);
   }

   @Override
   public String getName()
   {
      return DEFINITION.getName();
   }

   @Override
   public void destroy()
   {

   }
}
