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

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.*;
import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorMode.*;
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
   private final AtomicReference<BuildingExplorationBehaviorMode> mode = new AtomicReference<>(AUTO);
   private final TraverseStairsBehavior traverseStairsBehavior;
   private volatile double distanceFromDoorToTransition = 1.8;

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
      helper.subscribeViaCallback(Mode, mode::set);
      helper.subscribeViaCallback(DistanceFromDoorToTransition, distanceFromDoorToTransition -> this.distanceFromDoorToTransition = distanceFromDoorToTransition);
   }

   private void setGoal(Pose3D newGoal)
   {
      goal.set(newGoal);
      helper.publish(GoalForUI, goal.get());
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      syncedRobot.update();

      BehaviorTreeNodeStatus status = BehaviorTreeNodeStatus.RUNNING;
      BuildingExplorationBehaviorMode currentMode = mode.get();
      if (currentMode == AUTO)
      {
         if (!goal.get().containsNaN())
         {
            if (doorBehavior.getDistanceToDoor() < distanceFromDoorToTransition)
            {
               status = tickDoor();
            }
            else
            {
               status = tickLookAndStep();
            }
         }
      }
      else if (currentMode == DOOR)
      {
         status = tickDoor();
      }
      else if (currentMode == LOOK_AND_STEP)
      {
         status = tickLookAndStep();
      }

      return status;
   }

   private BehaviorTreeNodeStatus tickDoor()
   {
      return doorBehavior.tick();
   }

   private BehaviorTreeNodeStatus tickLookAndStep()
   {
      if (lookAndStepBehavior.isReset())
         lookAndStepBehavior.acceptGoal(goal.get());
      return lookAndStepBehavior.tick();
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
