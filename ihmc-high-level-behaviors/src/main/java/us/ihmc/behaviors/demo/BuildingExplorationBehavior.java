package us.ihmc.behaviors.demo;

import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
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
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorAPI.*;
import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorMode.*;
import static us.ihmc.behaviors.demo.BuildingExplorationBehaviorTools.NAN_POSE;

public class BuildingExplorationBehavior extends ResettingNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Building Exploration",
                                                                              BuildingExplorationBehavior::new,
                                                                              BuildingExplorationBehaviorAPI.API);
   private final BehaviorHelper helper;
   private final LookAndStepBehavior lookAndStepBehavior;
   private final DoorBehavior doorBehavior;
   private final ROS2SyncedRobotModel syncedRobot;
   private final AtomicReference<Pose3D> goal = new AtomicReference<>(NAN_POSE);
   private final AtomicReference<BuildingExplorationBehaviorMode> mode = new AtomicReference<>(TELEOP);
   private final TraverseStairsBehavior traverseStairsBehavior;
   private final BuildingExplorationBehaviorParameters parameters;
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("CommsRelay", true);
   private String lastTickedThing = "NONE";

   public BuildingExplorationBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      LogTools.info("Constructing");
      parameters = new BuildingExplorationBehaviorParameters();
      syncedRobot = helper.newSyncedRobot();
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      addChild(lookAndStepBehavior);
      doorBehavior = new DoorBehavior(helper);
      doorBehavior.setSyncedRobot(syncedRobot);
      addChild(doorBehavior);
      traverseStairsBehavior = new TraverseStairsBehavior(helper);
      traverseStairsBehavior.setSyncedRobot(syncedRobot);
      addChild(traverseStairsBehavior);
      helper.subscribeViaCallback(Parameters, parameters ->
      {
         helper.getOrCreateStatusLogger().info("Accepting new building exploration parameters");
         this.parameters.setAllFromStrings(parameters);
      });
      helper.subscribeViaCallback(Goal, newGoal ->
      {
         goal.set(newGoal);
      });
      helper.subscribeViaCallback(Mode, newValue ->
      {
         LogTools.info("Received mode: {}", newValue);
         mode.set(newValue);
      });
      helper.getOrCreateControllerStatusTracker().addNotWalkingStateAnymoreCallback(() ->
      {
         mode.set(TELEOP);
         traverseStairsBehavior.reset();
         executor.submit(() -> helper.publish(Mode, mode.get()));
      });
   }

   @Override
   public BehaviorTreeNodeStatus tickInternal()
   {
      syncedRobot.update();

      BehaviorTreeNodeStatus status = BehaviorTreeNodeStatus.RUNNING;
      lastTickedThing = "NONE";
      BuildingExplorationBehaviorMode currentMode = mode.get();
      if (currentMode == AUTO)
      {
         if (!goal.get().containsNaN())
         {
            if (doorBehavior.isDoingBehavior()
            || (doorBehavior.hasSeenDoorRecently() && doorBehavior.getDistanceToDoor() < parameters.getDistanceFromDoorToTransition()))
            {
               status = tickDoor();
            }
            else if (traverseStairsBehavior.isGoing()
                 || (traverseStairsBehavior.hasSeenStairsecently() && traverseStairsBehavior.getDistanceToStairs() < parameters.getDistanceFromDoorToTransition()))
            {
               tickStairs();
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
      else if (currentMode == STAIRS)
      {
         status = tickStairs();
      }

      helper.publish(LastTickedThing, lastTickedThing);

      return status;
   }

   private BehaviorTreeNodeStatus tickLookAndStep()
   {
      if (lookAndStepBehavior.isReset())
         lookAndStepBehavior.acceptGoal(goal.get());
      lastTickedThing = "LOOK_AND_STEP";
      return lookAndStepBehavior.tick();
   }

   private BehaviorTreeNodeStatus tickDoor()
   {
      lastTickedThing = "DOOR";
      return doorBehavior.tick();
   }

   private BehaviorTreeNodeStatus tickStairs()
   {
      lastTickedThing = "STAIRS";
      return traverseStairsBehavior.tick();
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
      traverseStairsBehavior.destroy();
      lookAndStepBehavior.destroy();
      doorBehavior.destroy();
   }
}
