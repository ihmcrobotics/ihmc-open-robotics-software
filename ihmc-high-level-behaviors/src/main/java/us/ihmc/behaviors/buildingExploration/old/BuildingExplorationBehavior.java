package us.ihmc.behaviors.buildingExploration.old;

import std_msgs.msg.dds.UInt16;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.behaviors.door.DoorBehavior;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.behaviors.stairs.TraverseStairsBehavior;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.BehaviorTools;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus;
import us.ihmc.behaviors.behaviorTree.ResettingNode;
import us.ihmc.communication.property.StoredPropertySetMessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.log.LogTools;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;

import java.util.concurrent.atomic.AtomicReference;

import static us.ihmc.behaviors.buildingExploration.old.BuildingExplorationBehaviorAPI.*;
import static us.ihmc.behaviors.buildingExploration.old.BuildingExplorationBehaviorMode.*;

/**
 * This was intended to be an upgrade for the 2019 Atlas building exploration demo but never got used.
 * @deprecated Not supported right now. Being kept for reference or revival.
 */
public class BuildingExplorationBehavior extends ResettingNode implements Destroyable
{
   private final BehaviorHelper helper;
   private final LookAndStepBehavior lookAndStepBehavior;
   private final DoorBehavior doorBehavior;
   private final ROS2SyncedRobotModel syncedRobot;
   private final AtomicReference<Pose3D> goal = new AtomicReference<>(BehaviorTools.createNaNPose());
   private final AtomicReference<BuildingExplorationBehaviorMode> mode = new AtomicReference<>(TELEOP);
   private final TraverseStairsBehavior traverseStairsBehavior;
   private final BuildingExplorationBehaviorParameters parameters;
   private final ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("CommsRelay", true);
   private String lastTickedNode = "NONE";

   public BuildingExplorationBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      LogTools.info("Constructing");
      parameters = new BuildingExplorationBehaviorParameters();
      syncedRobot = helper.newSyncedRobot();
      lookAndStepBehavior = new LookAndStepBehavior(helper);
      getChildren().add(lookAndStepBehavior);
      doorBehavior = new DoorBehavior(helper, syncedRobot);
      getChildren().add(doorBehavior);
      traverseStairsBehavior = new TraverseStairsBehavior(helper);
      traverseStairsBehavior.setSyncedRobot(syncedRobot);
      getChildren().add(traverseStairsBehavior);
      helper.subscribeViaCallback(PARAMETERS.getCommandTopic(), message ->
      {
         helper.getOrCreateStatusLogger().info("Accepting new building exploration parameters");
         StoredPropertySetMessageTools.copyToStoredPropertySet(message, parameters, () -> { });
      });
      helper.subscribeViaCallback(GOAL_COMMAND, newGoal ->
      {
         goal.set(newGoal);
      });
      helper.subscribeViaCallback(MODE, message ->
      {
         BuildingExplorationBehaviorMode newValue = values()[message.getData()];
         LogTools.info("Received mode: {}", newValue);
         mode.set(newValue);
      });
      helper.getOrCreateControllerStatusTracker().addNotWalkingStateAnymoreCallback(() ->
      {
         mode.set(TELEOP);
         traverseStairsBehavior.reset();
         executor.submit(() ->
         {
            UInt16 modeMessage = new UInt16();
            modeMessage.setData(mode.get().ordinal());
            helper.publish(MODE, modeMessage);
         });
      });
   }

   @Override
   public BehaviorTreeNodeStatus determineStatus()
   {
      syncedRobot.update();

      BehaviorTreeNodeStatus status = BehaviorTreeNodeStatus.RUNNING;
      lastTickedNode = "NONE";
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

      helper.publish(LAST_TICKED_NODE, lastTickedNode);

      return status;
   }

   private BehaviorTreeNodeStatus tickLookAndStep()
   {
      if (lookAndStepBehavior.isReset())
         lookAndStepBehavior.acceptGoal(goal.get());
      lastTickedNode = "LOOK_AND_STEP";
      return lookAndStepBehavior.tickAndGetStatus();
   }

   private BehaviorTreeNodeStatus tickDoor()
   {
      lastTickedNode = "DOOR";
      return doorBehavior.tickAndGetStatus();
   }

   private BehaviorTreeNodeStatus tickStairs()
   {
      lastTickedNode = "STAIRS";
      return traverseStairsBehavior.tickAndGetStatus();
   }

   @Override
   public void reset()
   {

   }

   public String getName()
   {
      return "Building Exploration";
   }

   @Override
   public void destroy()
   {
      traverseStairsBehavior.destroy();
      lookAndStepBehavior.destroy();
      doorBehavior.destroy();
   }
}
