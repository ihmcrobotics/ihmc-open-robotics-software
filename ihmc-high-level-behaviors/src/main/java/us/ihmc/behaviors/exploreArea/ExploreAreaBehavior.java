package us.ihmc.behaviors.exploreArea;

import us.ihmc.behaviors.behaviorTree.*;
import us.ihmc.commons.thread.Notification;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.behaviors.tools.BehaviorHelper;
import us.ihmc.behaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.behaviors.tools.interfaces.StatusLogger;
import us.ihmc.tools.Destroyable;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

import static us.ihmc.behaviors.behaviorTree.BehaviorTreeNodeStatus.*;

/**
 * An attempt mostly in simulation to explore a building.
 * @deprecated Not supported right now. Being kept for reference or revival.
 */
public class ExploreAreaBehavior extends FallbackNode implements Destroyable
{
   public static final double TICK_PERIOD = UnitConversions.hertzToSeconds(2);

   public enum ExploreAreaBehaviorState
   {
      Stop, LookRight, LookCenter, LookLeft, Perceive, GrabPlanarRegions, DetermineNextLocations, LookAndStep, TurnInPlace
   }

   private final BehaviorHelper helper;
   private final StatusLogger statusLogger;
   private final RemoteHumanoidRobotInterface robotInterface;

   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();
   private final StopNode stop;
   private final RestOfStatesNode restOfStatesNode;
   private final PausablePeriodicThread mainThread;

   private final AtomicReference<Boolean> explore = null;

   private final Notification lookAndStepReachedGoal;

   public ExploreAreaBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      statusLogger = helper.getOrCreateStatusLogger();
      robotInterface = helper.getOrCreateRobotInterface();

//      explore = helper.subscribeViaReference(ExploreArea, false);
//      helper.subscribeViaCallback(Parameters, parameters::setAllFromStrings);
      lookAndStepReachedGoal = helper.subscribeViaNotification(LookAndStepBehaviorAPI.REACHED_GOAL);

      statusLogger.info("Initializing explore area behavior");

      stop = new StopNode();
      restOfStatesNode = new RestOfStatesNode();

      getChildren().add(stop);
      getChildren().add(restOfStatesNode);

      mainThread = helper.createPausablePeriodicThread(getClass(), TICK_PERIOD, 5, this::tick);
   }

   public void setEnabled(boolean enabled)
   {
      mainThread.setRunning(enabled);
   }

   class StopNode extends LocalOnlyBehaviorTreeNodeExecutor // TODO: Is there a more general reusable robot stop node?
   {
      @Override
      public BehaviorTreeNodeStatus determineStatus()
      {
         if (explore.get())
         {
            return FAILURE;
         }
         else
         {
//            helper.publish(CurrentState, Stop);
            robotInterface.pauseWalking();
            return SUCCESS;
         }
      }
   }

   class RestOfStatesNode extends SequenceNode
   {
      private final ExploreAreaLookAroundNode lookAround;
      private final ExploreAreaDetermineNextLocationsNode determineNextLocations;
      private final LookAndStepNode lookAndStep;
      private final ExploreAreaTurnInPlace turnInPlace;

      public RestOfStatesNode()
      {
         lookAround = new ExploreAreaLookAroundNode(parameters, helper);
         determineNextLocations = new ExploreAreaDetermineNextLocationsNode(parameters,
                                                                            helper,
                                                                            lookAround::getConcatenatedMap,
                                                                            lookAround::getConcatenatedMapBoundingBox,
                                                                            lookAround::getPointsObservedFrom);
         lookAndStep = new LookAndStepNode(determineNextLocations::getBestBodyPath,
                                           determineNextLocations::getExploredGoalPosesSoFar,
                                           determineNextLocations::isFailedToFindNextLocation);
         turnInPlace = new ExploreAreaTurnInPlace(TICK_PERIOD,
                                                  parameters,
                                                  helper,
                                                  determineNextLocations.getExplorationPlanner());

         getChildren().add(lookAround);
         getChildren().add(determineNextLocations);
         getChildren().add(lookAndStep);
         getChildren().add(turnInPlace);
         getChildren().add(new AlwaysSuccessfulAction(lookAround::reset));
      }

      class LookAndStepNode extends AsynchronousActionNode // TODO: Use look and step node directly somehow.
      {
         private final Supplier<List<Pose3DReadOnly>> bestBodyPathSupplier;
         private final Supplier<ArrayList<Pose3D>> exploredGoalPosesSoFarSupplier;
         private final Supplier<Boolean> noWhereToExploreSupplier;

         public LookAndStepNode(Supplier<List<Pose3DReadOnly>> bestBodyPathSupplier,
                                Supplier<ArrayList<Pose3D>> exploredGoalPosesSoFarSupplier,
                                Supplier<Boolean> noWhereToExploreSupplier)
         {
            this.bestBodyPathSupplier = bestBodyPathSupplier;
            this.exploredGoalPosesSoFarSupplier = exploredGoalPosesSoFarSupplier;
            this.noWhereToExploreSupplier = noWhereToExploreSupplier;
         }

         @Override
         public BehaviorTreeNodeStatus doActionInternal()
         {
            if (noWhereToExploreSupplier.get())
               return SUCCESS; // return failure?

//            helper.publish(CurrentState, LookAndStep);

            List<Pose3DReadOnly> bestBodyPath = bestBodyPathSupplier.get();
            Pose3D goal = new Pose3D(bestBodyPath.get(bestBodyPath.size() - 1));

            exploredGoalPosesSoFarSupplier.get().add(goal);

            statusLogger.info("Walking to {}", StringTools.zUpPoseString(goal));
//            helper.publish(WalkingToPose, goal);

            helper.publish(LookAndStepBehaviorAPI.OPERATOR_REVIEW_ENABLED_COMMAND, false);
            helper.publish(LookAndStepBehaviorAPI.BODY_PATH_INPUT, MessageTools.createPoseListMessage(bestBodyPath));
            lookAndStepReachedGoal.poll();
            lookAndStepReachedGoal.blockingPoll();
            return SUCCESS;
         }

         @Override
         public void resetInternal()
         {

         }
      }
   }
}
