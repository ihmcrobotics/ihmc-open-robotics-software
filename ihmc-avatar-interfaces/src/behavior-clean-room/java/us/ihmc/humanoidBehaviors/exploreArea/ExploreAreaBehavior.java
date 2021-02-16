package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.commons.thread.Notification;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.*;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.messager.Messager;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior.ExploreAreaBehaviorState.*;
import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI.*;
import static us.ihmc.humanoidBehaviors.tools.behaviorTree.BehaviorTreeNodeStatus.*;

public class ExploreAreaBehavior extends FallbackNode implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Explore Area",
                                                                              ExploreAreaBehavior::new,
                                                                              create(),
                                                                              LookAndStepBehavior.DEFINITION);
   public static final double TICK_PERIOD = UnitConversions.hertzToSeconds(2);

   public enum ExploreAreaBehaviorState
   {
      Stop, LookRight, LookCenter, LookLeft, Perceive, GrabPlanarRegions, DetermineNextLocations, LookAndStep, TurnInPlace
   }

   private final BehaviorHelper helper;
   private final Messager messager;
   private final StatusLogger statusLogger;
   private final RemoteHumanoidRobotInterface robotInterface;

   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();
   private final StopNode stop;
   private final RestOfStatesNode restOfStatesNode;
   private final PausablePeriodicThread mainThread;

   private final AtomicReference<Boolean> explore;

   private final Notification lookAndStepReachedGoal;

   public ExploreAreaBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      messager = helper.getManagedMessager();
      statusLogger = helper.getOrCreateStatusLogger();
      robotInterface = helper.getOrCreateRobotInterface();

      explore = helper.createUIInput(ExploreArea, false);
      helper.createUICallback(Parameters, parameters::setAllFromStrings);
      lookAndStepReachedGoal = helper.createROS2Notification(LookAndStepBehaviorAPI.REACHED_GOAL);

      statusLogger.info("Initializing explore area behavior");

      stop = new StopNode();
      restOfStatesNode = new RestOfStatesNode();

      addChild(stop);
      addChild(restOfStatesNode);

      mainThread = helper.createPausablePeriodicThread(getClass(), TICK_PERIOD, 5, this::tick);
   }

   @Override
   public void setEnabled(boolean enabled)
   {
      helper.setCommunicationCallbacksEnabled(enabled);
      mainThread.setRunning(enabled);
   }

   class StopNode implements BehaviorTreeNode // TODO: Is there a more general reusable robot stop node?
   {
      @Override
      public BehaviorTreeNodeStatus tick()
      {
         if (explore.get())
         {
            return FAILURE;
         }
         else
         {
            helper.publishToUI(CurrentState, Stop);
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

         addChild(lookAround);
         addChild(determineNextLocations);
         addChild(lookAndStep);
         addChild(turnInPlace);
         addChild(new AlwaysSuccessfulAction(lookAround::reset));
      }

      class LookAndStepNode extends ParallelNodeBasics // TODO: Use look and step node directly somehow.
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
         public void doAction()
         {
            if (noWhereToExploreSupplier.get())
               return;

            helper.publishToUI(CurrentState, LookAndStep);

            List<Pose3DReadOnly> bestBodyPath = bestBodyPathSupplier.get();
            Pose3D goal = new Pose3D(bestBodyPath.get(bestBodyPath.size() - 1));

            exploredGoalPosesSoFarSupplier.get().add(goal);

            statusLogger.info("Walking to {}", StringTools.zUpPoseString(goal));
            helper.publishToUI(WalkingToPose, goal);

            messager.submitMessage(LookAndStepBehaviorAPI.OperatorReviewEnabled, false);
            helper.publishToUI(LookAndStepBehaviorAPI.BodyPathInput, bestBodyPath.stream().map(Pose3D::new).collect(Collectors.toList()));
            lookAndStepReachedGoal.poll();
            lookAndStepReachedGoal.blockingPoll();
         }
      }
   }
}
