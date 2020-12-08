package us.ihmc.humanoidBehaviors.exploreArea;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.*;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
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
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public enum ExploreAreaBehaviorState
   {
      Stop, LookAround, Perceive, GrabPlanarRegions, DetermineNextLocations, LookAndStep, TurnInPlace
   }

   private final BehaviorHelper helper;
   private final RemoteSyncedRobotModel syncedRobot;
   private final Messager messager;
   private final StatusLogger statusLogger;
   private final RemoteHumanoidRobotInterface robotInterface;

   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();
   private ExploreAreaBehaviorState currentState = LookAround;
   private final Map<ExploreAreaBehaviorState, BehaviorTreeNode> stateToNodeMap = new HashMap<>();
   private final Stopwatch stateTimeStopwatch = new Stopwatch().start();
   private final StopNode stop;
   private final RestOfStatesNode restOfStatesNode;
   private final PausablePeriodicThread mainThread;

   private final AtomicReference<Boolean> explore;

   private TypedNotification<WalkingStatusMessage> walkingCompleted;
   private final Notification lookAndStepReachedGoal;

   public ExploreAreaBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      messager = helper.getManagedMessager();
      statusLogger = helper.getOrCreateStatusLogger();
      robotInterface = helper.getOrCreateRobotInterface();
      syncedRobot = robotInterface.newSyncedRobot();

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

   class StopNode implements BehaviorTreeNode
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

   class RestOfStatesNode implements BehaviorTreeNode
   {
      private final ExploreAreaLookAroundNode lookAround;
      private final ExploreAreaDetermineNextLocationsNode determineNextLocations;
      private final LookAndStepNode lookAndStep;
      private final TurnInPlaceNode turnInPlace;

      public RestOfStatesNode()
      {
         lookAround = new ExploreAreaLookAroundNode(TICK_PERIOD, parameters, helper);
         determineNextLocations = new ExploreAreaDetermineNextLocationsNode(TICK_PERIOD,
                                                                                parameters,
                                                                                helper,
                                                                                lookAround::getConcatenatedMap,
                                                                                lookAround::getConcatenatedMapBoundingBox,
                                                                                lookAround::getPointsObservedFrom);

         stateToNodeMap.put(DetermineNextLocations, determineNextLocations);
         lookAndStep = new LookAndStepNode(determineNextLocations::getBestBodyPath, determineNextLocations::getExploredGoalPosesSoFar);
         stateToNodeMap.put(LookAndStep, lookAndStep);
         turnInPlace = new TurnInPlaceNode();
         stateToNodeMap.put(TurnInPlace, turnInPlace);
      }

      public BehaviorTreeNodeStatus tick()
      {
         ExploreAreaBehaviorState previousState = currentState;

         helper.publishToUI(CurrentState, currentState);
         stateToNodeMap.get(currentState).tick();

         switch (currentState)
         {
            case DetermineNextLocations:
               if (!determineNextLocations.isDeterminingNextLocation() && !determineNextLocations.isFailedToFindNextLocation())
                  currentState = LookAndStep;
               else if (!determineNextLocations.isDeterminingNextLocation() && determineNextLocations.isFailedToFindNextLocation())
                  currentState = TurnInPlace;
               break;
            case LookAndStep:
               if (lookAndStep.readyToTransitionFromLookAndStepToTurnInPlace())
                  currentState = TurnInPlace;
               break;
            case TurnInPlace:
               if (turnInPlace.readyToTransitionFromTurnInPlaceToStop())
                  currentState = LookAround;
               break;
         }

         if (currentState != previousState)
         {
            stateTimeStopwatch.reset();
            statusLogger.info("{} -> {}", previousState == null ? null : previousState.name(), currentState == null ? null : currentState.name());

            switch (currentState)
            {
               case LookAndStep:
                  lookAndStep.onEntry();
                  break;
               case TurnInPlace:
                  turnInPlace.onEntry();
                  break;
            }
         }

         return RUNNING;
      }

      class LookAndStepNode implements BehaviorTreeNode
      {
         private final Supplier<List<Pose3DReadOnly>> bestBodyPathSupplier;
         private final Supplier<ArrayList<Pose3D>> exploredGoalPosesSoFarSupplier;

         public LookAndStepNode(Supplier<List<Pose3DReadOnly>> bestBodyPathSupplier, Supplier<ArrayList<Pose3D>> exploredGoalPosesSoFarSupplier)
         {
            this.bestBodyPathSupplier = bestBodyPathSupplier;
            this.exploredGoalPosesSoFarSupplier = exploredGoalPosesSoFarSupplier;
         }

         @Override
         public BehaviorTreeNodeStatus tick()
         {
            return SUCCESS;
         }

         void onEntry()
         {
            List<Pose3DReadOnly> bestBodyPath = bestBodyPathSupplier.get();
            Pose3D goal = new Pose3D(bestBodyPath.get(bestBodyPath.size() - 1));

            exploredGoalPosesSoFarSupplier.get().add(goal);

            statusLogger.info("Walking to {}", StringTools.zUpPoseString(goal));
            helper.publishToUI(WalkingToPose, goal);

            messager.submitMessage(LookAndStepBehaviorAPI.OperatorReviewEnabled, false);
            helper.publishToUI(LookAndStepBehaviorAPI.BodyPathInput, bestBodyPath.stream().map(Pose3D::new).collect(Collectors.toList()));
            lookAndStepReachedGoal.poll();
         }

         private boolean readyToTransitionFromLookAndStepToTurnInPlace()
         {
            return lookAndStepReachedGoal.poll();
         }
      }

      class TurnInPlaceNode implements BehaviorTreeNode
      {
         @Override
         public BehaviorTreeNodeStatus tick()
         {
            return SUCCESS;
         }

         private void onEntry()
         {
            ArrayList<Pose3D> posesFromThePreviousStep = new ArrayList<>();

            RobotSide supportSide = RobotSide.RIGHT;
            RobotSide initialSupportSide = supportSide;
            double rotationPerStepOutside = Math.PI / 4.0;
            double rotationPerStepInside = Math.PI / 8.0;

            Quaternion orientation = new Quaternion();
            orientation.setYawPitchRoll(rotationPerStepOutside, 0.0, 0.0);
            Point3D translation = new Point3D(0.0, supportSide.negateIfLeftSide(0.2), 0.0);
            posesFromThePreviousStep.add(new Pose3D(translation, orientation));

            supportSide = supportSide.getOppositeSide();
            orientation = new Quaternion();
            orientation.setYawPitchRoll(rotationPerStepInside, 0.0, 0.0);
            translation = new Point3D(0.0, supportSide.negateIfLeftSide(0.2), 0.0);
            posesFromThePreviousStep.add(new Pose3D(translation, orientation));

            supportSide = supportSide.getOppositeSide();
            orientation = new Quaternion();
            orientation.setYawPitchRoll(rotationPerStepOutside, 0.0, 0.0);
            translation = new Point3D(0.0, supportSide.negateIfLeftSide(0.2), 0.0);
            posesFromThePreviousStep.add(new Pose3D(translation, orientation));

            supportSide = supportSide.getOppositeSide();
            orientation = new Quaternion();
            translation = new Point3D(0.0, supportSide.negateIfLeftSide(0.2), 0.0);
            posesFromThePreviousStep.add(new Pose3D(translation, orientation));

            requestWalk(initialSupportSide, posesFromThePreviousStep);
         }

         private boolean readyToTransitionFromTurnInPlaceToStop()
         {
            walkingCompleted.poll();
            return (walkingCompleted.hasValue());
            //      return ((timeInState > 0.1) && (!behaviorHelper.isRobotWalking()));
         }

         private void requestWalk(RobotSide supportSide, ArrayList<Pose3D> posesFromThePreviousStep)
         {
            RobotSide swingSide = supportSide.getOppositeSide();
            FootstepDataListMessage footstepDataListMessageToTurnInPlace = new FootstepDataListMessage();
            us.ihmc.idl.IDLSequence.Object<FootstepDataMessage> footstepDataList = footstepDataListMessageToTurnInPlace.getFootstepDataList();
            syncedRobot.update();
            ReferenceFrame supportFootFrame = syncedRobot.getReferenceFrames().getSoleFrame(supportSide);

            for (Pose3D pose : posesFromThePreviousStep)
            {
               FramePose3D nextStepFramePose = new FramePose3D(supportFootFrame, pose);
               PoseReferenceFrame nextStepFrame = new PoseReferenceFrame("nextStepFrame", nextStepFramePose);

               nextStepFramePose.changeFrame(worldFrame);

               FootstepDataMessage footstepMessage = footstepDataList.add();
               FootstepDataMessage footstep = HumanoidMessageTools.createFootstepDataMessage(swingSide, nextStepFramePose);
               footstepMessage.set(footstep);

               supportSide = supportSide.getOppositeSide();
               swingSide = swingSide.getOppositeSide();
               supportFootFrame = nextStepFrame;
            }

            walkingCompleted = robotInterface.requestWalk(footstepDataListMessageToTurnInPlace);
         }
      }
   }
}
