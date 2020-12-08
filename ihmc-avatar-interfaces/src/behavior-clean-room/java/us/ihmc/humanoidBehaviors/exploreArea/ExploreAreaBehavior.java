package us.ihmc.humanoidBehaviors.exploreArea;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
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
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
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

   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final List<Point3D> pointsObservedFrom = new ArrayList<>();
   private final RemoteSyncedRobotModel syncedRobot;
   private final Messager messager;

   private final BoundingBox3D maximumExplorationArea = new BoundingBox3D(new Point3D(-4.0, -7.0, -1.0), new Point3D(8.0, 5.0, 2.0));

   private double exploreGridXSteps = 0.5;
   private double exploreGridYSteps = 0.5;
   private int maxNumberOfFeasiblePointsToLookFor = 10; //30;

   public enum ExploreAreaBehaviorState
   {
      Stop, LookAround, Perceive, GrabPlanarRegions, DetermineNextLocations, LookAndStep, TurnInPlace
   }

   private final BehaviorHelper helper;
   private final StatusLogger statusLogger;
   private final RemoteHumanoidRobotInterface robotInterface;
   private final RemoteREAInterface rea;

   private ExploreAreaBehaviorState currentState = LookAround;
   private final Map<ExploreAreaBehaviorState, BehaviorTreeNode> stateToNodeMap = new HashMap<>();
   private final Stopwatch stateTimeStopwatch = new Stopwatch().start();
   private final StopNode stop;
   private final RestOfStatesNode restOfStatesNode;
   private final PausablePeriodicThread mainThread;

   private final boolean useNewGoalDetermination = false;
   private final ExploreAreaLatticePlanner explorationPlanner = new ExploreAreaLatticePlanner(maximumExplorationArea);
   private final double goalX = 6.0;
   private final double goalY = 0.0;

   private final AtomicReference<Boolean> explore;

   private TypedNotification<WalkingStatusMessage> walkingCompleted;
   private final Notification lookAndStepReachedGoal;

   private final ArrayList<FramePose3D> desiredFramePoses = new ArrayList<>();
   private boolean determiningNextLocation = false;
   private boolean failedToFindNextLocation = false;
   private final ArrayList<Pose3D> exploredGoalPosesSoFar = new ArrayList<>();
   private List<Pose3DReadOnly> bestBodyPath;

   private final VisibilityGraphPathPlanner bodyPathPlanner;

   public ExploreAreaBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      messager = helper.getManagedMessager();
      statusLogger = helper.getOrCreateStatusLogger();
      robotInterface = helper.getOrCreateRobotInterface();
      syncedRobot = robotInterface.newSyncedRobot();
      rea = helper.getOrCreateREAInterface();

      explore = helper.createUIInput(ExploreArea, false);
      helper.createUICallback(Parameters, parameters::setAllFromStrings);
      helper.createUICallback(RandomPoseUpdate, this::randomPoseUpdate);
      lookAndStepReachedGoal = helper.createROS2Notification(LookAndStepBehaviorAPI.REACHED_GOAL);
      bodyPathPlanner = helper.getOrCreateBodyPathPlanner();

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

   private void randomPoseUpdate(boolean doRandomPoseUpdate)
   {
      if (doRandomPoseUpdate)
      {
         RigidBodyTransform transform = new RigidBodyTransform();

         //TODO: Make random or allow user to input update on gui.
         //         transform.setTranslation(0.01, -0.01, 0.01);
         //         transform.setTranslation(0.02, -0.02, 0.0);
         //         transform.setTranslation(0.02, -0.02, 0.02);
         transform.getRotation().setToYawOrientation(0.025);

         boolean sendingSlamCorrection = false;
         publishPoseUpdateForStateEstimator(transform, sendingSlamCorrection);
      }
   }

   private void publishPoseUpdateForStateEstimator(RigidBodyTransform transformFromIncomingToMap, boolean sendingSlamCorrection)
   {
      syncedRobot.update();

      FramePose3D framePose = new FramePose3D(syncedRobot.getReferenceFrames().getPelvisFrame());
      framePose.changeFrame(ReferenceFrame.getWorldFrame());
      Pose3D pose3D = new Pose3D(framePose);

      // TODO: Verify which transform or appendTransform to use...

      RigidBodyTransform transform = new RigidBodyTransform(transformFromIncomingToMap);

      if (sendingSlamCorrection)
      {
         pose3D.prependTransform(transform);
      }
      else
      {
         pose3D.appendTransform(transform);
      }

      double confidenceFactor = 1.0;
      helper.getOrCreateRobotInterface().publishPose(pose3D, confidenceFactor, syncedRobot.getTimestamp());
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
      private final ExploreAreaBehaviorLookAroundNode lookAround;
      private final DetermineNextLocationsNode determineNextLocations;
      private final LookAndStepNode lookAndStep;
      private final TurnInPlaceNode turnInPlace;

      public RestOfStatesNode()
      {
         lookAround = new ExploreAreaBehaviorLookAroundNode(TICK_PERIOD, parameters, helper);

         determineNextLocations = new DetermineNextLocationsNode();
         stateToNodeMap.put(DetermineNextLocations, determineNextLocations);
         lookAndStep = new LookAndStepNode();
         stateToNodeMap.put(LookAndStep, lookAndStep);
         turnInPlace = new TurnInPlaceNode();
         stateToNodeMap.put(TurnInPlace, turnInPlace);
      }

      public BehaviorTreeNodeStatus tick()
      {
         ExploreAreaBehaviorState previousState = currentState;
         double timeInState = stateTimeStopwatch.totalElapsed();

         helper.publishToUI(CurrentState, currentState);
         stateToNodeMap.get(currentState).tick();

         switch (currentState)
         {
            case DetermineNextLocations:
               if (determineNextLocations.readyToTransitionFromDetermineNextLocationsToLookAndStep())
                  currentState = LookAndStep;
               else if (determineNextLocations.readyToTransitionFromDetermineNextLocationsToTurnInPlace())
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
               case DetermineNextLocations:
                  determineNextLocations.onEntry();
                  break;
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

      class DetermineNextLocationsNode implements BehaviorTreeNode
      {
         @Override
         public BehaviorTreeNodeStatus tick()
         {
            return SUCCESS;
         }

         void onEntry()
         {
            robotInterface.requestChestGoHome(parameters.get(ExploreAreaBehaviorParameters.turnChestTrajectoryDuration));

            desiredFramePoses = null;
            determinedNextLocations = false;

            syncedRobot.update();
            determineNextPlacesToWalkTo(syncedRobot);
            determinedNextLocations = true;
         }

         boolean readyToTransitionFromDetermineNextLocationsToTurnInPlace()
      {
         return !determiningNextLocation && failedToFindNextLocation;
      }

      boolean readyToTransitionFromDetermineNextLocationsToLookAndStep()
      {
         return !determiningNextLocation && !failedToFindNextLocation;
      }

      private void determineNextPlacesToWalkTo(RemoteSyncedRobotModel syncedRobot)
      {
         HumanoidReferenceFrames referenceFrames = syncedRobot.getReferenceFrames();
         MovingReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
         FramePoint3D midFeetPosition = new FramePoint3D(midFeetZUpFrame);
         midFeetPosition.changeFrame(worldFrame);

         if (useNewGoalDetermination)
         {
            explorationPlanner.processRegions(concatenatedMap);
            List<Point3D> waypoints = explorationPlanner.doPlan(midFeetPosition.getX(), midFeetPosition.getY(), goalX, goalY, true);

            if (waypoints.isEmpty())
            {
               failedToFindNextLocation = true;
               return;
            }

            int numberOfCells = waypoints.size();
            int maxLookAhead = 7;
            int lookAhead = Math.min(maxLookAhead, numberOfCells - 1);

            bestBodyPath = new ArrayList<>();
            for (int i = 0; i < lookAhead; i++)
            {
               Point3D position = waypoints.get(i);
               Quaternion orientation = new Quaternion();

               if (i != 0)
               {
                  Point3D previousPosition = waypoints.get(i - 1);
                  double yaw = Math.atan2(position.getY() - previousPosition.getY(), position.getX() - previousPosition.getX());
                  orientation.setYawPitchRoll(yaw, 0.0, 0.0);
               }

               bestBodyPath.add(new Pose3D(position, orientation));
            }

            helper.publishToUI(FoundBodyPath, bestBodyPath.stream().map(Pose3D::new).collect(Collectors.toList()));

            Pose3DReadOnly finalBodyPathPoint = bestBodyPath.get(bestBodyPath.size() - 1);
            Point3D goalPoint = new Point3D(finalBodyPathPoint.getX(), finalBodyPathPoint.getY(), 0.0);
            FrameVector3D startToGoal = new FrameVector3D();
            startToGoal.sub(goalPoint, midFeetPosition);

            FramePose3D desiredFramePose = new FramePose3D(worldFrame);
            desiredFramePose.getPosition().set(goalPoint);
            desiredFramePose.getOrientation().setYawPitchRoll(bestBodyPath.get(bestBodyPath.size() - 1).getYaw(), 0.0, 0.0);
            desiredFramePoses.add(desiredFramePose);
         }
         else
         {
            ArrayList<Point3D> potentialPoints = new ArrayList<>();

            // Do a grid over the bounding box to find potential places to step.

            BoundingBox3D intersectionBoundingBox = getBoundingBoxIntersection(maximumExplorationArea, concatenatedMapBoundingBox);

            ArrayList<BoundingBox3D> explorationBoundingBoxes = new ArrayList<>();
            explorationBoundingBoxes.add(maximumExplorationArea);
            explorationBoundingBoxes.add(concatenatedMapBoundingBox);
            explorationBoundingBoxes.add(intersectionBoundingBox);

            helper.publishToUI(ExplorationBoundingBoxes, explorationBoundingBoxes);

            for (double x = intersectionBoundingBox.getMinX() + exploreGridXSteps / 2.0; x <= intersectionBoundingBox.getMaxX(); x = x + exploreGridXSteps)
            {
               for (double y = intersectionBoundingBox.getMinY() + exploreGridYSteps / 2.0; y <= intersectionBoundingBox.getMaxY(); y = y + exploreGridYSteps)
               {
                  Point3D projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(x, y, 0.0), concatenatedMap);
                  if (projectedPoint == null)
                     continue;

                  if (pointIsTooCloseToPreviousObservationPoint(projectedPoint))
                     continue;

                  potentialPoints.add(projectedPoint);
               }
            }

            statusLogger.info("Found " + potentialPoints.size() + " potential Points on the grid.");

            ArrayList<Point3D> potentialPointsToSend = new ArrayList<>();
            potentialPointsToSend.addAll(potentialPoints);
            helper.publishToUI(PotentialPointsToExplore, potentialPointsToSend);

            // Compute distances to each.

            HashMap<Point3DReadOnly, Double> distances = new HashMap<>();
            for (Point3DReadOnly testGoal : potentialPoints)
            {
               double closestDistance = midFeetPosition.distanceXY(testGoal);
               for (Pose3D pose3D : exploredGoalPosesSoFar)
               {
                  double distance = pose3D.getPosition().distance(testGoal);
                  if (distance < closestDistance)
                     closestDistance = distance;
               }

               distances.put(testGoal, closestDistance);
            }

            sortBasedOnBestDistances(potentialPoints, distances, parameters.get(ExploreAreaBehaviorParameters.minDistanceToWalkIfPossible));

            statusLogger.info("Sorted the points based on best distances. Now looking for body paths to those potential goal locations.");

            long startTime = System.nanoTime();

            ArrayList<Point3DReadOnly> feasibleGoalPoints = new ArrayList<>();
            HashMap<Point3DReadOnly, List<Pose3DReadOnly>> potentialBodyPaths = new HashMap<>();

            bodyPathPlanner.setPlanarRegionsList(concatenatedMap);

            int numberConsidered = 0;

            for (Point3D testGoal : potentialPoints)
            {
               //         LogTools.info("Looking for body path to " + testGoal);

               bodyPathPlanner.setGoal(new Pose3D(testGoal, syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getPelvisZUpFrame).getOrientation()));
               bodyPathPlanner.setStanceFootPoses(referenceFrames);
               BodyPathPlanningResult bodyPathPlanningResult = bodyPathPlanner.planWaypointsWithOcclusionHandling();
               List<Pose3DReadOnly> bodyPath = bodyPathPlanner.getWaypoints();
               numberConsidered++;

               if (bodyPathPlanningResult == BodyPathPlanningResult.FOUND_SOLUTION)
               {
                  //            LogTools.info("Found body path to " + testGoal);
                  helper.publishToUI(FoundBodyPath, bodyPath.stream().map(Pose3D::new).collect(Collectors.toList())); // deep copy

                  feasibleGoalPoints.add(testGoal);
                  potentialBodyPaths.put(testGoal, bodyPath);
                  distances.put(testGoal, midFeetPosition.distanceXY(testGoal));

                  if (feasibleGoalPoints.size() >= maxNumberOfFeasiblePointsToLookFor)
                     break;
               }
            }

            long endTime = System.nanoTime();
            long duration = (endTime - startTime);
            double durationSeconds = ((double) duration) / 1.0e9;
            double durationPer = durationSeconds / ((double) numberConsidered);

            statusLogger.info("Found " + feasibleGoalPoints.size() + " feasible Points that have body paths to. Took " + durationSeconds
                              + " seconds to find the body paths, or " + durationPer + " seconds Per attempt.");
            failedToFindNextLocation = feasibleGoalPoints.isEmpty();

            if (feasibleGoalPoints.isEmpty())
            {
               statusLogger.info("Couldn't find a place to walk to. Just stepping in place.");
               FramePoint3D desiredLocation = new FramePoint3D(midFeetZUpFrame, 0.0, 0.0, 0.0);

               FramePose3D desiredFramePose = new FramePose3D(midFeetZUpFrame);
               desiredFramePose.getPosition().set(desiredLocation);

               desiredFramePose.changeFrame(worldFrame);
               desiredFramePoses.add(desiredFramePose);
               return;
            }

            Point3DReadOnly bestGoalPoint = feasibleGoalPoints.get(0);
            double bestDistance = distances.get(bestGoalPoint);
            bestBodyPath = potentialBodyPaths.get(bestGoalPoint);

            statusLogger.info("Found bestGoalPoint = " + bestGoalPoint + ", bestDistance = " + bestDistance);

            for (Point3DReadOnly goalPoint : feasibleGoalPoints)
            {
               FrameVector3D startToGoal = new FrameVector3D();
               startToGoal.sub(goalPoint, midFeetPosition);
               double yaw = Math.atan2(startToGoal.getY(), startToGoal.getX());

               FramePose3D desiredFramePose = new FramePose3D(worldFrame);
               desiredFramePose.getPosition().set(goalPoint);
               desiredFramePose.getOrientation().setYawPitchRoll(yaw, 0.0, 0.0);
               desiredFramePoses.add(desiredFramePose);
            }
         }

         private BoundingBox3D getBoundingBoxIntersection(BoundingBox3D boxOne, BoundingBox3D boxTwo)
         {
            //TODO: There should be BoundingBox2D.intersection() and BoundingBox3D.intersection() methods, same as union();
            double minimumX = Math.max(boxOne.getMinX(), boxTwo.getMinX());
            double minimumY = Math.max(boxOne.getMinY(), boxTwo.getMinY());
            double minimumZ = Math.max(boxOne.getMinZ(), boxTwo.getMinZ());

            double maximumX = Math.min(boxOne.getMaxX(), boxTwo.getMaxX());
            double maximumY = Math.min(boxOne.getMaxY(), boxTwo.getMaxY());
            double maximumZ = Math.min(boxOne.getMaxZ(), boxTwo.getMaxZ());

            return new BoundingBox3D(minimumX, minimumY, minimumZ, maximumX, maximumY, maximumZ);
         }

         private boolean pointIsTooCloseToPreviousObservationPoint(Point3DReadOnly pointToCheck)
         {
            for (Point3D observationPoint : pointsObservedFrom)
            {
               if (pointToCheck.distanceXY(observationPoint) < parameters.get(ExploreAreaBehaviorParameters.minimumDistanceBetweenObservationPoints))
               {
                  return true;
               }
            }
            return false;
         }

         private void sortBasedOnBestDistances(ArrayList<Point3D> potentialPoints, HashMap<Point3DReadOnly, Double> distances, double minDistanceIfPossible)
         {
            Comparator<Point3DReadOnly> comparator = (goalOne, goalTwo) ->
            {
               if (goalOne == goalTwo)
                  return 0;

               double distanceOne = distances.get(goalOne);
               double distanceTwo = distances.get(goalTwo);

               if (distanceOne >= minDistanceIfPossible)
               {
                  if (distanceTwo < minDistanceIfPossible)
                     return 1;
                  if (distanceOne < distanceTwo)
                  {
                     return 1;
                  }
                  else
                  {
                     return -1;
                  }
               }
               else
               {
                  if (distanceTwo >= minDistanceIfPossible)
                  {
                     return -1;
                  }
                  if (distanceTwo < distanceOne)
                  {
                     return -1;
                  }
                  else
                  {
                     return 1;
                  }
               }
            };

            // Sort them by best distances
            Collections.sort(potentialPoints, comparator);
         }
      }

      class LookAndStepNode implements BehaviorTreeNode
      {
         @Override
         public BehaviorTreeNodeStatus tick()
         {
            return SUCCESS;
         }

         void onEntry()
         {
            List<Pose3DReadOnly> bestBodyPath = ExploreAreaBehavior.this.bestBodyPath;
            Pose3D goal = new Pose3D(bestBodyPath.get(bestBodyPath.size() - 1));

            exploredGoalPosesSoFar.add(goal);

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
