package us.ihmc.humanoidBehaviors.exploreArea;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMResult;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMergerListener;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.tools.thread.TypedNotification;

public class ExploreAreaBehavior
{
   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final List<Double> chestYawsForLookingAround = Arrays.asList(-40.0, 0.0, 40.0); //, 40.0); //-10.0, 0.0); //Arrays.asList(-10.0, -20.0, -30.0, 0.0);
   private final List<Point3D> pointsObservedFrom = new ArrayList<Point3D>();

   private final double turnChestTrajectoryDuration = 1.0;
   private final double perceiveDuration = 14.0;

   private int chestYawForLookingAroundIndex = 0;

   public enum ExploreAreaBehaviorState
   {
      Stop, LookAround, Perceive, GrabPlanarRegions, DetermineNextLocations, Plan, WalkToNextLocation
   }

   private final BehaviorHelper behaviorHelper;

   private final Messager messager;
   private final StateMachine<ExploreAreaBehaviorState, State> stateMachine;

   private final AtomicReference<Boolean> explore;

   private TypedNotification<RemoteFootstepPlannerResult> footstepPlanResultNotification;
   private TypedNotification<WalkingStatusMessage> walkingCompleted;

   private PlanarRegionsList concatenatedMap;
   private BoundingBox3D concatenatedMapBoundingBox;

   private final NavigableRegionsManager navigableRegionsManager;

   private final BoundingBox2D maximumExplorationArea = new BoundingBox2D(-6.0, -8.0, 0.0, 8.0);

   private double minimumDistanceBetweenObservationPoints = 2.0;
   private double minDistanceToWalkIfPossible = 3.0;

   public ExploreAreaBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel)
   {
      this.behaviorHelper = behaviorHelper;
      this.messager = messager;

      explore = messager.createInput(ExploreAreaBehaviorAPI.ExploreArea, false);
      navigableRegionsManager = new NavigableRegionsManager();

      LogTools.debug("Initializing patrol behavior");

      EnumBasedStateMachineFactory<ExploreAreaBehaviorState> factory = new EnumBasedStateMachineFactory<>(ExploreAreaBehaviorState.class);
      factory.setOnEntry(ExploreAreaBehaviorState.Stop, this::onStopStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.Stop, this::doStopStateAction);
      factory.addTransition(ExploreAreaBehaviorState.Stop, ExploreAreaBehaviorState.LookAround, this::readyToTransitionFromStopToLookAround);

      factory.setOnEntry(ExploreAreaBehaviorState.LookAround, this::onLookAroundStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.LookAround, this::doLookAroundStateAction);
      factory.addTransition(ExploreAreaBehaviorState.LookAround, ExploreAreaBehaviorState.Perceive, this::readyToTransitionFromLookAroundToPerceive);

      factory.setOnEntry(ExploreAreaBehaviorState.Perceive, this::onPerceiveStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.Perceive, this::doPerceiveStateAction);
      factory.addTransition(ExploreAreaBehaviorState.Perceive, ExploreAreaBehaviorState.GrabPlanarRegions,
                            this::readyToTransitionFromPerceiveToGrabPlanarRegions);

      factory.setOnEntry(ExploreAreaBehaviorState.GrabPlanarRegions, this::onGrabPlanarRegionsStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.GrabPlanarRegions, this::doGrabPlanarRegionsStateAction);
      factory.addTransition(ExploreAreaBehaviorState.GrabPlanarRegions, ExploreAreaBehaviorState.LookAround,
                            this::readyToTransitionFromGrabPlanarRegionsToLookAround);
      factory.addTransition(ExploreAreaBehaviorState.GrabPlanarRegions, ExploreAreaBehaviorState.DetermineNextLocations,
                            this::readyToTransitionFromGrabPlanarRegionsToDetermineNextLocations);

      factory.setOnEntry(ExploreAreaBehaviorState.DetermineNextLocations, this::onDetermineNextLocationsStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.DetermineNextLocations, this::doDetermineNextLocationsStateAction);
      factory.addTransition(ExploreAreaBehaviorState.DetermineNextLocations, ExploreAreaBehaviorState.Plan,
                            this::readyToTransitionFromDetermineNextLocationsToPlan);

      factory.setOnEntry(ExploreAreaBehaviorState.Plan, this::onPlanStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.Plan, this::doPlanStateAction);
      factory.addTransition(ExploreAreaBehaviorState.Plan, ExploreAreaBehaviorState.WalkToNextLocation, this::readyToTransitionFromPlanToWalkToNextLocation);
      factory.addTransition(ExploreAreaBehaviorState.Plan, ExploreAreaBehaviorState.Plan, this::readyToTransitionFromPlanToPlan);

      factory.setOnEntry(ExploreAreaBehaviorState.WalkToNextLocation, this::onWalkToNextLocationStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.WalkToNextLocation, this::doWalkToNextLocationStateAction);
      factory.addTransition(ExploreAreaBehaviorState.WalkToNextLocation, ExploreAreaBehaviorState.Stop, this::readyToTransitionFromWalkToNextLocationToStop);

      // Go to stop from any state when no longer exploring.
      factory.addTransition(ExploreAreaBehaviorState.LookAround, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.Perceive, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.GrabPlanarRegions, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.DetermineNextLocations, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.Plan, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.WalkToNextLocation, ExploreAreaBehaviorState.Stop, this::noLongerExploring);

      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(ExploreAreaBehaviorState.Stop);

      ExceptionHandlingThreadScheduler exploreAreaThread = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                                DefaultExceptionHandler.PRINT_STACKTRACE, 5);
      exploreAreaThread.schedule(this::runExploreAreaThread, 5, TimeUnit.MILLISECONDS);
   }

   private boolean noLongerExploring(double timeInState)
   {
      return !explore.get();
   }

   private void runExploreAreaThread()
   {
      stateMachine.doActionAndTransition();
   }

   private void onStopStateEntry()
   {
      chestYawForLookingAroundIndex = 0;
      behaviorHelper.pauseWalking();
   }

   private void doStopStateAction(double timeInState)
   {
   }

   private boolean readyToTransitionFromStopToLookAround(double timeInState)
   {
      return explore.get();
   }

   private void onLookAroundStateEntry()
   {
      double chestYaw = Math.toRadians(chestYawsForLookingAround.get(chestYawForLookingAroundIndex++));
      turnChest(chestYaw, turnChestTrajectoryDuration);
   }

   private void doLookAroundStateAction(double timeInState)
   {
   }

   private boolean readyToTransitionFromLookAroundToPerceive(double timeInState)
   {
      return (timeInState > 9.0 * turnChestTrajectoryDuration);
   }

   private void onPerceiveStateEntry()
   {
      LogTools.info("Entering perceive state. Clearing LIDAR");
      behaviorHelper.clearREA();
   }

   private void rememberObservationPoint()
   {
      //TODO: Remember the LIDAR pointing at transform instead of just where the robot was at. But how to get that frame?
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
      MovingReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FramePoint3D midFeetLocation = new FramePoint3D(midFeetZUpFrame);
      midFeetLocation.changeFrame(worldFrame);

      messager.submitMessage(ExploreAreaBehaviorAPI.ObservationPosition, new Point3D(midFeetLocation));

      this.pointsObservedFrom.add(new Point3D(midFeetLocation));
   }

   private void doPerceiveStateAction(double timeInState)
   {
   }

   private boolean readyToTransitionFromPerceiveToGrabPlanarRegions(double timeInState)
   {
      return (timeInState > perceiveDuration);
   }

   private void onGrabPlanarRegionsStateEntry()
   {
      rememberObservationPoint();
      PlanarRegionsList latestPlanarRegionsList = behaviorHelper.getLatestPlanarRegionList();

      addNewPlanarRegionsToTheMap(latestPlanarRegionsList);
   }

   private ConcaveHullMergerListener listener = null;
//   private ConcaveHullMergerListener listener = new ConcaveHullMergerListener();

   private void addNewPlanarRegionsToTheMap(PlanarRegionsList latestPlanarRegionsList)
   {
      messager.submitMessage(ExploreAreaBehaviorAPI.ClearPlanarRegions, true);

      if (concatenatedMap == null)
      {
         concatenatedMap = latestPlanarRegionsList;
      }
      else
      {
         PlanarRegionSLAMParameters slamParameters = new PlanarRegionSLAMParameters();

         slamParameters.setBoundingBoxHeight(0.03);
         slamParameters.setIterationsForMatching(10);
         slamParameters.setDampedLeastSquaresLambda(1.0);

         PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(concatenatedMap, latestPlanarRegionsList, slamParameters, listener);

         concatenatedMap = slamResult.getMergedMap();
         RigidBodyTransform transformFromIncomingToMap = slamResult.getTransformFromIncomingToMap();
         LogTools.info("SLAM transformFromIncomingToMap = \n " + transformFromIncomingToMap);
      }

      computeMapBoundingBox3D();

      LogTools.info("concatenatedMap has " + concatenatedMap.getNumberOfPlanarRegions() + " planar Regions");
      LogTools.info("boundingBox = " + concatenatedMapBoundingBox);

      List<PlanarRegion> planarRegionsAsList = concatenatedMap.getPlanarRegionsAsList();

      int index = 0;
      for (PlanarRegion planarRegion : planarRegionsAsList)
      {
         messager.submitMessage(ExploreAreaBehaviorAPI.AddPlanarRegionToMap,
                                TemporaryPlanarRegionMessage.convertToTemporaryPlanarRegionMessage(planarRegion, index));

         List<ConvexPolygon2D> convexPolygons = planarRegion.getConvexPolygons();
         for (ConvexPolygon2D polygon : convexPolygons)
         {
            messager.submitMessage(ExploreAreaBehaviorAPI.AddPolygonToPlanarRegion,
                                   TemporaryConvexPolygon2DMessage.convertToTemporaryConvexPolygon2DMessage(polygon, index));
         }

         index++;
      }

      messager.submitMessage(ExploreAreaBehaviorAPI.DrawMap, true);

      // Send it to the GUI for a viz...
      //         PlanarRegionsListMessage concatenatedMapMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(concatenatedMap);
      //         messager.submitMessage(ExploreAreaBehavior.ExploreAreaBehaviorAPI.ConcatenatedMap, concatenatedMapMessage);

      // Find a point that has not been observed, but is close to a point that can be walked to, in order to observe it...
   }

   private void computeMapBoundingBox3D()
   {
      List<PlanarRegion> mapPlanarRegions = concatenatedMap.getPlanarRegionsAsList();
      concatenatedMapBoundingBox = null;

      for (PlanarRegion planarRegion : mapPlanarRegions)
      {
         BoundingBox3D boundingBox = planarRegion.getBoundingBox3dInWorldCopy();
         if (concatenatedMapBoundingBox == null)
         {
            concatenatedMapBoundingBox = boundingBox;
         }
         else
         {
            concatenatedMapBoundingBox = BoundingBox3D.union(concatenatedMapBoundingBox, boundingBox);
         }
      }
   }

   private void doGrabPlanarRegionsStateAction(double timeInState)
   {
   }

   private boolean readyToTransitionFromGrabPlanarRegionsToLookAround(double timeInState)
   {
      return (chestYawForLookingAroundIndex < chestYawsForLookingAround.size()); //(timeInState > 1.0);
   }

   private boolean readyToTransitionFromGrabPlanarRegionsToDetermineNextLocations(double timeInState)
   {
      return (chestYawForLookingAroundIndex >= chestYawsForLookingAround.size());
   }

   private ArrayList<FramePose3D> desiredFramePoses;
   private boolean determinedNextLocations = false;

   private void onDetermineNextLocationsStateEntry()
   {
      behaviorHelper.requestChestGoHome(turnChestTrajectoryDuration);

      desiredFramePoses = null;
      determinedNextLocations = false;

      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
      FramePose3DReadOnly midFeetZUpPose = behaviorHelper.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);

      determineNextPlacesToWalkTo(referenceFrames);
      determinedNextLocations = true;
   }

   private void determineNextPlacesToWalkTo(HumanoidReferenceFrames referenceFrames)
   {
      MovingReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FramePoint3D midFeetPosition = new FramePoint3D(midFeetZUpFrame);
      midFeetPosition.changeFrame(worldFrame);

      ArrayList<Point3DReadOnly> potentialPoints = new ArrayList<>();

      // Do a grid over the bounding box to find potential places to step.
      double xSteps = 0.5;
      double ySteps = 0.5;

      double minimumX = Math.max(maximumExplorationArea.getMinX(), concatenatedMapBoundingBox.getMinX());
      double minimumY = Math.max(maximumExplorationArea.getMinY(), concatenatedMapBoundingBox.getMinY());
      double maximumX = Math.min(maximumExplorationArea.getMaxX(), concatenatedMapBoundingBox.getMaxX());
      double maximumY = Math.min(maximumExplorationArea.getMaxY(), concatenatedMapBoundingBox.getMaxY());

      for (double x = minimumX + xSteps / 2.0; x <= maximumX; x = x + xSteps)
      {
         for (double y = minimumY + ySteps / 2.0; y <= maximumY; y = y + ySteps)
         {
            Point3DReadOnly projectedPoint = PlanarRegionTools.projectPointToPlanesVertically(new Point3D(x, y, 0.0), concatenatedMap);
            if (projectedPoint == null)
               continue;

            if (pointIsTooCloseToPreviousObservationPoint(projectedPoint))
               continue;

            potentialPoints.add(projectedPoint);
         }
      }

      LogTools.info("Found " + potentialPoints.size() + " potential Points on the grid.");

      // Compute distances to each.

      HashMap<Point3DReadOnly, Double> distancesFromStart = new HashMap<>();
      for (Point3DReadOnly testGoal : potentialPoints)
      {
         distancesFromStart.put(testGoal, midFeetPosition.distanceXY(testGoal));
      }

      sortBasedOnBestDistances(potentialPoints, distancesFromStart, minDistanceToWalkIfPossible);

      LogTools.info("Sorted the points based on best distances. Now looking for body paths to those potential goal locations.");

      long startTime = System.nanoTime();

      ArrayList<Point3DReadOnly> feasibleGoalPoints = new ArrayList<>();
      HashMap<Point3DReadOnly, List<Point3DReadOnly>> potentialBodyPaths = new HashMap<>();

      navigableRegionsManager.setPlanarRegions(concatenatedMap.getPlanarRegionsAsList());

      int maxNumberOfFeasiblePointsToLookFor = 30;
      int numberConsidered = 0;

      for (Point3DReadOnly testGoal : potentialPoints)
      {
         //         LogTools.info("Looking for body path to " + testGoal);
         List<Point3DReadOnly> bodyPath = navigableRegionsManager.calculateBodyPath(midFeetPosition, testGoal);
         numberConsidered++;

         if (bodyPath != null)
         {
            //            LogTools.info("Found body path to " + testGoal);

            feasibleGoalPoints.add(testGoal);
            potentialBodyPaths.put(testGoal, bodyPath);
            distancesFromStart.put(testGoal, midFeetPosition.distanceXY(testGoal));

            if (feasibleGoalPoints.size() >= maxNumberOfFeasiblePointsToLookFor)
               break;
         }
      }

      long endTime = System.nanoTime();
      long duration = (endTime - startTime);
      double durationSeconds = ((double) duration) / 1.0e9;
      double durationPer = durationSeconds / ((double) numberConsidered);

      LogTools.info("Found " + feasibleGoalPoints.size() + " feasible Points that have body paths to. Took " + durationSeconds
            + " seconds to find the body paths, or " + durationPer + " seconds Per attempt.");

      desiredFramePoses = new ArrayList<>();

      if (feasibleGoalPoints.isEmpty())
      {
         LogTools.info("Couldn't find a place to walk to. Just stepping in place.");

         FramePoint3D desiredLocation = new FramePoint3D(midFeetZUpFrame, 0.0, 0.0, 0.0);

         FramePose3D desiredFramePose = new FramePose3D(midFeetZUpFrame);
         desiredFramePose.setPosition(desiredLocation);

         desiredFramePose.changeFrame(worldFrame);
         desiredFramePoses.add(desiredFramePose);
         return;
      }

      Point3DReadOnly bestGoalPoint = feasibleGoalPoints.get(0);
      double bestDistance = distancesFromStart.get(bestGoalPoint);
      List<Point3DReadOnly> bestBodyPath = potentialBodyPaths.get(bestGoalPoint);

      LogTools.info("Found bestGoalPoint = " + bestGoalPoint + ", bestDistance = " + bestDistance);

      for (Point3DReadOnly goalPoint : feasibleGoalPoints)
      {
         FrameVector3D startToGoal = new FrameVector3D();
         startToGoal.sub(goalPoint, midFeetPosition);
         double yaw = Math.atan2(startToGoal.getY(), startToGoal.getX());

         FramePose3D desiredFramePose = new FramePose3D(worldFrame);
         desiredFramePose.setPosition(goalPoint);
         desiredFramePose.setOrientationYawPitchRoll(yaw, 0.0, 0.0);
         desiredFramePoses.add(desiredFramePose);
      }

   }

   private boolean pointIsTooCloseToPreviousObservationPoint(Point3DReadOnly pointToCheck)
   {
      for (Point3D observationPoint : pointsObservedFrom)
      {
         if (pointToCheck.distanceXY(observationPoint) < minimumDistanceBetweenObservationPoints)
         {
            return true;
         }
      }
      return false;
   }

   private void sortBasedOnBestDistances(ArrayList<Point3DReadOnly> potentialPoints, HashMap<Point3DReadOnly, Double> distancesFromStart,
                                         double minDistanceIfPossible)
   {
      Comparator<Point3DReadOnly> comparator = new Comparator<Point3DReadOnly>()
      {
         @Override
         public int compare(Point3DReadOnly goalOne, Point3DReadOnly goalTwo)
         {
            if (goalOne == goalTwo)
               return 0;

            double distanceOne = distancesFromStart.get(goalOne);
            double distanceTwo = distancesFromStart.get(goalTwo);

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
         }
      };

      // Sort them by best distances
      Collections.sort(potentialPoints, comparator);
   }

   private void doDetermineNextLocationsStateAction(double timeInState)
   {
   }

   private boolean readyToTransitionFromDetermineNextLocationsToPlan(double timeInState)
   {
      return determinedNextLocations;
   }

   private boolean plannerFinished = false;
   private boolean foundValidPlan = false;
   private FootstepPlan footstepPlan = null;
   private FootstepDataListMessage footstepDataListMessageFromPlan = null;
   private PlanarRegionsList planarRegionsListFromPlan = null;

   private void onPlanStateEntry()
   {
      resetFootstepPlanning();

      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
      FramePose3DReadOnly midFeetZUpPose = behaviorHelper.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);

      if (!desiredFramePoses.isEmpty())
      {
         FramePose3D goal = desiredFramePoses.get(0);
         desiredFramePoses.remove(0);

         LogTools.info("Planning to " + goal);
         footstepPlanResultNotification = behaviorHelper.requestPlan(midFeetZUpPose, goal, concatenatedMap);
      }
      else
      {
         LogTools.info("Out of places to plan too!!!! We're lost!!");
      }
   }

   private void doPlanStateAction(double timeInState)
   {
      plannerFinished = footstepPlanResultNotification.poll();

      if (plannerFinished)
      {
         RemoteFootstepPlannerResult plannerResult = footstepPlanResultNotification.peek();
         FootstepPlanningResult planResult = plannerResult.getResult();

         LogTools.info("planResult = " + planResult);

         if (planResult.validForExecution())
         {
            foundValidPlan = true;
            footstepPlan = plannerResult.getFootstepPlan();
            footstepDataListMessageFromPlan = plannerResult.getFootstepDataListMessage();
            planarRegionsListFromPlan = plannerResult.getPlanarRegionsList();

            reduceAndSendFootstepsForVisualization(footstepPlan);
         }
         else
         {
            foundValidPlan = false;
         }
      }
   }

   private void resetFootstepPlanning()
   {
      plannerFinished = false;
      foundValidPlan = false;
      footstepPlan = null;
      footstepDataListMessageFromPlan = null;
      planarRegionsListFromPlan = null;
      behaviorHelper.abortPlanning();
   }

   private boolean readyToTransitionFromPlanToPlan(double timeInState)
   {
      return (plannerFinished && !foundValidPlan);
   }

   private boolean readyToTransitionFromPlanToWalkToNextLocation(double timeInState)
   {
      return (foundValidPlan == true);
   }

   private void onWalkToNextLocationStateEntry()
   {
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
      boolean swingOverPlanarRegions = true;

      walkingCompleted = behaviorHelper.requestWalk(footstepDataListMessageFromPlan, referenceFrames, swingOverPlanarRegions, planarRegionsListFromPlan);
   }

   private void doWalkToNextLocationStateAction(double timeInState)
   {
      walkingCompleted.poll();
   }

   private boolean readyToTransitionFromWalkToNextLocationToStop(double timeInState)
   {
      return (walkingCompleted.hasNext());
      //      return ((timeInState > 0.1) && (!behaviorHelper.isRobotWalking()));
   }

   public void turnChest(double chestYaw, double trajectoryTime)
   {
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FrameQuaternion chestOrientation = new FrameQuaternion(midFeetZUpFrame, chestYaw, 0.0, 0.0);
      chestOrientation.changeFrame(worldFrame);
      behaviorHelper.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, referenceFrames.getPelvisZUpFrame());

      behaviorHelper.requestPelvisGoHome(trajectoryTime);
   }

   //TODO: Hijacking PatrolBehavior Viz here. Should not be doing that. Should have some common vizzes for things like this that are shared.
   private void reduceAndSendFootstepsForVisualization(FootstepPlan footstepPlan)
   {
      ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++) // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getSoleFramePose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
      messager.submitMessage(PatrolBehaviorAPI.CurrentFootstepPlan, footstepLocations);
   }

   public static class ExploreAreaBehaviorAPI
   {
      private static final MessagerAPIFactory apiFactory = new MessagerAPIFactory();
      private static final Category RootCategory = apiFactory.createRootCategory("ExploreAreaBehavior");
      private static final CategoryTheme ExploreAreaTheme = apiFactory.createCategoryTheme("ExploreArea");
      private static final Category ExploreAreaCategory = RootCategory.child(ExploreAreaTheme);

      public static final Topic<Boolean> ExploreArea = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("ExploreArea"));
      public static final Topic<PlanarRegionsListMessage> ConcatenatedMap = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("ConcatenatedMap"));
      public static final Topic<Point3D> ObservationPosition = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("ObservationPosition"));
      public static final Topic<Boolean> DrawMap = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("DrawMap"));
      public static final Topic<Boolean> ClearPlanarRegions = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("ClearPlanarRegions"));
      public static final Topic<TemporaryPlanarRegionMessage> AddPlanarRegionToMap = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("AddPlanarRegionToMap"));
      public static final Topic<TemporaryConvexPolygon2DMessage> AddPolygonToPlanarRegion = ExploreAreaCategory.topic(apiFactory.createTypedTopicTheme("AddPolygonToPlanarRegion"));

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }

}
