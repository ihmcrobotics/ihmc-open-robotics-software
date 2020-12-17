package us.ihmc.humanoidBehaviors.exploreArea;

import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.behaviorTree.ParallelNodeBasics;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.util.*;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior.*;
import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI.*;
import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI.FoundBodyPath;

public class ExploreAreaDetermineNextLocationsNode extends ParallelNodeBasics
{
   public static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final boolean useNewGoalDetermination = true;

   private final ExploreAreaBehaviorParameters parameters;
   private final BehaviorHelper helper;
   private final VisibilityGraphPathPlanner bodyPathPlanner;
   private final Supplier<List<Point3D>> pointsObservedFromSupplier;
   private final RemoteSyncedRobotModel syncedRobot;
   private final Supplier<PlanarRegionsList> concatenatedMapSupplier;
   private final Supplier<BoundingBox3D> concatenatedMapBoundingBoxSupplier;
   private final StatusLogger statusLogger;

   private final BoundingBox3D maximumExplorationArea = new BoundingBox3D(new Point3D(-3.5, -6.0, -1.0), new Point3D(7.5, 3.0, 2.0));
   private final ExploreAreaLatticePlanner explorationPlanner = new ExploreAreaLatticePlanner(maximumExplorationArea);
   private boolean determiningNextLocation = false;
   private boolean failedToFindNextLocation = false;
   private double exploreGridXSteps = 0.5;
   private double exploreGridYSteps = 0.5;
   private int maxNumberOfFeasiblePointsToLookFor = 10; //30;
   private final ArrayList<FramePose3D> desiredFramePoses = new ArrayList<>();
   private final ArrayList<Pose3D> exploredGoalPosesSoFar = new ArrayList<>();
   private List<Pose3DReadOnly> bestBodyPath;

   private double goalX;
   private double goalY;
   private final int numberOfIterationsBeforeSwitchingGoals = 5;
   private int counter = 0;
   private final Random random = new Random(3242);

//   private final ExploreAreaMapUI exploreAreaMapUI;

   public ExploreAreaDetermineNextLocationsNode(ExploreAreaBehaviorParameters parameters,
                                                BehaviorHelper helper,
                                                Supplier<PlanarRegionsList> concatenatedMapSupplier,
                                                Supplier<BoundingBox3D> concatenatedMapBoundingBoxSupplier,
                                                Supplier<List<Point3D>> pointsObservedFromSupplier)
   {
      this.parameters = parameters;
      this.helper = helper;
      this.concatenatedMapSupplier = concatenatedMapSupplier;
      this.concatenatedMapBoundingBoxSupplier = concatenatedMapBoundingBoxSupplier;
      this.pointsObservedFromSupplier = pointsObservedFromSupplier;

      statusLogger = helper.getOrCreateStatusLogger();
      syncedRobot = helper.getOrCreateRobotInterface().newSyncedRobot();
      bodyPathPlanner = helper.newBodyPathPlanner();

      goalX = maximumExplorationArea.getMaxX();
      goalY = 0.0;

//      exploreAreaMapUI = new ExploreAreaMapUI(explorationPlanner);
   }

   @Override
   public void doAction()
   {
      helper.publishToUI(CurrentState, ExploreAreaBehaviorState.DetermineNextLocations);

      helper.getOrCreateRobotInterface().requestChestGoHome(parameters.getTurnChestTrajectoryDuration());

      syncedRobot.update();
      determineNextPlacesToWalkTo(syncedRobot);

//      exploreAreaMapUI.update();
   }

   private void determineNextPlacesToWalkTo(RemoteSyncedRobotModel syncedRobot)
   {
      PlanarRegionsList concatenatedMap = concatenatedMapSupplier.get();
      BoundingBox3D concatenatedMapBoundingBox = concatenatedMapBoundingBoxSupplier.get();

      HumanoidReferenceFrames referenceFrames = syncedRobot.getReferenceFrames();
      MovingReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FramePoint3D midFeetPosition = new FramePoint3D(midFeetZUpFrame);
      midFeetPosition.changeFrame(worldFrame);

      if (useNewGoalDetermination)
      {
         counter++;
         if (counter > numberOfIterationsBeforeSwitchingGoals)
         {
            counter = 0;
            switch (random.nextInt(4))
            {
               case 0:
                  goalX = EuclidCoreTools.interpolate(maximumExplorationArea.getMinX(), maximumExplorationArea.getMaxX(), random.nextDouble());
                  goalY = maximumExplorationArea.getMinY();
                  break;
               case 1:
                  goalX = EuclidCoreTools.interpolate(maximumExplorationArea.getMinX(), maximumExplorationArea.getMaxX(), random.nextDouble());
                  goalY = maximumExplorationArea.getMaxY();
                  break;
               case 2:
                  goalX = maximumExplorationArea.getMinX();
                  goalY = EuclidCoreTools.interpolate(maximumExplorationArea.getMinY(), maximumExplorationArea.getMaxY(), random.nextDouble());
                  break;
               case 3:
                  goalX = maximumExplorationArea.getMaxX();
                  goalY = EuclidCoreTools.interpolate(maximumExplorationArea.getMinY(), maximumExplorationArea.getMaxY(), random.nextDouble());
                  break;
            }
         }

         explorationPlanner.processRegions(concatenatedMap);
         List<Point3D> waypoints = explorationPlanner.doPlan(midFeetPosition.getX(), midFeetPosition.getY(), goalX, goalY, false);

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

         sortBasedOnBestDistances(potentialPoints, distances, parameters.getMinDistanceToWalkIfPossible());

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
      for (Point3D observationPoint : pointsObservedFromSupplier.get())
      {
         if (pointToCheck.distanceXY(observationPoint) < parameters.getMinimumDistanceBetweenObservationPoints())
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

   public ExploreAreaLatticePlanner getExplorationPlanner()
   {
      return explorationPlanner;
   }

   public boolean isDeterminingNextLocation()
   {
      return determiningNextLocation;
   }

   public boolean isFailedToFindNextLocation()
   {
      return failedToFindNextLocation;
   }

   public List<Pose3DReadOnly> getBestBodyPath()
   {
      return bestBodyPath;
   }

   public ArrayList<Pose3D> getExploredGoalPosesSoFar()
   {
      return exploredGoalPosesSoFar;
   }
}
