package us.ihmc.humanoidBehaviors.exploreArea;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import org.apache.commons.lang3.tuple.MutablePair;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.thread.TypedNotification;
import us.ihmc.communication.RemoteREAInterface;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.BodyPathPlanningResult;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.graphSearch.VisibilityGraphPathPlanner;
import us.ihmc.humanoidBehaviors.BehaviorDefinition;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehavior;
import us.ihmc.humanoidBehaviors.lookAndStep.LookAndStepBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.RemoteHumanoidRobotInterface;
import us.ihmc.avatar.drcRobot.RemoteSyncedRobotModel;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerInterface;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidBehaviors.tools.interfaces.StatusLogger;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAM;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.slam.PlanarRegionSLAMResult;
import us.ihmc.robotEnvironmentAwareness.tools.ConcaveHullMergerListener;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.string.StringTools;
import us.ihmc.tools.thread.PausablePeriodicThread;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;

import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehavior.ExploreAreaBehaviorState.*;
import static us.ihmc.humanoidBehaviors.exploreArea.ExploreAreaBehaviorAPI.*;

public class ExploreAreaBehavior implements BehaviorInterface
{
   public static final BehaviorDefinition DEFINITION = new BehaviorDefinition("Explore Area",
                                                                              ExploreAreaBehavior::new,
                                                                              create(),
                                                                              LookAndStepBehavior.DEFINITION);

   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final List<Double> chestYawsForLookingAround = Arrays.asList(-40.0, 0.0, 40.0); //, 40.0); //-10.0, 0.0); //Arrays.asList(-10.0, -20.0, -30.0, 0.0);
   private final List<Double> headPitchForLookingAround = Arrays.asList(-20.0, 0.0, 20.0);

   private final List<Point3D> pointsObservedFrom = new ArrayList<>();
   private final RemoteFootstepPlannerInterface footstepPlannerToolbox;
   private final RemoteSyncedRobotModel syncedRobot;
   private final Messager messager;

   private int chestYawForLookingAroundIndex = 0;

   private final BoundingBox3D maximumExplorationArea = new BoundingBox3D(-10.0, -10.0, -1.0, 10.0, 10.0, 2.0);

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

   private final StateMachine<ExploreAreaBehaviorState, State> stateMachine;
   private final PausablePeriodicThread mainThread;

   private final AtomicReference<Boolean> explore;
   private final AtomicReference<Boolean> hullGotLooped = new AtomicReference<Boolean>();

   private TypedNotification<RemoteFootstepPlannerResult> footstepPlanResultNotification;
   private TypedNotification<WalkingStatusMessage> walkingCompleted;
   private final Notification lookAndStepReachedGoal;

   private PlanarRegionsList concatenatedMap, latestPlanarRegionsList;
   private BoundingBox3D concatenatedMapBoundingBox;

   private final VisibilityGraphPathPlanner bodyPathPlanner;

   public ExploreAreaBehavior(BehaviorHelper helper)
   {
      this.helper = helper;
      messager = helper.getManagedMessager();
      statusLogger = helper.getOrCreateStatusLogger();
      robotInterface = helper.getOrCreateRobotInterface();
      syncedRobot = robotInterface.newSyncedRobot();
      rea = helper.getOrCreateREAInterface();
      footstepPlannerToolbox = helper.getOrCreateFootstepPlannerToolboxInterface();

      explore = helper.createUIInput(ExploreArea, false);
      helper.createUICallback(Parameters, parameters::setAllFromStrings);
      helper.createUICallback(RandomPoseUpdate, this::randomPoseUpdate);
      helper.createUICallback(DoSlam, this::doSlam);
      helper.createUICallback(ClearMap, this::clearMap);
      lookAndStepReachedGoal = helper.createROS2Notification(LookAndStepBehaviorAPI.REACHED_GOAL);
      bodyPathPlanner = helper.getOrCreateBodyPathPlanner();

      statusLogger.info("Initializing explore area behavior");

      EnumBasedStateMachineFactory<ExploreAreaBehaviorState> factory = new EnumBasedStateMachineFactory<>(ExploreAreaBehaviorState.class);
      factory.setOnEntry(Stop, this::onStopStateEntry);
      factory.setDoAction(Stop, this::doStopStateAction);
      factory.addTransition(Stop, LookAround, this::readyToTransitionFromStopToLookAround);

      factory.setOnEntry(LookAround, this::onLookAroundStateEntry);
      factory.setDoAction(LookAround, this::doLookAroundStateAction);
      factory.addTransition(LookAround, Perceive, this::readyToTransitionFromLookAroundToPerceive);

      factory.setOnEntry(Perceive, this::onPerceiveStateEntry);
      factory.setDoAction(Perceive, this::doPerceiveStateAction);
      factory.addTransition(Perceive, GrabPlanarRegions, this::readyToTransitionFromPerceiveToGrabPlanarRegions);

      factory.setOnEntry(GrabPlanarRegions, this::onGrabPlanarRegionsStateEntry);
      factory.setDoAction(GrabPlanarRegions, this::doGrabPlanarRegionsStateAction);
      factory.addTransition(GrabPlanarRegions, LookAround, this::readyToTransitionFromGrabPlanarRegionsToLookAround);
      factory.addTransition(GrabPlanarRegions, DetermineNextLocations, this::readyToTransitionFromGrabPlanarRegionsToDetermineNextLocations);

      factory.setOnEntry(DetermineNextLocations, this::onDetermineNextLocationsStateEntry);
      factory.setDoAction(DetermineNextLocations, this::doDetermineNextLocationsStateAction);
      factory.addTransition(DetermineNextLocations, TurnInPlace, this::readyToTransitionFromDetermineNextLocationsToTurnInPlace);
      factory.addTransition(DetermineNextLocations, LookAndStep, this::readyToTransitionFromDetermineNextLocationsToLookAndStep);

      factory.setOnEntry(LookAndStep, this::onLookAndStepStateEntry);
      factory.setDoAction(LookAndStep, this::doLookAndStepStateAction);
      factory.addTransition(LookAndStep, Stop, this::readyToTransitionFromLookAndStepToStop);
      factory.addTransition(LookAndStep, TurnInPlace, this::readyToTransitionFromLookAndStepToTurnInPlace);
      factory.setOnExit(LookAndStep, this::doLookAndStepStateExit);

      factory.setOnEntry(TurnInPlace, this::onTurnInPlaceStateEntry);
      factory.setDoAction(TurnInPlace, this::doTurnInPlaceStateAction);
      factory.addTransition(TurnInPlace, Stop, this::readyToTransitionFromTurnInPlaceToStop);

      // Go to stop from any state when no longer exploring.
      factory.addTransition(LookAround, Stop, this::noLongerExploring);
      factory.addTransition(Perceive, Stop, this::noLongerExploring);
      factory.addTransition(GrabPlanarRegions, Stop, this::noLongerExploring);
      factory.addTransition(DetermineNextLocations, Stop, this::noLongerExploring);
      factory.addTransition(LookAndStep, Stop, this::noLongerExploring);
      factory.addTransition(TurnInPlace, Stop, this::noLongerExploring);
      factory.getFactory().addStateChangedListener((from, to) ->
                                                   {
                                                      helper.publishToUI(CurrentState, to);
                                                      statusLogger.info("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
                                                   });

      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(Stop);

      mainThread = helper.createPausablePeriodicThread(getClass(), UnitConversions.hertzToSeconds(2), 5, this::runExploreAreaThread);
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
      robotInterface.pauseWalking();
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
      double chestYaw = Math.toRadians(chestYawsForLookingAround.get(chestYawForLookingAroundIndex));
      double headPitch = Math.toRadians(headPitchForLookingAround.get(chestYawForLookingAroundIndex));
      turnChestWithRespectToMidFeetZUpFrame(chestYaw, parameters.get(ExploreAreaBehaviorParameters.turnChestTrajectoryDuration));
      pitchHeadWithRespectToChest(headPitch, parameters.get(ExploreAreaBehaviorParameters.turnChestTrajectoryDuration));

      chestYawForLookingAroundIndex++;
   }

   private void doLookAroundStateAction(double timeInState)
   {
   }

   private boolean readyToTransitionFromLookAroundToPerceive(double timeInState)
   {
      return (timeInState > parameters.get(ExploreAreaBehaviorParameters.turnTrajectoryWaitTimeMulitplier)
                            * parameters.get(ExploreAreaBehaviorParameters.turnChestTrajectoryDuration));
   }

   private void onPerceiveStateEntry()
   {
      statusLogger.info("Entering perceive state. Clearing LIDAR");
      rea.clearREA();
   }

   private void rememberObservationPoint()
   {
      //TODO: Remember the LIDAR pointing at transform instead of just where the robot was at. But how to get that frame?
      syncedRobot.update();
      MovingReferenceFrame midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      FramePoint3D midFeetLocation = new FramePoint3D(midFeetZUpFrame);
      midFeetLocation.changeFrame(worldFrame);

      helper.publishToUI(ObservationPosition, new Point3D(midFeetLocation));

      this.pointsObservedFrom.add(new Point3D(midFeetLocation));
   }

   private void doPerceiveStateAction(double timeInState)
   {
   }

   private boolean readyToTransitionFromPerceiveToGrabPlanarRegions(double timeInState)
   {
      double perceiveDuration = parameters.get(ExploreAreaBehaviorParameters.perceiveDuration);
      statusLogger.info("Perceiving for {} s", perceiveDuration);
      return (timeInState > perceiveDuration);
   }

   private void onGrabPlanarRegionsStateEntry()
   {
      helper.publishToUI(ClearPlanarRegions, true);
      rememberObservationPoint();
      doSlam(true);
   }

   private ConcaveHullMergerListener listener = new ConcaveHullMergerListener()
   {
      boolean savedOutTroublesomeRegions = false;

      @Override
      public void originalHulls(List<Point2D> hullOne, List<Point2D> hullTwo)
      {
      }

      @Override
      public void preprocessedHull(List<Point2D> hullOne, List<Point2D> hullTwo)
      {
      }

      @Override
      public void hullGotLooped(List<Point2D> hullOne, List<Point2D> hullTwo, List<Point2D> mergedVertices)
      {
         hullGotLooped.set(true);

         statusLogger.error("Hull got looped.");

         if (savedOutTroublesomeRegions)
            return;

         savedOutTroublesomeRegions = true;

         statusLogger.error("First occurance this run, so saving out PlanarRegions");

         try
         {
            Path planarRegionsPath = new File("Troublesome" + File.separator + "MapPlanarRegions").toPath();
            statusLogger.error("map planarRegionsPath = {}", planarRegionsPath);
            Files.createDirectories(planarRegionsPath);
            PlanarRegionFileTools.exportPlanarRegionData(planarRegionsPath, concatenatedMap.copy());

            planarRegionsPath = new File("Troublesome" + File.separator + "NewPlanarRegions").toPath();
            statusLogger.error("new planarRegionsPath = {}", planarRegionsPath);
            Files.createDirectories(planarRegionsPath);
            PlanarRegionFileTools.exportPlanarRegionData(planarRegionsPath, latestPlanarRegionsList.copy());
         }
         catch (IOException e)
         {
            e.printStackTrace();
         }

         //         ThreadTools.sleepForever();
      }

      @Override
      public void foundStartingVertexAndWorkingHull(Point2D startingVertex, List<Point2D> workingHull, boolean workingHullIsOne)
      {
      }

      @Override
      public void foundIntersectionPoint(Point2D intersectionPoint, boolean workingHullIsOne)
      {
      }

      @Override
      public void consideringWorkingEdge(LineSegment2D workingEdge, boolean workingHullIsOne)
      {
      }

      @Override
      public void hullIsInvalid(List<Point2D> invalidHull)
      {
      }

      @Override
      public void hullsAreInvalid(List<Point2D> invalidHullA, List<Point2D> invalidHullB)
      {
      }
   };

   //   private ConcaveHullMergerListener listener = new ConcaveHullMergerListener();

   private void doSlam(boolean doSlam)
   {
      PlanarRegionsList latestPlanarRegionsList = rea.getLatestPlanarRegionsList();

      this.latestPlanarRegionsList = latestPlanarRegionsList;
      if (concatenatedMap == null)
      {
         concatenatedMap = latestPlanarRegionsList;
      }
      else
      {
         PlanarRegionSLAMParameters slamParameters = new PlanarRegionSLAMParameters();

         //TODO: Tune these and add parameter decay factor for each iteration. 
         // Maybe have an array of values to use for each iteration.
         //TODO: Have SLAM return statistics, like residual error, to help determine the goodness of the data
         // and take action accordingly.
         slamParameters.setIterationsForMatching(5);
         slamParameters.setBoundingBoxHeight(0.05);
         slamParameters.setMinimumNormalDotProduct(0.99);
         slamParameters.setDampedLeastSquaresLambda(5.0);
         slamParameters.setMaximumPointProjectionDistance(0.10);

         hullGotLooped.set(false);
         syncedRobot.update();
         RigidBodyTransform referenceTransform = syncedRobot.getReferenceFrames().getIMUFrame().getTransformToWorldFrame();
         statusLogger.info("Doing SLAM with IMU reference Transform \n {} ", referenceTransform);

         PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(concatenatedMap, latestPlanarRegionsList, slamParameters, referenceTransform, listener);

         concatenatedMap = slamResult.getMergedMap();
         RigidBodyTransform transformFromIncomingToMap = slamResult.getTransformFromIncomingToMap();
         statusLogger.info("SLAM transformFromIncomingToMap = \n {}", transformFromIncomingToMap);

         boolean sendingSlamCorrection = true;
         publishPoseUpdateForStateEstimator(transformFromIncomingToMap, sendingSlamCorrection);
      }

      computeMapBoundingBox3D();

      statusLogger.info("concatenatedMap has " + concatenatedMap.getNumberOfPlanarRegions() + " planar Regions");
      statusLogger.info("concatenatedMapBoundingBox = " + concatenatedMapBoundingBox);

      List<PlanarRegion> planarRegionsAsList = concatenatedMap.getPlanarRegionsAsList();

      int index = 0;
      for (PlanarRegion planarRegion : planarRegionsAsList)
      {
         helper.publishToUI(AddPlanarRegionToMap, TemporaryPlanarRegionMessage.convertToTemporaryPlanarRegionMessage(planarRegion, index));

         List<ConvexPolygon2D> convexPolygons = planarRegion.getConvexPolygons();
         for (ConvexPolygon2D polygon : convexPolygons)
         {
            helper.publishToUI(AddPolygonToPlanarRegion, TemporaryConvexPolygon2DMessage.convertToTemporaryConvexPolygon2DMessage(polygon, index));
         }

         index++;
      }

      helper.publishToUI(DrawMap, true);

      // Send it to the GUI for a viz...
      //         PlanarRegionsListMessage concatenatedMapMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(concatenatedMap);
      //         helper.publishToUI(ExploreAreaBehavior.ConcatenatedMap, concatenatedMapMessage);

      // Find a point that has not been observed, but is close to a point that can be walked to, in order to observe it...
   }

   private void clearMap(boolean clearMap)
   {
      helper.publishToUI(ClearPlanarRegions, true);
      concatenatedMap = null;
   }

   private void publishPoseUpdateForStateEstimator(RigidBodyTransform transformFromIncomingToMap, boolean sendingSlamCorrection)
   {
      syncedRobot.update();

      FramePose3D framePose = new FramePose3D(syncedRobot.getReferenceFrames().getPelvisFrame());
      framePose.changeFrame(worldFrame);
      Pose3D pose3D = new Pose3D(framePose);

      //TODO: Verify which transform or appendTransform to use...

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
      robotInterface.publishPose(pose3D, confidenceFactor, syncedRobot.getTimestamp());
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
      robotInterface.requestChestGoHome(parameters.get(ExploreAreaBehaviorParameters.turnChestTrajectoryDuration));

      desiredFramePoses = null;
      determinedNextLocations = false;

      syncedRobot.update();
      determineNextPlacesToWalkTo(syncedRobot);
      determinedNextLocations = true;
   }

   private void determineNextPlacesToWalkTo(RemoteSyncedRobotModel syncedRobot)
   {
      HumanoidReferenceFrames referenceFrames = syncedRobot.getReferenceFrames();
      MovingReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FramePoint3D midFeetPosition = new FramePoint3D(midFeetZUpFrame);
      midFeetPosition.changeFrame(worldFrame);

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

      HashMap<Point3DReadOnly, Double> distancesFromStart = new HashMap<>();
      for (Point3DReadOnly testGoal : potentialPoints)
      {
         distancesFromStart.put(testGoal, midFeetPosition.distanceXY(testGoal));
      }

      sortBasedOnBestDistances(potentialPoints, distancesFromStart, parameters.get(ExploreAreaBehaviorParameters.minDistanceToWalkIfPossible));

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
            distancesFromStart.put(testGoal, midFeetPosition.distanceXY(testGoal));

            if (feasibleGoalPoints.size() >= maxNumberOfFeasiblePointsToLookFor)
               break;
         }
      }

      long endTime = System.nanoTime();
      long duration = (endTime - startTime);
      double durationSeconds = ((double) duration) / 1.0e9;
      double durationPer = durationSeconds / ((double) numberConsidered);

      statusLogger.info(
            "Found " + feasibleGoalPoints.size() + " feasible Points that have body paths to. Took " + durationSeconds + " seconds to find the body paths, or "
            + durationPer + " seconds Per attempt.");

      desiredFramePoses = new ArrayList<>();

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
      double bestDistance = distancesFromStart.get(bestGoalPoint);
      List<Pose3DReadOnly> bestBodyPath = potentialBodyPaths.get(bestGoalPoint);

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

   private void sortBasedOnBestDistances(ArrayList<Point3D> potentialPoints, HashMap<Point3DReadOnly, Double> distancesFromStart, double minDistanceIfPossible)
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

   private boolean readyToTransitionFromDetermineNextLocationsToTurnInPlace(double timeInState)
   {
      boolean outOfPlaces = desiredFramePoses.isEmpty();
      if (outOfPlaces)
      {
         statusLogger.error("Out of places to plan to. We're lost!");
      }
      return outOfPlaces;
   }

   private boolean readyToTransitionFromDetermineNextLocationsToLookAndStep(double timeInState)
   {
      return determinedNextLocations && !desiredFramePoses.isEmpty();
   }

   private void onLookAndStepStateEntry()
   {
      FramePose3D goal = desiredFramePoses.remove(0);

      statusLogger.info("Planning to {}", StringTools.zUpPoseString(goal));
      Point3D goalToSend = new Point3D(goal.getPosition());

      helper.publishToUI(PlanningToPosition, goalToSend);

      messager.submitMessage(LookAndStepBehaviorAPI.OperatorReviewEnabled, false);
      helper.publishROS2(LookAndStepBehaviorAPI.GOAL_INPUT, new Pose3D(goal));
      lookAndStepReachedGoal.poll();
   }

   private void doLookAndStepStateAction()
   {

   }

   private boolean readyToTransitionFromLookAndStepToStop()
   {
      return false;
   }

   private boolean readyToTransitionFromLookAndStepToTurnInPlace(double timeInState)
   {
      return lookAndStepReachedGoal.poll();
   }

   private void doLookAndStepStateExit()
   {

   }

   private void onTurnInPlaceStateEntry()
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

   private boolean readyToTransitionFromTurnInPlaceToStop(double timeInState)
   {
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

   private void doTurnInPlaceStateAction(double timeInState)
   {
      walkingCompleted.poll();
   }

   public void turnChestWithRespectToMidFeetZUpFrame(double chestYaw, double trajectoryTime)
   {
      syncedRobot.update();

      ReferenceFrame midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      FrameQuaternion chestOrientation = new FrameQuaternion(midFeetZUpFrame, chestYaw, 0.0, 0.0);
      chestOrientation.changeFrame(worldFrame);
      robotInterface.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, syncedRobot.getReferenceFrames().getPelvisZUpFrame());
      robotInterface.requestPelvisGoHome(trajectoryTime);
   }

   public void pitchHeadWithRespectToChest(double headPitch, double trajectoryTime)
   {
      syncedRobot.update();

      ReferenceFrame chestFrame = syncedRobot.getReferenceFrames().getChestFrame();
      FrameQuaternion headOrientation = new FrameQuaternion(chestFrame, 0.0, headPitch, 0.0);
      headOrientation.changeFrame(worldFrame);
      robotInterface.requestHeadOrientationTrajectory(trajectoryTime, headOrientation, worldFrame, syncedRobot.getReferenceFrames().getPelvisZUpFrame());
   }

   private void rotateChestAndPitchHeadToLookAtPointInWorld(double timeInState,
                                                            RobotSide swingSide,
                                                            Point3D pointToLookAtInWorld,
                                                            FullHumanoidRobotModel fullRobotModel,
                                                            HumanoidReferenceFrames referenceFrames)
   {
      //      ReferenceFrame headBaseFrame = fullRobotModel.getHeadBaseFrame();
      MovingReferenceFrame chestFrame = referenceFrames.getChestFrame();

      //      FramePoint3D footstepLocationInHeadFrame = new FramePoint3D(worldFrame, locationInWorld);
      //      footstepLocationInHeadFrame.changeFrame(headBaseFrame);

      FramePoint3D pointToLookAtInWorldFrame = new FramePoint3D(worldFrame, pointToLookAtInWorld);
//      FramePoint3D supportFootLocationInWorld = new FramePoint3D(referenceFrames.getSoleFrame(swingSide.getOppositeSide()));
//      supportFootLocationInWorld.changeFrame(worldFrame);

//      FrameVector3D soleToPointToLookAtVector = new FrameVector3D(worldFrame);
//      soleToPointToLookAtVector.sub(pointToLookAtInWorldFrame, supportFootLocationInWorld);

      FramePoint3D headLocationInWorld = new FramePoint3D(fullRobotModel.getHeadBaseFrame());
      headLocationInWorld.changeFrame(worldFrame);

      FrameVector3D headToPointToLookAtVector = new FrameVector3D(worldFrame);
      headToPointToLookAtVector.sub(pointToLookAtInWorldFrame, headLocationInWorld);

      double totalMotionTime = 1.0;
      double trajectoryTime = totalMotionTime - timeInState;
      if (trajectoryTime < 0.1)
         trajectoryTime = 0.1;

      //      MovingReferenceFrame neckFrame = referenceFrames.getNeckFrame(NeckJointName.PROXIMAL_NECK_PITCH);
      //      Vector2D footstepLocationXY = new Vector2D(footstepLocationInChestFrame);

      Vector2D headToFootstepFrameXY = new Vector2D(headToPointToLookAtVector);

      double yaw = Math.atan2(headToPointToLookAtVector.getY(), headToPointToLookAtVector.getX());
      double pitch = -Math.atan2(headToPointToLookAtVector.getZ(), headToFootstepFrameXY.length());

//      LogTools.info("Turning to look at footstep. Yaw = {}, pitch = {}", yaw, pitch);

      FrameQuaternion chestOrientation = new FrameQuaternion(worldFrame);
      chestOrientation.setYawPitchRoll(yaw, 0.25 * pitch, 0.0);
      robotInterface.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, worldFrame);

//      double worldYaw = chestOrientation.getYaw();
      FrameQuaternion headOrientation = new FrameQuaternion(worldFrame);
      headOrientation.setYawPitchRoll(yaw, 0.75 * pitch, 0.0);
      robotInterface.requestHeadOrientationTrajectory(trajectoryTime, headOrientation, worldFrame, worldFrame);
   }

   //TODO: Hijacking PatrolBehavior Viz here. Should not be doing that. Should have some common vizzes for things like this that are shared.
   private void reduceAndSendFootstepsForVisualization(FootstepPlan footstepPlan)
   {
      ArrayList<Pair<RobotSide, Pose3D>> footstepLocations = new ArrayList<>();
      for (int i = 0; i < footstepPlan.getNumberOfSteps(); i++) // this code makes the message smaller to send over the network, TODO investigate
      {
         FramePose3D soleFramePoseToPack = new FramePose3D();
         footstepPlan.getFootstep(i).getFootstepPose(soleFramePoseToPack);
         footstepLocations.add(new MutablePair<>(footstepPlan.getFootstep(i).getRobotSide(), new Pose3D(soleFramePoseToPack)));
      }
   }
}
