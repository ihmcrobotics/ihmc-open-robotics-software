package us.ihmc.humanoidBehaviors.exploreArea;

import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
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
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.PlanarRegionsListMessage;
import controller_msgs.msg.dds.WalkingStatusMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.BoundingBox3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlanningResult;
import us.ihmc.humanoidBehaviors.BehaviorInterface;
import us.ihmc.humanoidBehaviors.patrol.PatrolBehaviorAPI;
import us.ihmc.humanoidBehaviors.tools.BehaviorHelper;
import us.ihmc.humanoidBehaviors.tools.footstepPlanner.RemoteFootstepPlannerResult;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAM;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMParameters;
import us.ihmc.humanoidBehaviors.tools.perception.PlanarRegionSLAMResult;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.idl.IDLSequence.Object;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.messager.Messager;
import us.ihmc.messager.MessagerAPIFactory;
import us.ihmc.messager.MessagerAPIFactory.Category;
import us.ihmc.messager.MessagerAPIFactory.CategoryTheme;
import us.ihmc.messager.MessagerAPIFactory.MessagerAPI;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ConcaveHullMergerListener;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateMachine;
import us.ihmc.robotics.stateMachine.extra.EnumBasedStateMachineFactory;
import us.ihmc.tools.thread.ExceptionHandlingThreadScheduler;
import us.ihmc.tools.thread.TypedNotification;

public class ExploreAreaBehavior implements BehaviorInterface
{
   private final ExploreAreaBehaviorParameters parameters = new ExploreAreaBehaviorParameters();

   private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final List<Double> chestYawsForLookingAround = Arrays.asList(-40.0, 0.0, 40.0); //, 40.0); //-10.0, 0.0); //Arrays.asList(-10.0, -20.0, -30.0, 0.0);
   private final List<Double> headPitchForLookingAround = Arrays.asList(-20.0, 0.0, 20.0);

   private final List<Point3D> pointsObservedFrom = new ArrayList<>();

   private int chestYawForLookingAroundIndex = 0;

   private final BoundingBox3D maximumExplorationArea = new BoundingBox3D(-10.0, -10.0, -1.0, 10.0, 10.0, 2.0);

   private double exploreGridXSteps = 0.5;
   private double exploreGridYSteps = 0.5;
   private int maxNumberOfFeasiblePointsToLookFor = 10; //30;

   public enum ExploreAreaBehaviorState
   {
      Stop, LookAround, Perceive, GrabPlanarRegions, DetermineNextLocations, Plan, WalkToNextLocation, TakeAStep, TurnInPlace
   }

   private final BehaviorHelper behaviorHelper;

   private final Messager messager;
   private final StateMachine<ExploreAreaBehaviorState, State> stateMachine;

   private final AtomicReference<Boolean> explore;
   private final AtomicReference<Boolean> hullGotLooped = new AtomicReference<Boolean>();

   private TypedNotification<RemoteFootstepPlannerResult> footstepPlanResultNotification;
   private TypedNotification<WalkingStatusMessage> walkingCompleted;

   private PlanarRegionsList concatenatedMap, latestPlanarRegionsList;
   private BoundingBox3D concatenatedMapBoundingBox;

   private final NavigableRegionsManager navigableRegionsManager;

   public ExploreAreaBehavior(BehaviorHelper behaviorHelper, Messager messager, DRCRobotModel robotModel)
   {
      this.behaviorHelper = behaviorHelper;
      this.messager = messager;

      explore = messager.createInput(ExploreAreaBehaviorAPI.ExploreArea, false);
      messager.registerTopicListener(ExploreAreaBehaviorAPI.RandomPoseUpdate, this::randomPoseUpdate);
      messager.registerTopicListener(ExploreAreaBehaviorAPI.DoSlam, this::doSlam);
      messager.registerTopicListener(ExploreAreaBehaviorAPI.ClearMap, this::clearMap);
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
      factory.addTransition(ExploreAreaBehaviorState.Perceive,
                            ExploreAreaBehaviorState.GrabPlanarRegions,
                            this::readyToTransitionFromPerceiveToGrabPlanarRegions);

      factory.setOnEntry(ExploreAreaBehaviorState.GrabPlanarRegions, this::onGrabPlanarRegionsStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.GrabPlanarRegions, this::doGrabPlanarRegionsStateAction);
      factory.addTransition(ExploreAreaBehaviorState.GrabPlanarRegions,
                            ExploreAreaBehaviorState.LookAround,
                            this::readyToTransitionFromGrabPlanarRegionsToLookAround);
      factory.addTransition(ExploreAreaBehaviorState.GrabPlanarRegions,
                            ExploreAreaBehaviorState.DetermineNextLocations,
                            this::readyToTransitionFromGrabPlanarRegionsToDetermineNextLocations);

      factory.setOnEntry(ExploreAreaBehaviorState.DetermineNextLocations, this::onDetermineNextLocationsStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.DetermineNextLocations, this::doDetermineNextLocationsStateAction);
      factory.addTransition(ExploreAreaBehaviorState.DetermineNextLocations,
                            ExploreAreaBehaviorState.Plan,
                            this::readyToTransitionFromDetermineNextLocationsToPlan);

      factory.setOnEntry(ExploreAreaBehaviorState.Plan, this::onPlanStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.Plan, this::doPlanStateAction);
      factory.addTransition(ExploreAreaBehaviorState.Plan, ExploreAreaBehaviorState.WalkToNextLocation, this::readyToTransitionFromPlanToWalkToNextLocation);
      factory.addTransition(ExploreAreaBehaviorState.Plan, ExploreAreaBehaviorState.Plan, this::readyToTransitionFromPlanToPlan);
      factory.addTransition(ExploreAreaBehaviorState.Plan, ExploreAreaBehaviorState.TurnInPlace, this::readyToTransitionFromPlanToTurnInPlace);

      factory.setOnEntry(ExploreAreaBehaviorState.WalkToNextLocation, this::onWalkToNextLocationStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.WalkToNextLocation, this::doWalkToNextLocationStateAction);
      // factory.addTransition(ExploreAreaBehaviorState.WalkToNextLocation, ExploreAreaBehaviorState.Stop, this::readyToTransitionFromWalkToNextLocationToStop);
      factory.addTransition(ExploreAreaBehaviorState.WalkToNextLocation,
                            ExploreAreaBehaviorState.TakeAStep,
                            this::readyToTransitionFromWalkToNextLocationToTakeAStep);

      factory.setOnEntry(ExploreAreaBehaviorState.TakeAStep, this::onTakeAStepStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.TakeAStep, this::doTakeAStepStateAction);
      factory.addTransition(ExploreAreaBehaviorState.TakeAStep, ExploreAreaBehaviorState.TakeAStep, this::readyToTransitionFromTakeAStepToTakeAStep);
      factory.addTransition(ExploreAreaBehaviorState.TakeAStep, ExploreAreaBehaviorState.Stop, this::readyToTransitionFromTakeAStepToStop);

      factory.setOnEntry(ExploreAreaBehaviorState.TurnInPlace, this::onTurnInPlaceStateEntry);
      factory.setDoAction(ExploreAreaBehaviorState.TurnInPlace, this::doTurnInPlaceStateAction);
      factory.addTransition(ExploreAreaBehaviorState.TurnInPlace, ExploreAreaBehaviorState.Stop, this::readyToTransitionFromTurnInPlaceToStop);

      // Go to stop from any state when no longer exploring.
      factory.addTransition(ExploreAreaBehaviorState.LookAround, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.Perceive, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.GrabPlanarRegions, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.DetermineNextLocations, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.Plan, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.WalkToNextLocation, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.addTransition(ExploreAreaBehaviorState.TurnInPlace, ExploreAreaBehaviorState.Stop, this::noLongerExploring);
      factory.getFactory().addStateChangedListener((from, to) ->
      {
         messager.submitMessage(ExploreAreaBehaviorAPI.CurrentState, to);
         LogTools.debug("{} -> {}", from == null ? null : from.name(), to == null ? null : to.name());
      });

      factory.getFactory().buildClock(() -> Conversions.nanosecondsToSeconds(System.nanoTime()));
      stateMachine = factory.getFactory().build(ExploreAreaBehaviorState.Stop);

      ExceptionHandlingThreadScheduler exploreAreaThread = new ExceptionHandlingThreadScheduler(getClass().getSimpleName(),
                                                                                                DefaultExceptionHandler.PRINT_STACKTRACE,
                                                                                                5);
      exploreAreaThread.schedule(this::runExploreAreaThread, 500, TimeUnit.MILLISECONDS);
   }

   @Override
   public void setEnabled(boolean enabled)
   {

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
         transform.setRotationYaw(0.025);

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
      return (timeInState > parameters.get(ExploreAreaBehaviorParameters.perceiveDuration));
   }

   private void onGrabPlanarRegionsStateEntry()
   {
      messager.submitMessage(ExploreAreaBehaviorAPI.ClearPlanarRegions, true);
      rememberObservationPoint();
      doSlam(true);
   }

   private ConcaveHullMergerListener listener = new ConcaveHullMergerListener()
   {
      boolean savedOutTroublesomeRegions = false;

      @Override
      public void originalHulls(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo)
      {
      }

      @Override
      public void preprocessedHull(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo)
      {
      }

      @Override
      public void hullGotLooped(ArrayList<Point2D> hullOne, ArrayList<Point2D> hullTwo, ArrayList<Point2D> mergedVertices)
      {
         hullGotLooped.set(true);

         LogTools.error("Hull got looped.");

         if (savedOutTroublesomeRegions)
            return;

         savedOutTroublesomeRegions = true;

         LogTools.error("First occurance this run, so saving out PlanarRegions");

         try
         {
            Path planarRegionsPath = new File("Troublesome" + File.separator + "MapPlanarRegions").toPath();
            LogTools.error("map planarRegionsPath = {}", planarRegionsPath);
            Files.createDirectories(planarRegionsPath);
            PlanarRegionFileTools.exportPlanarRegionData(planarRegionsPath, concatenatedMap.copy());

            planarRegionsPath = new File("Troublesome" + File.separator + "NewPlanarRegions").toPath();
            LogTools.error("new planarRegionsPath = {}", planarRegionsPath);
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
      public void foundStartingVertexAndWorkingHull(Point2D startingVertex, ArrayList<Point2D> workingHull, boolean workingHullIsOne)
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
      public void hullIsInvalid(ArrayList<Point2D> invalidHull)
      {
      }

      @Override
      public void hullsAreInvalid(ArrayList<Point2D> invalidHullA, ArrayList<Point2D> invalidHullB)
      {
      }
   };

   //   private ConcaveHullMergerListener listener = new ConcaveHullMergerListener();

   private void doSlam(boolean doSlam)
   {
      PlanarRegionsList latestPlanarRegionsList = behaviorHelper.getLatestPlanarRegionList();

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
         RigidBodyTransform referenceTransform = behaviorHelper.pollHumanoidReferenceFrames().getIMUFrame().getTransformToWorldFrame();
         LogTools.info("Doing SLAM with IMU reference Transform \n {} ", referenceTransform);

         PlanarRegionSLAMResult slamResult = PlanarRegionSLAM.slam(concatenatedMap, latestPlanarRegionsList, slamParameters, referenceTransform, listener);

         concatenatedMap = slamResult.getMergedMap();
         RigidBodyTransform transformFromIncomingToMap = slamResult.getTransformFromIncomingToMap();
         LogTools.info("SLAM transformFromIncomingToMap = \n {}", transformFromIncomingToMap);

         boolean sendingSlamCorrection = true;
         publishPoseUpdateForStateEstimator(transformFromIncomingToMap, sendingSlamCorrection);
      }

      computeMapBoundingBox3D();

      LogTools.info("concatenatedMap has " + concatenatedMap.getNumberOfPlanarRegions() + " planar Regions");
      LogTools.info("concatenatedMapBoundingBox = " + concatenatedMapBoundingBox);

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

   private void clearMap(boolean clearMap)
   {
      messager.submitMessage(ExploreAreaBehaviorAPI.ClearPlanarRegions, true);
      concatenatedMap = null;
   }

   private void publishPoseUpdateForStateEstimator(RigidBodyTransform transformFromIncomingToMap, boolean sendingSlamCorrection)
   {
      Pair<HumanoidReferenceFrames, Long> referenceFramesAndTimestamp = behaviorHelper.pollHumanoidReferenceFramesAndTimestamp();
      HumanoidReferenceFrames referenceFrames = referenceFramesAndTimestamp.getLeft();
      Long timestamp = referenceFramesAndTimestamp.getRight();

      FramePose3D framePose = new FramePose3D(referenceFrames.getPelvisFrame());
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
      behaviorHelper.publishPose(pose3D, confidenceFactor, timestamp);
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
      behaviorHelper.requestChestGoHome(parameters.get(ExploreAreaBehaviorParameters.turnChestTrajectoryDuration));

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

      ArrayList<Point3D> potentialPoints = new ArrayList<>();

      // Do a grid over the bounding box to find potential places to step.

      BoundingBox3D intersectionBoundingBox = getBoundingBoxIntersection(maximumExplorationArea, concatenatedMapBoundingBox);

      ArrayList<BoundingBox3D> explorationBoundingBoxes = new ArrayList<BoundingBox3D>();
      explorationBoundingBoxes.add(maximumExplorationArea);
      explorationBoundingBoxes.add(concatenatedMapBoundingBox);
      explorationBoundingBoxes.add(intersectionBoundingBox);

      messager.submitMessage(ExploreAreaBehaviorAPI.ExplorationBoundingBoxes, explorationBoundingBoxes);

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

      LogTools.info("Found " + potentialPoints.size() + " potential Points on the grid.");

      ArrayList<Point3D> potentialPointsToSend = new ArrayList<Point3D>();
      potentialPointsToSend.addAll(potentialPoints);
      messager.submitMessage(ExploreAreaBehaviorAPI.PotentialPointsToExplore, potentialPointsToSend);

      // Compute distances to each.

      HashMap<Point3DReadOnly, Double> distancesFromStart = new HashMap<>();
      for (Point3DReadOnly testGoal : potentialPoints)
      {
         distancesFromStart.put(testGoal, midFeetPosition.distanceXY(testGoal));
      }

      sortBasedOnBestDistances(potentialPoints, distancesFromStart, parameters.get(ExploreAreaBehaviorParameters.minDistanceToWalkIfPossible));

      LogTools.info("Sorted the points based on best distances. Now looking for body paths to those potential goal locations.");

      long startTime = System.nanoTime();

      ArrayList<Point3DReadOnly> feasibleGoalPoints = new ArrayList<>();
      HashMap<Point3DReadOnly, List<Point3DReadOnly>> potentialBodyPaths = new HashMap<>();

      navigableRegionsManager.setPlanarRegions(concatenatedMap.getPlanarRegionsAsList());

      int numberConsidered = 0;

      for (Point3D testGoal : potentialPoints)
      {
         //         LogTools.info("Looking for body path to " + testGoal);

         List<Point3DReadOnly> bodyPath = navigableRegionsManager.calculateBodyPath(midFeetPosition, testGoal);
         numberConsidered++;

         if (bodyPath != null)
         {
            //            LogTools.info("Found body path to " + testGoal);
            messager.submitMessage(ExploreAreaBehaviorAPI.FoundBodyPathTo, new Point3D(testGoal));

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

   private boolean readyToTransitionFromDetermineNextLocationsToPlan(double timeInState)
   {
      return determinedNextLocations;
   }

   private boolean plannerFinished = false;
   private boolean foundValidPlan = false;
   private FootstepPlan footstepPlan = null;
   private FootstepDataListMessage footstepDataListMessageFromPlan = null;
   private PlanarRegionsList planarRegionsListFromPlan = null;
   private boolean outOfPlans = false;

   private void onPlanStateEntry()
   {
      resetFootstepPlanning();

      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
      FramePose3DReadOnly midFeetZUpPose = behaviorHelper.quickPollPoseReadOnly(HumanoidReferenceFrames::getMidFeetZUpFrame);

      if (!desiredFramePoses.isEmpty())
      {
         FramePose3D goal = desiredFramePoses.get(0);
         desiredFramePoses.remove(0);

         LogTools.info("\nPlanning to " + goal);
         Point3D goalToSend = new Point3D(goal.getPosition());

         messager.submitMessage(ExploreAreaBehaviorAPI.PlanningToPosition, goalToSend);

         footstepPlanResultNotification = behaviorHelper.requestPlan(midFeetZUpPose, goal, concatenatedMap);
      }
      else
      {
         LogTools.info("Out of places to plan too!!!! We're lost!!");
         outOfPlans = true;
      }
   }

   private void doPlanStateAction(double timeInState)
   {
      //TODO: Something wrong with the planner. If you check on the results too quickly you get some errors. Need to fix planner issues.
      if (timeInState < 3.0)
         return;

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
      outOfPlans = false;
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

   private boolean readyToTransitionFromPlanToTurnInPlace(double timeInState)
   {
      return outOfPlans;
   }

   private RecyclingArrayList<FootstepDataMessage> footstepDataList = null;
   private int footstepIndex = 0;

   private void onWalkToNextLocationStateEntry()
   {
      //      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
      footstepDataList = footstepDataListMessageFromPlan.getFootstepDataList();
      footstepIndex = 0;

      //      walkingCompleted = behaviorHelper.requestWalk(footstepDataListMessageFromPlan, referenceFrames, planarRegionsListFromPlan);
   }

   private void doWalkToNextLocationStateAction(double timeInState)
   {
      //      walkingCompleted.poll();
   }

   private boolean readyToTransitionFromWalkToNextLocationToTakeAStep(double timeInState)
   {
      return true;
   }

   //   private boolean readyToTransitionFromWalkToNextLocationToStop(double timeInState)
   //   {
   //      return (walkingCompleted.hasNext());
   //      //      return ((timeInState > 0.1) && (!behaviorHelper.isRobotWalking()));
   //   }

   private RobotSide takeAStepSwingSide;
   private Point3D nextFootstepLocation;

   private void onTakeAStepStateEntry()
   {
      FullHumanoidRobotModel fullRobotModel = behaviorHelper.pollFullRobotModel();
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();

      FootstepDataMessage footstepDataMessage = footstepDataList.get(footstepIndex);
      takeAStepSwingSide = RobotSide.fromByte(footstepDataMessage.getRobotSide());

      FootstepDataListMessage messageWithOneStep = new FootstepDataListMessage(footstepDataListMessageFromPlan);
      messageWithOneStep.getFootstepDataList().clear();
      messageWithOneStep.getFootstepDataList().add().set(footstepDataMessage);
      walkingCompleted = behaviorHelper.requestWalk(messageWithOneStep, referenceFrames, planarRegionsListFromPlan);

      LogTools.info("Stepping to " + footstepDataMessage.getLocation());

      if (footstepIndex < footstepDataList.size() - 1)
      {
         FootstepDataMessage nextFootstep = footstepDataList.get(footstepIndex + 1);
         nextFootstepLocation = nextFootstep.getLocation();
         System.out.println("Stare at next Footstep " + nextFootstep.getLocation());

//         rotateChestAndPitchHeadToLookAtPointInWorld(0.0, takeAStepSwingSide, nextFootstepLocation, fullRobotModel, referenceFrames);
      }
      else
      {
         nextFootstepLocation = null;
         behaviorHelper.requestChestGoHome(parameters.get(ExploreAreaBehaviorParameters.turnChestTrajectoryDuration));
      }

      footstepIndex++;
   }

   private void doTakeAStepStateAction(double timeInState)
   {
      FullHumanoidRobotModel fullRobotModel = behaviorHelper.pollFullRobotModel();
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();

      if (nextFootstepLocation != null)
      {
//         rotateChestAndPitchHeadToLookAtPointInWorld(timeInState, takeAStepSwingSide, nextFootstepLocation, fullRobotModel, referenceFrames);
         nextFootstepLocation = null;
      }

      walkingCompleted.poll();
   }

   private boolean readyToTransitionFromTakeAStepToTakeAStep(double timeInState)
   {
      return ((footstepIndex < footstepDataList.size()) && (walkingCompleted.hasNext()));
      //      return ((timeInState > 0.1) && (!behaviorHelper.isRobotWalking()));
   }

   private boolean readyToTransitionFromTakeAStepToStop(double timeInState)
   {
      return ((footstepIndex >= footstepDataList.size()) && (walkingCompleted.hasNext()));
      //      return ((timeInState > 0.1) && (!behaviorHelper.isRobotWalking()));
   }

   private void onTurnInPlaceStateEntry()
   {
      ArrayList<Pose3D> posesFromThePreviousStep = new ArrayList<Pose3D>();

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
      return (walkingCompleted.hasNext());
      //      return ((timeInState > 0.1) && (!behaviorHelper.isRobotWalking()));
   }

   private void requestWalk(RobotSide supportSide, ArrayList<Pose3D> posesFromThePreviousStep)
   {
      RobotSide swingSide = supportSide.getOppositeSide();
      FootstepDataListMessage footstepDataListMessageToTurnInPlace = new FootstepDataListMessage();
      Object<FootstepDataMessage> footstepDataList = footstepDataListMessageToTurnInPlace.getFootstepDataList();
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();
      ReferenceFrame supportFootFrame = referenceFrames.getSoleFrame(supportSide);

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

      walkingCompleted = behaviorHelper.requestWalk(footstepDataListMessageToTurnInPlace, referenceFrames, concatenatedMap);
   }

   private void doTurnInPlaceStateAction(double timeInState)
   {
      walkingCompleted.poll();
   }

   public void turnChestWithRespectToMidFeetZUpFrame(double chestYaw, double trajectoryTime)
   {
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();

      ReferenceFrame midFeetZUpFrame = referenceFrames.getMidFeetZUpFrame();
      FrameQuaternion chestOrientation = new FrameQuaternion(midFeetZUpFrame, chestYaw, 0.0, 0.0);
      chestOrientation.changeFrame(worldFrame);
      behaviorHelper.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, referenceFrames.getPelvisZUpFrame());
      behaviorHelper.requestPelvisGoHome(trajectoryTime);
   }

   public void pitchHeadWithRespectToChest(double headPitch, double trajectoryTime)
   {
      HumanoidReferenceFrames referenceFrames = behaviorHelper.pollHumanoidReferenceFrames();

      ReferenceFrame chestFrame = referenceFrames.getChestFrame();
      FrameQuaternion headOrientation = new FrameQuaternion(chestFrame, 0.0, headPitch, 0.0);
      headOrientation.changeFrame(worldFrame);
      behaviorHelper.requestHeadOrientationTrajectory(trajectoryTime, headOrientation, worldFrame, referenceFrames.getPelvisZUpFrame());
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
      behaviorHelper.requestChestOrientationTrajectory(trajectoryTime, chestOrientation, worldFrame, worldFrame);

//      double worldYaw = chestOrientation.getYaw();
      FrameQuaternion headOrientation = new FrameQuaternion(worldFrame);
      headOrientation.setYawPitchRoll(yaw, 0.75 * pitch, 0.0);
      behaviorHelper.requestHeadOrientationTrajectory(trajectoryTime, headOrientation, worldFrame, worldFrame);
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

      public static final Topic<Boolean> ExploreArea = topic("ExploreArea");
      public static final Topic<Boolean> RandomPoseUpdate = topic("RandomPoseUpdate");
      public static final Topic<Boolean> DoSlam = topic("DoSlam");
      public static final Topic<Boolean> ClearMap = topic("ClearMap");
      public static final Topic<PlanarRegionsListMessage> ConcatenatedMap = topic("ConcatenatedMap");
      public static final Topic<Point3D> ObservationPosition = topic("ObservationPosition");
      public static final Topic<ArrayList<BoundingBox3D>> ExplorationBoundingBoxes = topic("ExplorationBoundingBoxes");
      public static final Topic<ArrayList<Point3D>> PotentialPointsToExplore = topic("PotentialPointsToExplore");
      public static final Topic<Point3D> FoundBodyPathTo = topic("FoundBodyPathTo");
      public static final Topic<Point3D> PlanningToPosition = topic("PlanningToPosition");
      public static final Topic<Boolean> DrawMap = topic("DrawMap");
      public static final Topic<Boolean> ClearPlanarRegions = topic("ClearPlanarRegions");
      public static final Topic<TemporaryPlanarRegionMessage> AddPlanarRegionToMap = topic("AddPlanarRegionToMap");
      public static final Topic<TemporaryConvexPolygon2DMessage> AddPolygonToPlanarRegion = topic("AddPolygonToPlanarRegion");
      public static final Topic<ExploreAreaBehaviorState> CurrentState = topic("CurrentState");

      private static final <T> Topic<T> topic(String name)
      {
         return RootCategory.child(ExploreAreaTheme).topic(apiFactory.createTypedTopicTheme(name));
      }

      public static final MessagerAPI create()
      {
         return apiFactory.getAPIAndCloseFactory();
      }
   }
}
