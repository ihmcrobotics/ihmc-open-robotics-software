package us.ihmc.footstepPlanning.ui.components;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.FootstepDataMessage;
import controller_msgs.msg.dds.HeightMapMessage;
import javafx.animation.AnimationTimer;
import map_sense.RawGPUPlanarRegionList;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersReadOnly;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.ui.components.collision.CollidingScanRegionFilter;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.ihmcPerception.depthData.CollisionBoxProvider;
import us.ihmc.ihmcPerception.depthData.CollisionShapeTester;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.messager.Messager;
import us.ihmc.pathPlanning.visibilityGraphs.parameters.DefaultVisibilityGraphParameters;
import us.ihmc.robotEnvironmentAwareness.updaters.GPUPlanarRegionUpdater;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import java.net.URI;
import java.util.ArrayList;
import java.util.List;
import java.util.UUID;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Consumer;

public class HeightMapNavigationUpdater extends AnimationTimer
{
   private final Messager messager;
   private final Stopwatch stopwatch = new Stopwatch();
   private static final double replanDelay = 0.8;
   private static final double swingDuration = 1.6;
   private static final RobotSide initialStanceSide = RobotSide.RIGHT;

   private static final RigidBodyTransform transformChestToL515DepthCamera = new RigidBodyTransform();
   static
   {
      // TODO: Move this stuff to a file so it can be tuned and saved
      transformChestToL515DepthCamera.setIdentity();
      transformChestToL515DepthCamera.getTranslation().set(0.275000, 0.052000, 0.140000);
      transformChestToL515DepthCamera.getRotation().setYawPitchRoll(0.010000, 1.151900, 0.045000);
   }

   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));

   private final AtomicReference<Boolean> startHeightMapNavigation;
   private final AtomicReference<Boolean> stopHeightMapNavigation;
   private final AtomicBoolean firstStep = new AtomicBoolean();
   private final AtomicBoolean firstTickInStepState = new AtomicBoolean();

   private final AtomicReference<Point3D> goalPosition;
   private final AtomicReference<Quaternion> goalOrientation;
   private final AtomicReference<HeightMapMessage> heightMapMessage;
   private HeightMapMessage heightMapUsedForPlanning;

   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();

   private final FootstepPlannerRequest request = new FootstepPlannerRequest();
   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
   private RobotSide lastStepSide;
   private long previousStepMessageId = 0;
   private final Pose3D lastStepPose = new Pose3D();
   private final List<Pose3D> steps = new ArrayList<>();
   private final List<Point3D> bodyPathUnsmoothed = new ArrayList<>();

   private final AtomicReference<PlanarRegionsList> planarRegions = new AtomicReference<>();
   private final AtomicReference<State> currentState = new AtomicReference<>();
   private final AtomicBoolean executeRequested = new AtomicBoolean();
   private final AtomicBoolean replanRequested = new AtomicBoolean();
   private final AtomicBoolean writeLog = new AtomicBoolean();
   private final FootstepPlanningModule planningModule;
   private final FootstepPlannerLogger logger;
   private final HumanoidReferenceFrames referenceFrames;
   private final ReferenceFrame steppingFrame;
   private final SideDependentList<ConvexPolygon2D> footPolygons;

   private final URI rosuri = NetworkParameters.getROSURI();
   private RosMainNode ros1Node;
   private CollidingScanRegionFilter collisionFilter;

   private enum State
   {
      WAITING_TO_START,
      PLAN_BODY_PATH,
      PLAN_A_STEP,
      APPROVE_AND_SEND,
      WAIT_A_BIT
   }

   public HeightMapNavigationUpdater(Messager messager,
                                     FootstepPlannerParametersBasics footstepPlannerParameters,
                                     WalkingControllerParameters walkingControllerParameters,
                                     SideDependentList<List<Point2D>> defaultContactPoints,
                                     FullHumanoidRobotModel fullHumanoidRobotModel,
                                     CollisionBoxProvider collisionBoxProvider)
   {
      this.messager = messager;
      this.referenceFrames = new HumanoidReferenceFrames(fullHumanoidRobotModel);

      startHeightMapNavigation = messager.createInput(FootstepPlannerMessagerAPI.StartHeightMapNavigation, false);
      stopHeightMapNavigation = messager.createInput(FootstepPlannerMessagerAPI.StopHeightMapNavigation, false);
      goalPosition = messager.createInput(FootstepPlannerMessagerAPI.GoalMidFootPosition);
      goalOrientation = messager.createInput(FootstepPlannerMessagerAPI.GoalMidFootOrientation);
      heightMapMessage = messager.createInput(FootstepPlannerMessagerAPI.HeightMapData);

      footPolygons = new SideDependentList<>(side ->
                                             {
                                                ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
                                                defaultContactPoints.get(side).forEach(defaultFoothold::addVertex);
                                                defaultFoothold.update();
                                                return defaultFoothold;
                                             });

      planningModule = new FootstepPlanningModule("HeightMap", new DefaultVisibilityGraphParameters(), footstepPlannerParameters, new DefaultSwingPlannerParameters(), walkingControllerParameters, footPolygons, null);
      logger = new FootstepPlannerLogger(planningModule);
      planningModule.addCustomTerminationCondition((plannerTime, iterations, bestFinalStep, bestSecondToLastStep, bestPathSize) -> iterations > 1);

      messager.registerTopicListener(FootstepPlannerMessagerAPI.ApproveStep, executeRequested::set);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.ReplanStep, replanRequested::set);
      messager.registerTopicListener(FootstepPlannerMessagerAPI.WriteHeightMapLog, writeLog::set);

      currentState.set(State.WAITING_TO_START);

      ros1Node = RosTools.createRosNode(rosuri, "height_map_navigator");
      createROS1Callback(RosTools.MAPSENSE_REGIONS, ros1Node, this::handleRegions);
      ros1Node.execute();

      steppingFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("steppingCamera", referenceFrames.getChestFrame(), transformChestToL515DepthCamera);

      CollisionShapeTester shapeTester = new CollisionShapeTester();
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullHumanoidRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullHumanoidRobotModel.getPelvis(), shin, joints);
         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
      }
      collisionFilter = new CollidingScanRegionFilter(shapeTester);
   }

   public static AbstractRosTopicSubscriber<RawGPUPlanarRegionList> createROS1Callback(String topic, RosNodeInterface ros1Node, Consumer<RawGPUPlanarRegionList> callback)
   {
      AbstractRosTopicSubscriber<RawGPUPlanarRegionList> subscriber = new AbstractRosTopicSubscriber<RawGPUPlanarRegionList>(RawGPUPlanarRegionList._TYPE)
      {
         @Override
         public void onNewMessage(RawGPUPlanarRegionList rawGPUPlanarRegionList)
         {
            callback.accept(rawGPUPlanarRegionList);
         }
      };
      ros1Node.attachSubscriber(topic, subscriber);
      return subscriber;
   }

   private static final RigidBodyTransform zForwardXRightToZUpXForward = new RigidBodyTransform();
   static
   {
      zForwardXRightToZUpXForward.appendPitchRotation(Math.PI / 2.0);
      zForwardXRightToZUpXForward.appendYawRotation(-Math.PI / 2.0);
   }

   private final AtomicBoolean isProcessingRegions = new AtomicBoolean();
   private int uiUpdateCounter = 0;
   private final int uiUpdateRate = 40;

   private void handleRegions(RawGPUPlanarRegionList rawGPUPlanarRegionList)
   {
      if (!isProcessingRegions.getAndSet(true))
      {
         executorService.submit(() ->
                                {
                                   uiUpdateCounter++;
                                   if (rawGPUPlanarRegionList.getNumOfRegions() == 0)
                                   {
                                      return;
                                   }

                                   referenceFrames.updateFrames();
                                   steppingFrame.update();

                                   PlanarRegionsList planarRegionsList = gpuPlanarRegionUpdater.generatePlanarRegions(rawGPUPlanarRegionList);
                                   planarRegionsList.applyTransform(zForwardXRightToZUpXForward);
                                   planarRegionsList.applyTransform(steppingFrame.getTransformToWorldFrame());

                                   collisionFilter.update();
                                   gpuPlanarRegionUpdater.filterCollidingPlanarRegions(planarRegionsList, collisionFilter);
                                   this.planarRegions.set(planarRegionsList);

                                   if (uiUpdateCounter == uiUpdateRate)
                                   {
                                      messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegionsList);
                                      uiUpdateCounter = 0;
                                   }

                                   isProcessingRegions.set(false);
                                });
      }
   }

   @Override
   public void handle(long l)
   {
      // Execute steps
      if (stopHeightMapNavigation.getAndSet(false))
      {
         LogTools.error("Stop requested");
         currentState.set(State.WAITING_TO_START);
         writeLog();
         reset();
         return;
      }

      if (writeLog.getAndSet(false))
      {
         writeLog();
         reset();
         return;
      }

      if (currentState.get() == State.WAITING_TO_START)
      {
         if (!startHeightMapNavigation.getAndSet(false))
         {
            return;
         }

         if (goalPosition.get() == null || goalOrientation.get() == null)
         {
            LogTools.error("Need to set goal before starting.");
            return;
         }
         if (heightMapMessage.get() == null)
         {
            LogTools.error("Height map not received.");
            return;
         }

         reset();
         currentState.set(State.PLAN_BODY_PATH);
      }

      if (currentState.get() == State.PLAN_BODY_PATH)
      {
         // Plan body path
         setStartFootPosesToCurrent();

         heightMapUsedForPlanning = heightMapMessage.get();
         request.setHeightMapMessage(heightMapUsedForPlanning);

         Pose3D goalPose = new Pose3D(goalPosition.get(), goalOrientation.get());
         request.setGoalFootPoses(0.2, goalPose);
         request.setPlanBodyPath(true);
         request.setPerformAStarSearch(false);

         LogTools.info("Starting plan");
         FootstepPlannerOutput output = planningModule.handleRequest(request);
         LogTools.info("Finished plan, logging session.");
         logger.logSession();

         if (output.getBodyPathPlanningResult() != BodyPathPlanningResult.FOUND_SOLUTION)
         {
            LogTools.error("Body path failed to find solution");
            currentState.set(State.WAITING_TO_START);
            reset();
            return;
         }

         request.getBodyPathWaypoints().clear();
         for (int i = 0; i < output.getBodyPath().size(); i++)
         {
            request.getBodyPathWaypoints().add(new Pose3D(output.getBodyPath().get(i)));
         }

         bodyPathUnsmoothed.clear();
         bodyPathUnsmoothed.addAll(output.getBodyPathUnsmoothed());

         messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, request.getBodyPathWaypoints());
         currentState.set(State.PLAN_A_STEP);
         setStartFootPosesToCurrent();
         stopwatch.start();

         RobotSide stepSide = lastStepSide.getOppositeSide();
         request.setRequestedInitialStanceSide(stepSide);
      }

      if (currentState.get() == State.PLAN_A_STEP)
      {
         if (planarRegions.get() == null)
         {
            LogTools.error("No planar regions");
            currentState.set(State.WAITING_TO_START);
            reset();
            return;
         }

         boolean firstStep = this.firstStep.getAndSet(false);
         boolean planAStep = firstStep || stopwatch.lapElapsed() > replanDelay;
         if (!planAStep)
         {
            return;
         }

         request.setPlanBodyPath(false);
         request.setPerformAStarSearch(true);
         request.setHeightMapMessage(null);
         request.setPlanarRegionsList(planarRegions.get());
         request.setRequestedInitialStanceSide(lastStepSide);

         LogTools.info("Planning step");
         FootstepPlannerOutput output = planningModule.handleRequest(request);
         FootstepPlanningResult result = output.getFootstepPlanningResult();
         LogTools.info(" \t " + result);
         stopwatch.lap();

         footstepDataListMessage.getFootstepDataList().clear();
         PlannedFootstep footstep = planningModule.getOutput().getFootstepPlan().getFootstep(0);
         FootstepDataMessage footstepDataMessage = footstepDataListMessage.getFootstepDataList().add();
         footstepDataMessage.getLocation().set(footstep.getFootstepPose().getPosition());
         footstepDataMessage.getOrientation().set(footstep.getFootstepPose().getOrientation());
         messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepDataListMessage);

         if ((result != FootstepPlanningResult.FOUND_SOLUTION && result != FootstepPlanningResult.HALTED) || output.getFootstepPlan().getNumberOfSteps() == 0)
         {
            logger.logSession();
            return;
         }

         currentState.set(State.APPROVE_AND_SEND);
      }

      if (currentState.get() == State.APPROVE_AND_SEND)
      {
         if (replanRequested.getAndSet(false))
         {
            // keep same request
            currentState.set(State.PLAN_A_STEP);
            return;
         }

         if (executeRequested.getAndSet(false))
         {
            ExecutionMode executionMode = ExecutionMode.QUEUE;

            footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
            long messageId = UUID.randomUUID().getLeastSignificantBits();
            footstepDataListMessage.getQueueingProperties().setMessageId(messageId);
            footstepDataListMessage.getQueueingProperties().setPreviousMessageId(previousStepMessageId);
            messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanToRobot, footstepDataListMessage);

            firstStep.set(false);
            lastStepSide = lastStepSide.getOppositeSide();
            lastStepPose.set(footstepDataListMessage.getFootstepDataList().get(0).getLocation(), footstepDataListMessage.getFootstepDataList().get(0).getOrientation());
            steps.add(new Pose3D(lastStepPose));
            previousStepMessageId = messageId;
            logger.logSession();
            stopwatch.lap();
            setStartFootPosesBasedOnLastCommandedStep();
         }
      }

      if (currentState.get() == State.WAIT_A_BIT)
      {
         if (stopwatch.lapElapsed() > replanDelay)
         {
            currentState.set(State.PLAN_A_STEP);
            return;
         }
      }
   }

   private void setStartFootPosesToCurrent()
   {
      referenceFrames.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         FramePose3D foot = new FramePose3D(referenceFrames.getSoleFrame(robotSide));
         foot.changeFrame(ReferenceFrame.getWorldFrame());
         request.setStartFootPose(robotSide, foot);
      }
   }

   private void setStartFootPosesBasedOnLastCommandedStep()
   {
      referenceFrames.updateFrames();

      for (RobotSide robotSide : RobotSide.values)
      {
         if (robotSide == lastStepSide)
         {
            request.setStartFootPose(robotSide, lastStepPose);
         }
         else
         {
            FramePose3D foot = new FramePose3D(referenceFrames.getSoleFrame(robotSide));
            foot.changeFrame(ReferenceFrame.getWorldFrame());
            request.setStartFootPose(robotSide, foot);
         }
      }
   }

   private void writeLog()
   {
      // pack all data into one log
      FootstepPlannerOutput output = planningModule.getOutput();
      output.getBodyPath().clear();
      for (int i = 0; i < output.getBodyPath().size(); i++)
      {
         output.getBodyPath().add(new Pose3D(request.getBodyPathWaypoints().get(i)));
      }

      RobotSide side = initialStanceSide.getOppositeSide();
      for (int i = 0; i < steps.size(); i++)
      {
         PlannedFootstep footstep = output.getFootstepPlan().addFootstep(side, new FramePose3D(steps.get(i)));
         footstep.getFoothold().set(footPolygons.get(side));
         side = side.getOppositeSide();
      }

      request.setHeightMapMessage(heightMapUsedForPlanning);
      logger.logSession();
   }

   private void reset()
   {
      stopHeightMapNavigation.set(false);
      firstStep.set(true);
      firstTickInStepState.set(true);
      previousStepMessageId = 0;
      lastStepSide = initialStanceSide;
      stopwatch.start();
      steps.clear();
      replanRequested.set(false);
      executeRequested.set(false);
   }

   public void destroy()
   {
      ros1Node.shutdown();
      ros1Node = null;
   }
}
