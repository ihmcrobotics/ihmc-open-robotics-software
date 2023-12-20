package us.ihmc.footstepPlanning.ui.components;

import controller_msgs.msg.dds.FootstepDataListMessage;
import perception_msgs.msg.dds.HeightMapMessage;
import javafx.animation.AnimationTimer;
import map_sense.RawGPUPlanarRegionList;
import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.communication.packets.ExecutionMode;
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
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.DefaultSwingPlannerParameters;
import us.ihmc.footstepPlanning.tools.PlanarRegionToHeightMapConverter;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.perception.depthData.CollisionBoxProvider;
import us.ihmc.perception.depthData.CollisionShapeTester;
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
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapMessageTools;
import us.ihmc.utilities.ros.RosNodeInterface;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

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
   private final AtomicBoolean reconnectRos1Node = new AtomicBoolean();

   private final AtomicReference<Point3D> goalPosition;
   private final AtomicReference<Quaternion> goalOrientation;
   private final AtomicReference<HeightMapMessage> heightMapMessage;
   private HeightMapData heightMapUsedForPlanning;

   private final GPUPlanarRegionUpdater gpuPlanarRegionUpdater = new GPUPlanarRegionUpdater();

   private final FootstepPlannerRequest request = new FootstepPlannerRequest();

   private long sequenceID = 0;
   private final FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
   private final FootstepDataListMessage footstepDataListMessageCache = new FootstepDataListMessage();
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
      messager.addTopicListener(FootstepPlannerMessagerAPI.PlanarRegionData, planarRegions::set);

      footPolygons = new SideDependentList<>(side ->
                                             {
                                                ConvexPolygon2D defaultFoothold = new ConvexPolygon2D();
                                                defaultContactPoints.get(side).forEach(defaultFoothold::addVertex);
                                                defaultFoothold.update();
                                                return defaultFoothold;
                                             });

      footstepPlannerParameters.setIdealFootstepLength(0.28);
      planningModule = new FootstepPlanningModule("HeightMap", new DefaultVisibilityGraphParameters(), new AStarBodyPathPlannerParameters(), footstepPlannerParameters, new DefaultSwingPlannerParameters(), walkingControllerParameters, footPolygons, null);
      logger = new FootstepPlannerLogger(planningModule);
      planningModule.addCustomTerminationCondition((plannerTime, iterations, bestFinalStep, bestSecondToLastStep, bestPathSize) -> iterations > 1);

      messager.addTopicListener(FootstepPlannerMessagerAPI.ApproveStep, executeRequested::set);
      messager.addTopicListener(FootstepPlannerMessagerAPI.ReplanStep, replanRequested::set);
      messager.addTopicListener(FootstepPlannerMessagerAPI.WriteHeightMapLog, writeLog::set);
      messager.addTopicListener(FootstepPlannerMessagerAPI.ResendLastStep, r -> messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanToRobot, footstepDataListMessageCache));
      messager.addTopicListener(FootstepPlannerMessagerAPI.ReconnectRos1Node, reconnectRos1Node::set);

      currentState.set(State.WAITING_TO_START);

      steppingFrame = ReferenceFrameTools.constructFrameWithChangingTransformToParent("steppingCamera", referenceFrames.getChestFrame(), transformChestToL515DepthCamera);

      CollisionShapeTester shapeTester = new CollisionShapeTester();
      for (RobotSide robotSide : RobotSide.values)
      {
         List<JointBasics> joints = new ArrayList<>();
         RigidBodyBasics shin = fullHumanoidRobotModel.getFoot(robotSide).getParentJoint().getPredecessor().getParentJoint().getPredecessor();
         MultiBodySystemTools.collectJointPath(fullHumanoidRobotModel.getPelvis(), shin, joints);
         joints.forEach(joint -> shapeTester.addJoint(collisionBoxProvider, joint));
      }
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

         heightMapUsedForPlanning = HeightMapMessageTools.unpackMessage(heightMapMessage.get());
         request.setHeightMapData(heightMapUsedForPlanning);

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

         messager.submitMessage(FootstepPlannerMessagerAPI.BodyPathData, Pair.of(request.getBodyPathWaypoints(), null));
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
         request.setHeightMapData(HeightMapMessageTools.unpackMessage(PlanarRegionToHeightMapConverter.convertFromPlanarRegionsToHeightMap(planarRegions.get())));
         request.setRequestedInitialStanceSide(lastStepSide);

         LogTools.info("Planning step");
         FootstepPlannerOutput output = planningModule.handleRequest(request);
         FootstepPlanningResult result = output.getFootstepPlanningResult();
         LogTools.info(" \t " + result);
         stopwatch.lap();

         footstepDataListMessage.getFootstepDataList().clear();

//         if (footstep.getFoothold().isEmpty())
//         {
//            ConvexPolygon2D footPolygon = footPolygons.get(footstep.getRobotSide());
//            for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
//            {
//               footstepDataMessage.getPredictedContactPoints2d().add().set(footPolygon.getVertex(i));
//            }
//         }
//         else
//         {
//            ConvexPolygon2D footPolygon = footstep.getFoothold();
//            for (int i = 0; i < footPolygon.getNumberOfVertices(); i++)
//            {
//               footstepDataMessage.getPredictedContactPoints2d().add().set(footPolygon.getVertex(i));
//            }
//         }

         logger.logSession();

         if ((result != FootstepPlanningResult.FOUND_SOLUTION && result != FootstepPlanningResult.HALTED) || output.getFootstepPlan().getNumberOfSteps() == 0)
         {
            return;
         }

         messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanResponse, footstepDataListMessage);
         PlannedFootstep footstep = planningModule.getOutput().getFootstepPlan().getFootstep(0);
         footstep.setSwingDuration(1.9);

         footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());

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
            LogTools.info("Executing step");
            ExecutionMode executionMode = ExecutionMode.OVERRIDE;

            footstepDataListMessage.getQueueingProperties().setExecutionMode(executionMode.toByte());
            long messageId = UUID.randomUUID().getLeastSignificantBits();
            footstepDataListMessage.getQueueingProperties().setMessageId(messageId);
            footstepDataListMessage.getQueueingProperties().setPreviousMessageId(previousStepMessageId);

            // sequence ids
            footstepDataListMessage.getQueueingProperties().setMessageId(sequenceID);
            footstepDataListMessage.setSequenceId(sequenceID);
            footstepDataListMessage.getFootstepDataList().get(0).setSequenceId(sequenceID);

            messager.submitMessage(FootstepPlannerMessagerAPI.FootstepPlanToRobot, footstepDataListMessage);
            footstepDataListMessageCache.set(footstepDataListMessage);
            sequenceID++;

            firstStep.set(false);
            lastStepSide = lastStepSide.getOppositeSide();
            lastStepPose.set(footstepDataListMessage.getFootstepDataList().get(0).getLocation(), footstepDataListMessage.getFootstepDataList().get(0).getOrientation());
            steps.add(new Pose3D(lastStepPose));
            previousStepMessageId = messageId;
            logger.logSession();
            stopwatch.lap();
            setStartFootPosesBasedOnLastCommandedStep();
            currentState.set(State.WAIT_A_BIT);
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

      request.setHeightMapData(heightMapUsedForPlanning);
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
}
