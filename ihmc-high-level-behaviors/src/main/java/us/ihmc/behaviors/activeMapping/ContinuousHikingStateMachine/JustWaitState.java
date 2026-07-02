package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.ContinuousHikingCommandMessage;
import controller_msgs.FootstepDataListMessage;
import controller_msgs.QueuedFootstepStatusMessage;
import ihmc_common_msgs.PoseListMessage;
import ihmc_common_msgs.QueueableMessage;
import org.jetbrains.annotations.NotNull;
import std_msgs.Float32;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.activeMapping.ActiveMappingParameterToolBox;
import us.ihmc.behaviors.activeMapping.ContinuousHikingParameters;
import us.ihmc.behaviors.activeMapping.TerrainPlanningDebugger;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.fastddsjava.cdr.idl.IDLObjectSequence;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.EnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.humanoidRobotics.communication.ControllerFootstepQueueMonitor;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.stateMachine.core.State;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class JustWaitState implements State
{
   @NotNull
   private final ROS2SyncedRobotModel syncedRobot;
   private final AtomicReference<ContinuousHikingCommandMessage> commandMessage;
   private final ControllerFootstepQueueMonitor controllerQueueMonitor;
   private final ContinuousHikingParameters continuousHikingParameters;
   private final FootstepPlannerLogger logger;
   private boolean isDone;
   private final FootstepPlanningModule footstepPlanner;
   private final TerrainPlanningDebugger terrainPlanningDebugger;
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;
   private final Supplier<EnvironmentHandler> environmentHandlerSupplier;

   private final ROS2Topic<FootstepDataListMessage> controllerFootstepDataTopic;
   private final ROS2Publisher<FootstepDataListMessage> footstepPublisher;
   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private final MovingReferenceFrame midFeetZUpFrame;
   private final FramePose3D startPose = new FramePose3D();
   private final FootstepSnapAndWiggler footstepSnapAndWiggler;
   private boolean prepareSquareUpStep;
   private AtomicBoolean useEnvironmentData = new AtomicBoolean(false);

   public JustWaitState(DRCRobotModel robotModel,
                        ROS2Node ros2Node,
                        ROS2SyncedRobotModel syncedRobot,
                        AtomicReference<ContinuousHikingCommandMessage> commandMessage,
                        ControllerFootstepQueueMonitor controllerQueueMonitor,
                        TerrainPlanningDebugger terrainPlanningDebugger,
                        ActiveMappingParameterToolBox activeMappingParameterToolBox,
                        Supplier<EnvironmentHandler> environmentHandlerSupplier)
   {
      this.syncedRobot = syncedRobot;
      this.commandMessage = commandMessage;
      this.controllerQueueMonitor = controllerQueueMonitor;

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      this.terrainPlanningDebugger = terrainPlanningDebugger;
      logger = new FootstepPlannerLogger(footstepPlanner);

      this.continuousHikingParameters = activeMappingParameterToolBox.getContinuousHikingParameters();
      this.footstepPlannerParameters = activeMappingParameterToolBox.getFootstepPlannerParameters();
      this.swingPlannerParameters = activeMappingParameterToolBox.getSwingPlannerParameters();
      this.environmentHandlerSupplier = environmentHandlerSupplier;

      footstepSnapAndWiggler = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), this.footstepPlannerParameters, environmentHandlerSupplier.get());

      midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, robotModel.getSimpleRobotName());
      footstepPublisher = ros2Node.createPublisher(controllerFootstepDataTopic);

      ros2Node.createSubscriptionSampler(ContinuousHikingAPI.ROTATE_90_DEGREES, this::rotate90Degrees);
      ros2Node.createSubscriptionSampler(ContinuousHikingAPI.ROTATE_GOAL_FOOTSTEPS, this::planToGoal);
      ros2Node.createSubscriptionSampler(ContinuousHikingAPI.SQUARE_UP_STEP, sample -> squareUpStep());
   }

   @Override
   public void onEntry()
   {
      ContinuousHikingCommandMessage continuousHikingCommandMessage = commandMessage.get();

      if (continuousHikingCommandMessage.getSquareUpToGoal() && !controllerQueueMonitor.getControllerFootstepQueue().isEmpty())
      {
         squareUpStep();
         terrainPlanningDebugger.resetVisualizationForUIPublisher();
      }

      isDone = false;
   }

   @Override
   public void doAction(double timeInState)
   {
      if (controllerQueueMonitor.getControllerFootstepQueue().isEmpty())
      {
         isDone = true;
      }
   }

   @Override
   public void onExit(double timeInState)
   {
   }

   @Override
   public boolean isDone(double timeInState)
   {
      return isDone;
   }

   public void rotate90Degrees(Float32 float32Message)
   {
      float rotationRadians = float32Message.getData();

      MovingReferenceFrame midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();
      FramePose3D midFeetZUpPose = new FramePose3D(midFeetZUpFrame, midFeetZUpFrame.getTransformToWorldFrame());
      SideDependentList<FramePose3D> goalPoses = new SideDependentList<>(new FramePose3D(syncedRobot.getReferenceFrames().getMidFeetZUpFrame()),
                                                                         new FramePose3D(syncedRobot.getReferenceFrames().getMidFeetZUpFrame()));

      if (!controllerQueueMonitor.getControllerFootstepQueue().isEmpty())
      {
         FramePose3DReadOnly lastFootstepInQueue = controllerQueueMonitor.getLastFootstepInQueue();
         RobotSide lastFootstepSide = RobotSide.fromByte(controllerQueueMonitor.getControllerFootstepQueue()
                                                                               .get(controllerQueueMonitor.getControllerFootstepQueue().size() - 1)
                                                                               .getRobotSide());

         FramePose3D tempMidFeetPose = new FramePose3D();
         tempMidFeetPose.set(lastFootstepInQueue);

         if (lastFootstepSide == RobotSide.LEFT)
         {
            tempMidFeetPose.appendTranslation(0.0, -0.12, 0.0);
         }
         else
         {
            tempMidFeetPose.appendTranslation(0.0, 0.12, 0.0);
         }

         midFeetZUpPose.set(midFeetZUpFrame, tempMidFeetPose);
      }

      midFeetZUpPose.appendYawRotation(rotationRadians);
      goalPoses.get(RobotSide.RIGHT).set(midFeetZUpPose);
      goalPoses.get(RobotSide.RIGHT).changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      goalPoses.get(RobotSide.RIGHT).appendTranslation(0.0, -0.13, 0.0);

      goalPoses.get(RobotSide.LEFT).set(midFeetZUpPose);
      goalPoses.get(RobotSide.LEFT).changeFrame(syncedRobot.getReferenceFrames().getMidFeetZUpFrame());
      goalPoses.get(RobotSide.LEFT).appendTranslation(0.0, 0.13, 0.0);

      List<Pose3D> poses = new ArrayList<>();
      poses.add(new Pose3D(goalPoses.get(RobotSide.LEFT)));
      poses.add(new Pose3D(goalPoses.get(RobotSide.RIGHT)));

      PoseListMessage poseListMessage = new PoseListMessage();
      MessageTools.packPoseListMessage(poses, poseListMessage);

      // We assume the 90 degree turn is happening on flat ground, makes it easier
      // TODO snap footsteps under the robot, this is difficult cause of the noise in the height map causing inaccurate steps
      useEnvironmentData.set(false);
      planToGoal(poseListMessage);
   }

   public void planToGoal(PoseListMessage poseListMessage)
   {
      ThreadTools.startAThread(() ->
                               {
                                  List<Pose3D> poses = MessageTools.unpackPoseListMessage(poseListMessage);
                                  FramePose3D leftFootPose = new FramePose3D();
                                  FramePose3D rightFootPose = new FramePose3D();

                                  leftFootPose.set(poses.get(0));
                                  rightFootPose.set(poses.get(1));

                                  footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
                                  footstepPlanner.getSwingPlannerParameters().set(swingPlannerParameters);

                                  FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
                                  footstepPlannerRequest.setGoalFootPoses(leftFootPose, rightFootPose);
                                  footstepPlannerRequest.setSwingPlannerType(SwingPlannerType.NONE);

                                  footstepPlannerRequest.getStartFootPoses().forEach((side, pose3D) ->
                                                                                     {
                                                                                        FramePose3DReadOnly soleFramePose;
                                                                                        if (controllerQueueMonitor.getNumberOfIncompleteFootsteps() > 0)
                                                                                        {
                                                                                           // We pass in the opposite side because the method returns the footstep on the opposite side
                                                                                           soleFramePose = controllerQueueMonitor.getLastFootstepQueuedOnOppositeSide(
                                                                                                 side.getOppositeSide());
                                                                                        }
                                                                                        else
                                                                                        {
                                                                                           soleFramePose = syncedRobot.getFramePoseReadOnly(referenceFrames -> referenceFrames.getSoleFrame(
                                                                                                 side));
                                                                                        }
                                                                                        soleFramePose.get(pose3D);
                                                                                     });

                                  footstepPlannerRequest.setRequestedInitialStanceSide(RobotSide.LEFT);

                                  if (useEnvironmentData.get())
                                  {
                                     EnvironmentHandler environmentHandler = environmentHandlerSupplier.get();
                                     footstepPlannerRequest.setTerrainMapData(environmentHandler.getTerrainMapData());
                                     footstepPlannerRequest.setSnapGoalSteps(useEnvironmentData.get());
                                  }

                                  footstepPlannerRequest.setPlanBodyPath(false);

                                  FramePose3D goalFramePose = new FramePose3D();
                                  goalFramePose.interpolate(leftFootPose, rightFootPose, 0.5);

                                  Pose3DReadOnly goalPose = new Pose3D(goalFramePose.getPosition(), goalFramePose.getOrientation());

                                  midFeetZUpPose.setToZero(midFeetZUpFrame);
                                  midFeetZUpPose.changeFrame(ReferenceFrame.getWorldFrame());
                                  startPose.setToZero(midFeetZUpFrame);
                                  startPose.changeFrame(ReferenceFrame.getWorldFrame());
                                  startPose.getOrientation().set(goalPose.getOrientation());
                                  footstepPlannerRequest.getBodyPathWaypoints().add(startPose);
                                  footstepPlannerRequest.getBodyPathWaypoints().add(goalPose);

                                  FootstepPlannerOutput plannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);

                                  if (plannerOutput == null)
                                     return;

                                  FootstepPlan newestFootstepPlan = plannerOutput.getFootstepPlan();

                                  FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
                                  footstepDataListMessage.setDefaultSwingDuration(continuousHikingParameters.getNinetyDegreeTurnSwingTime());
                                  footstepDataListMessage.setDefaultTransferDuration(continuousHikingParameters.getNinetyDegreeTurnTransferTime());
                                  footstepDataListMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_QUEUE);
                                  footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);

                                  for (int i = 0; i < footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps(); i++)
                                  {
                                     PlannedFootstep footstep = newestFootstepPlan.getFootstep(i);
                                     footstep.limitFootholdVertices();
                                     footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());
                                  }

                                  if (!footstepDataListMessage.getFootstepDataList().isEmpty())
                                  {
                                     useEnvironmentData.set(true);
                                     logFootStePlan();
                                     footstepPublisher.publish(footstepDataListMessage);
                                  }
                                  else
                                  {
                                     LogTools.warn("Didn't have any steps to publish, try again :( :(");
                                  }
                               }, "PlanToGoalThread");
   }

   private final FramePose3D tempFramePose = new FramePose3D();

   public void squareUpStep()
   {
      FramePose3DReadOnly firstStepInQueue;
      PlannedFootstep squareUpStep = null;
      ReferenceFrame leftFootFrame = syncedRobot.getReferenceFrames().getFootFrame(RobotSide.LEFT);
      ReferenceFrame rightFootFrame = syncedRobot.getReferenceFrames().getFootFrame(RobotSide.RIGHT);

      if (controllerQueueMonitor.getNumberOfIncompleteFootsteps() > 0)
      {
         firstStepInQueue = controllerQueueMonitor.getFirstFootstepInQueue();
         IDLObjectSequence<QueuedFootstepStatusMessage> controllerFootstepQueue = controllerQueueMonitor.getControllerFootstepQueue();

         RobotSide robotSide = RobotSide.fromByte(controllerFootstepQueue.get(0).getRobotSide());

         tempFramePose.set(firstStepInQueue);

         if (robotSide == RobotSide.LEFT)
         {
            tempFramePose.changeFrame(leftFootFrame);
            tempFramePose.getTranslation().addY(-footstepPlannerParameters.getIdealFootstepWidth());
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

            squareUpStep = new PlannedFootstep(robotSide.getOppositeSide(), tempFramePose);
         }
         else if (robotSide == RobotSide.RIGHT)
         {
            tempFramePose.changeFrame(rightFootFrame);
            tempFramePose.getTranslation().addY(footstepPlannerParameters.getIdealFootstepWidth());
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

            squareUpStep = new PlannedFootstep(robotSide.getOppositeSide(), tempFramePose);
         }
      }
      else
      {
         FramePose3D rightFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                     syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT).getTransformToWorldFrame());
         rightFootPose.changeFrame(leftFootFrame);
         RobotSide furthestForwardFootstep = rightFootPose.getTranslationX() > 0 ? RobotSide.RIGHT : RobotSide.LEFT;
         MovingReferenceFrame furthestForwardSoleFrame = syncedRobot.getReferenceFrames().getSoleFrame(furthestForwardFootstep);

         tempFramePose.setToZero(furthestForwardSoleFrame);
         tempFramePose.getTranslation().addY(furthestForwardFootstep.negateIfLeftSide(footstepPlannerParameters.getIdealFootstepWidth()));
         tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

         squareUpStep = new PlannedFootstep(furthestForwardFootstep.getOppositeSide(), tempFramePose);
      }

      if (squareUpStep == null)
         return;

      //TODO, set the translation but don't worry about the orientation, it should be good enough to stand when stepping there.
      //      DiscreteFootstep footstep = new DiscreteFootstep(squareUpStep.getFootstepPose().getX(), squareUpStep.getFootstepPose().getY());
      //      footstepSnapAndWiggler.snapFootstep(footstep);
      //
      //      squareUpStep.getFootstepPose().setZ(0.0);
      //      squareUpStep.getFootstepPose().applyTransform(footstep.getSnapData().getSnapTransform());

      // Set Pitch and Roll to Zero, not expecting some crazy orientation
      //      ReferenceFrame referenceFrame = squareUpStep.getFootstepPose().getReferenceFrame();
      //      squareUpStep.getFootstepPose().changeFrame(ReferenceFrame.getWorldFrame());
      //      squareUpStep.getFootstepPose().getOrientation().setToPitchOrientation(-0.6);
      //      squareUpStep.getFootstepPose().getOrientation().setToRollOrientation(0.3);
      //      squareUpStep.getFootstepPose().getOrientation().setToYawOrientation(0.43);
      //      squareUpStep.getFootstepPose().changeFrame(referenceFrame);

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(continuousHikingParameters.getSwingTime());
      footstepDataListMessage.setDefaultTransferDuration(continuousHikingParameters.getTransferTime());
      footstepDataListMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_OVERRIDE);
      footstepDataListMessage.setOffsetFootstepsHeightWithExecutionError(true);
      footstepDataListMessage.getFootstepDataList().add().set(squareUpStep.getAsMessage());

      logFootStePlan();

      footstepPublisher.publish(footstepDataListMessage);
   }

   public void logFootStePlan()
   {
      ThreadTools.startAThread(() ->
                               {
                                  // In case logging footstep plans becomes a problem, we have this feature where we can not log plans if we want too
                                  if (continuousHikingParameters.getLogFootstepPlans())
                                  {
                                     logger.logSession();
                                  }
                               }, "Footstep Logger Thead");
   }
}
