package us.ihmc.behaviors.activeMapping.ContinuousHikingStateMachine;

import behavior_msgs.msg.dds.ContinuousHikingCommandMessage;
import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.QueuedFootstepStatusMessage;
import ihmc_common_msgs.msg.dds.PoseListMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import org.jetbrains.annotations.NotNull;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.activeMapping.ControllerFootstepQueueMonitor;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.FootstepPlan;
import us.ihmc.footstepPlanning.FootstepPlannerOutput;
import us.ihmc.footstepPlanning.FootstepPlannerRequest;
import us.ihmc.footstepPlanning.FootstepPlanningModule;
import us.ihmc.footstepPlanning.PlannedFootstep;
import us.ihmc.footstepPlanning.communication.ContinuousHikingAPI;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.DiscreteFootstep;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerParametersBasics;
import us.ihmc.footstepPlanning.swing.SwingPlannerType;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

import java.util.List;
import java.util.concurrent.atomic.AtomicReference;
import java.util.function.Supplier;

public class JustWaitState implements State
{
   @NotNull
   private final ROS2Helper ros2Helper;
   @NotNull
   private final ROS2SyncedRobotModel syncedRobot;
   private final AtomicReference<ContinuousHikingCommandMessage> commandMessage;
   private final ControllerFootstepQueueMonitor controllerQueueMonitor;
   private final FootstepPlannerEnvironmentHandler environmentHandler;
   private boolean isDone;
   private final FootstepPlanningModule footstepPlanner;
   private final DefaultFootstepPlannerParametersBasics footstepPlannerParameters;
   private final SwingPlannerParametersBasics swingPlannerParameters;
   private final Supplier<HeightMapData> heightMapData;
   private final Supplier<TerrainMapData> terrainMapData;

   private final ROS2Topic<FootstepDataListMessage> controllerFootstepDataTopic;
   private final FramePose3D midFeetZUpPose = new FramePose3D();
   private final MovingReferenceFrame midFeetZUpFrame;
   private final FramePose3D startPose = new FramePose3D();
   private final FootstepSnapAndWiggler footstepSnapAndWiggler;

   public JustWaitState(DRCRobotModel robotModel,
                        ROS2Helper ros2Helper,
                        ROS2SyncedRobotModel syncedRobot,
                        AtomicReference<ContinuousHikingCommandMessage> commandMessage,
                        ControllerFootstepQueueMonitor controllerQueueMonitor,
                        DefaultFootstepPlannerParametersBasics footstepPlannerParameters,
                        SwingPlannerParametersBasics swingPlannerParameters,
                        Supplier<HeightMapData> heightMapData,
                        Supplier<TerrainMapData> terrainMapData)
   {
      this.ros2Helper = ros2Helper;
      this.syncedRobot = syncedRobot;
      this.commandMessage = commandMessage;
      this.controllerQueueMonitor = controllerQueueMonitor;

      footstepPlanner = FootstepPlanningModuleLauncher.createModule(robotModel);
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.swingPlannerParameters = swingPlannerParameters;
      this.heightMapData = heightMapData;
      this.terrainMapData = terrainMapData;

      environmentHandler = new FootstepPlannerEnvironmentHandler();
      footstepSnapAndWiggler = new FootstepSnapAndWiggler(PlannerTools.createDefaultFootPolygons(), footstepPlannerParameters, environmentHandler);

      midFeetZUpFrame = syncedRobot.getReferenceFrames().getMidFeetZUpFrame();

      controllerFootstepDataTopic = HumanoidControllerAPI.getTopic(FootstepDataListMessage.class, "Nadia");
      ros2Helper.createPublisher(controllerFootstepDataTopic);

      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.ROTATE_GOAL_FOOTSTEPS, this::planToGoal);
      ros2Helper.subscribeViaCallback(ContinuousHikingAPI.SQUARE_UP_STEP, this::squareUpStep);
   }

   @Override
   public void onEntry()
   {
      ContinuousHikingCommandMessage continuousHikingCommandMessage = commandMessage.get();

      if (continuousHikingCommandMessage.getSquareUpToGoal())
      {
         squareUpStep();
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
                                                                                           LogTools.info("Yes this queue is not empty");
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
                                  footstepPlannerRequest.setHeightMapData(heightMapData.get());

                                  footstepPlannerRequest.setSnapGoalSteps(true);
                                  footstepPlannerRequest.setPlanBodyPath(true);

                                  FramePose3D goalFramePose = new FramePose3D();
                                  goalFramePose.interpolate(leftFootPose, rightFootPose, 0.5);

                                  Pose3DReadOnly goalPose = new Pose3D(goalFramePose.getPosition(), goalFramePose.getOrientation());

                                  midFeetZUpPose.setToZero(midFeetZUpFrame);
                                  midFeetZUpPose.changeFrame(ReferenceFrame.getWorldFrame());
                                  startPose.setToZero(midFeetZUpFrame);
                                  startPose.changeFrame(ReferenceFrame.getWorldFrame());
                                  startPose.getOrientation().set(goalPose.getOrientation());
                                  footstepPlannerRequest.getBodyPathWaypoints().add(midFeetZUpPose);
                                  footstepPlannerRequest.getBodyPathWaypoints().add(startPose);
                                  footstepPlannerRequest.getBodyPathWaypoints().add(goalPose);

                                  FootstepPlannerOutput plannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);

                                  FootstepPlan newestFootstepPlan = null;

                                  if (plannerOutput != null)
                                  {
                                     newestFootstepPlan = plannerOutput.getFootstepPlan();
                                  }

                                  FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
                                  footstepDataListMessage.setDefaultSwingDuration(0.8);
                                  footstepDataListMessage.setDefaultTransferDuration(0.4);
                                  footstepDataListMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_QUEUE);

                                  for (int i = 0; i < footstepPlanner.getOutput().getFootstepPlan().getNumberOfSteps(); i++)
                                  {
                                     assert newestFootstepPlan != null;
                                     PlannedFootstep footstep = newestFootstepPlan.getFootstep(i);
                                     footstep.limitFootholdVertices();
                                     footstepDataListMessage.getFootstepDataList().add().set(footstep.getAsMessage());
                                  }

                                  ros2Helper.publish(controllerFootstepDataTopic, footstepDataListMessage);
                               }, "PlanToGoalThread");
   }

   private final FramePose3D tempFramePose = new FramePose3D();

   public void squareUpStep()
   {
      environmentHandler.setHeightMap(heightMapData.get());
      environmentHandler.setTerrainMapData(terrainMapData.get());

      FramePose3DReadOnly lastStepInQueue;
      PlannedFootstep squareUpStep = null;

      if (controllerQueueMonitor.getNumberOfIncompleteFootsteps() > 0)
      {
         lastStepInQueue = controllerQueueMonitor.getLastFootstepInQueue();
         List<QueuedFootstepStatusMessage> controllerFootstepQueue = controllerQueueMonitor.getControllerFootstepQueue();

         RobotSide robotSide = RobotSide.fromByte(controllerFootstepQueue.get(controllerFootstepQueue.size() - 1).getRobotSide());

         if (robotSide == RobotSide.LEFT)
         {
            tempFramePose.set(lastStepInQueue);
            tempFramePose.getTranslation().addY(-footstepPlannerParameters.getIdealFootstepWidth());
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

            squareUpStep = new PlannedFootstep(robotSide.getOppositeSide(), tempFramePose);
         }
         else if (robotSide == RobotSide.RIGHT)
         {
            tempFramePose.set(lastStepInQueue);
            tempFramePose.getTranslation().addY(footstepPlannerParameters.getIdealFootstepWidth());
            tempFramePose.changeFrame(ReferenceFrame.getWorldFrame());

            squareUpStep = new PlannedFootstep(robotSide.getOppositeSide(), tempFramePose);
         }
      }
      else
      {
         ReferenceFrame leftFootFrame = syncedRobot.getReferenceFrames().getFootFrame(RobotSide.LEFT);
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

      DiscreteFootstep footstep = new DiscreteFootstep(squareUpStep.getFootstepPose().getX(), squareUpStep.getFootstepPose().getY());
      footstepSnapAndWiggler.snapFootstep(footstep);

      squareUpStep.getFootstepPose().setZ(0.0);
      squareUpStep.getFootstepPose().applyTransform(footstep.getSnapData().getSnapTransform());

      FootstepDataListMessage footstepDataListMessage = new FootstepDataListMessage();
      footstepDataListMessage.setDefaultSwingDuration(0.8);
      footstepDataListMessage.setDefaultTransferDuration(0.4);
      footstepDataListMessage.getQueueingProperties().setExecutionMode(QueueableMessage.EXECUTION_MODE_QUEUE);
      footstepDataListMessage.getFootstepDataList().add().set(squareUpStep.getAsMessage());

      ros2Helper.publish(controllerFootstepDataTopic, footstepDataListMessage);
   }
}
