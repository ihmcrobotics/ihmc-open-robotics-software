package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.ActionNodeExecutor;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.Timer;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class PelvisHeightPitchActionExecutor extends ActionNodeExecutor<PelvisHeightPitchActionState, PelvisHeightPitchActionDefinition>
{
   public static final double POSITION_TOLERANCE = 0.15;
   public static final double ORIENTATION_TOLERANCE = Math.toRadians(10.0);

   private final PelvisHeightPitchActionDefinition definition;
   private final PelvisHeightPitchActionState state;
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final Timer executionTimer = new Timer();
   private final FramePose3D desiredPelvisPose = new FramePose3D();
   private final FramePose3D syncedPelvisPose = new FramePose3D();
   private double startPositionDistanceToGoal;
   private double startOrientationDistanceToGoal;
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public PelvisHeightPitchActionExecutor(long id,
                                          CRDTInfo crdtInfo,
                                          WorkspaceResourceDirectory saveFileDirectory,
                                          ROS2ControllerHelper ros2ControllerHelper,
                                          ReferenceFrameLibrary referenceFrameLibrary,
                                          ROS2SyncedRobotModel syncedRobot)
   {
      super(new PelvisHeightPitchActionState(id, crdtInfo, saveFileDirectory, referenceFrameLibrary));

      state = getState();

      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;

      definition = getDefinition();
   }

   @Override
   public void update()
   {
      super.update();

      state.setCanExecute(state.getPelvisFrame().isChildOfWorld());
   }

   @Override
   public void triggerActionExecution()
   {
      if (state.getPelvisFrame().isChildOfWorld())
      {
         FramePose3D framePose = new FramePose3D(state.getPelvisFrame().getReferenceFrame());
         FramePose3D syncedPose = new FramePose3D(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         framePose.getRotation().setYawPitchRoll(syncedPose.getYaw(), framePose.getPitch(), syncedPose.getRoll());
         framePose.changeFrame(ReferenceFrame.getWorldFrame());
         syncedPose.changeFrame(ReferenceFrame.getWorldFrame());

         PelvisTrajectoryMessage message = new PelvisTrajectoryMessage();
         message.getSe3Trajectory()
                .set(HumanoidMessageTools.createSE3TrajectoryMessage(definition.getTrajectoryDuration(),
                                                                     framePose.getPosition(),
                                                                     framePose.getOrientation(),
                                                                     ReferenceFrame.getWorldFrame()));
         long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
         message.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(frameId);
         message.getSe3Trajectory().getLinearSelectionMatrix().setXSelected(false);
         message.getSe3Trajectory().getLinearSelectionMatrix().setYSelected(false);
         message.getSe3Trajectory().getLinearSelectionMatrix().setZSelected(true);
         ros2ControllerHelper.publishToController(message);
         executionTimer.reset();

         desiredPelvisPose.setFromReferenceFrame(state.getPelvisFrame().getReferenceFrame());
         syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslationX(), syncedPelvisPose.getTranslationY(), desiredPelvisPose.getTranslationZ());
         desiredPelvisPose.getRotation().setYawPitchRoll(syncedPelvisPose.getYaw(), desiredPelvisPose.getPitch(), syncedPelvisPose.getRoll());
         startPositionDistanceToGoal = syncedPelvisPose.getTranslation().differenceNorm(desiredPelvisPose.getTranslation());
         startOrientationDistanceToGoal = syncedPelvisPose.getRotation().distance(desiredPelvisPose.getRotation(), true);
      }
      else
      {
         LogTools.error("Cannot execute. Frame is not a child of World frame.");
      }
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      if (state.getPelvisFrame().isChildOfWorld())
      {
         desiredPelvisPose.setFromReferenceFrame(state.getPelvisFrame().getReferenceFrame());
         syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
         desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslationX(), syncedPelvisPose.getTranslationY(), desiredPelvisPose.getTranslationZ());
         desiredPelvisPose.getRotation().setYawPitchRoll(syncedPelvisPose.getYaw(), desiredPelvisPose.getPitch(), syncedPelvisPose.getRoll());

         state.setIsExecuting(!completionCalculator.isComplete(desiredPelvisPose,
                                                               syncedPelvisPose,
                                                               POSITION_TOLERANCE,
                                                               Double.NaN,
                                                               definition.getTrajectoryDuration(),
                                                               executionTimer,
                                                               BehaviorActionCompletionComponent.TRANSLATION));

         state.setNominalExecutionDuration(definition.getTrajectoryDuration());
         state.setElapsedExecutionTime(executionTimer.getElapsedTime());
         state.setStartOrientationDistanceToGoal(startOrientationDistanceToGoal);
         state.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
         state.setCurrentOrientationDistanceToGoal(completionCalculator.getRotationError());
         state.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
         state.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
         state.setOrientationDistanceToGoalTolerance(ORIENTATION_TOLERANCE);
      }
   }
}
