package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ActionExecutionStatusMessage;
import controller_msgs.msg.dds.PelvisHeightTrajectoryMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionCalculator;
import us.ihmc.behaviors.sequence.BehaviorActionCompletionComponent;
import us.ihmc.behaviors.sequence.BehaviorActionSequence;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.Timer;

public class PelvisHeightAction extends PelvisHeightActionData implements BehaviorAction
{
   public static final double POSITION_TOLERANCE = 0.15;

   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final ROS2SyncedRobotModel syncedRobot;
   private int actionIndex;
   private final Timer executionTimer = new Timer();
   private boolean isExecuting;
   private final FramePose3D desiredPelvisPose = new FramePose3D();
   private final FramePose3D syncedPelvisPose = new FramePose3D();
   private double startPositionDistanceToGoal;
   private final ActionExecutionStatusMessage executionStatusMessage = new ActionExecutionStatusMessage();
   private final BehaviorActionCompletionCalculator completionCalculator = new BehaviorActionCompletionCalculator();

   public PelvisHeightAction(ROS2ControllerHelper ros2ControllerHelper,
                             ReferenceFrameLibrary referenceFrameLibrary,
                             ROS2SyncedRobotModel syncedRobot)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.referenceFrameLibrary = referenceFrameLibrary;
      this.syncedRobot = syncedRobot;
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex)
   {
      update(referenceFrameLibrary);

      this.actionIndex = actionIndex;
   }

   @Override
   public void triggerActionExecution()
   {
      PelvisHeightTrajectoryMessage message = new PelvisHeightTrajectoryMessage();
      message.getEuclideanTrajectory()
             .set(HumanoidMessageTools.createEuclideanTrajectoryMessage(getTrajectoryDuration(),
                                                                        new Point3D(0.0, 0.0, getHeight()),
                                                                        ReferenceFrame.getWorldFrame()));
      long frameId = MessageTools.toFrameId(ReferenceFrame.getWorldFrame());
      message.getEuclideanTrajectory().getFrameInformation().setDataReferenceFrameId(frameId);
      message.getEuclideanTrajectory().getSelectionMatrix().setXSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setYSelected(false);
      message.getEuclideanTrajectory().getSelectionMatrix().setZSelected(true);


      ros2ControllerHelper.publishToController(message);
      executionTimer.reset();

      desiredPelvisPose.setFromReferenceFrame(getConditionalReferenceFrame().get());
      syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslation());
      desiredPelvisPose.getTranslation().setZ(desiredPelvisPose.getTranslationZ());
      startPositionDistanceToGoal = syncedPelvisPose.getTranslation().differenceNorm(desiredPelvisPose.getTranslation());
   }

   @Override
   public void updateCurrentlyExecuting()
   {
      desiredPelvisPose.setFromReferenceFrame(getConditionalReferenceFrame().get());
      syncedPelvisPose.setFromReferenceFrame(syncedRobot.getFullRobotModel().getPelvis().getBodyFixedFrame());
      desiredPelvisPose.getTranslation().set(syncedPelvisPose.getTranslation());
      desiredPelvisPose.getTranslation().setZ(desiredPelvisPose.getTranslationZ());

      isExecuting = !completionCalculator.isComplete(desiredPelvisPose,
                                                     syncedPelvisPose,
                                                     POSITION_TOLERANCE, Double.NaN,
                                                     getTrajectoryDuration(),
                                                     executionTimer,
                                                     BehaviorActionCompletionComponent.TRANSLATION);

      executionStatusMessage.setActionIndex(actionIndex);
      executionStatusMessage.setNominalExecutionDuration(getTrajectoryDuration());
      executionStatusMessage.setElapsedExecutionTime(executionTimer.getElapsedTime());
      executionStatusMessage.setStartPositionDistanceToGoal(startPositionDistanceToGoal);
      executionStatusMessage.setCurrentPositionDistanceToGoal(completionCalculator.getTranslationError());
      executionStatusMessage.setPositionDistanceToGoalTolerance(POSITION_TOLERANCE);
      ros2ControllerHelper.publish(BehaviorActionSequence.ACTION_EXECUTION_STATUS, this.executionStatusMessage);
   }

   @Override
   public boolean isExecuting()
   {
      return isExecuting;
   }
}
