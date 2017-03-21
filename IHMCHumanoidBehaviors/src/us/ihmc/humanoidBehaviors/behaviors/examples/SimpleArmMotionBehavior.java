package us.ihmc.humanoidBehaviors.behaviors.examples;

import us.ihmc.communication.packets.TextToSpeechPacket;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.TurnValveBehaviorStateMachine.TurnValveBehaviorState;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridge;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.taskExecutor.PipeLine;

public class SimpleArmMotionBehavior extends AbstractBehavior
{
   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final HumanoidReferenceFrames referenceFrames;

   private final ResetRobotBehavior resetRobotBehavior;

   public SimpleArmMotionBehavior(DoubleYoVariable yoTime, HumanoidReferenceFrames referenceFrames, CommunicationBridge outgoingCommunicationBridge,
         AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(outgoingCommunicationBridge);
      this.referenceFrames = referenceFrames;
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      resetRobotBehavior = new ResetRobotBehavior(communicationBridge, yoTime);

   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
   }

   private void setupPipeline()
   {
      pipeLine.clearAll();
      BehaviorAction<TurnValveBehaviorState> resetRobot = new BehaviorAction<TurnValveBehaviorState>(TurnValveBehaviorState.RESET_ROBOT, resetRobotBehavior);

      //    MOVE_HAND_TO_APPROACH_POINT, using joint angles to make sure wrist is in proper turn location

      double[] approachPointLocation = new double[] {0.42441454428847003, -0.5829781169010966, 1.8387098771297432, -2.35619, 0.11468460263836734,
            1.0402909950400858, 0.9434293109027067};

      ArmTrajectoryMessage rightHandValveApproachMessage = new ArmTrajectoryMessage(RobotSide.RIGHT, 2, approachPointLocation);

      ArmTrajectoryTask moveHandToApproachPoint = new ArmTrajectoryTask(rightHandValveApproachMessage, atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            super.setBehaviorInput();
            TextToSpeechPacket p1 = new TextToSpeechPacket("Joint Angles Movement");
            sendPacket(p1);
         }
      };

      BehaviorAction moveHandCloseToValve = new BehaviorAction(atlasPrimitiveActions.rightHandTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            moveHand(0.5, 0.5, 0.5, 1.5708, 1.5708, -3.14159, "Hand Trajectory Movement");
         }
      };

      //    CLOSE_HAND,
      // pipeLine.submitSingleTaskStage(closeHand);

      //    MOVE_HAND_TO_APPROACH_POINT,
      pipeLine.submitSingleTaskStage(moveHandToApproachPoint);
      //    MOVE_HAND_ABOVE_VALVE,
      pipeLine.submitSingleTaskStage(moveHandCloseToValve);
      pipeLine.submitSingleTaskStage(resetRobot);

   }

   private void moveHand(final double x, final double y, final double z, final double yaw, final double pitch, final double roll, final String description)
   {
      TextToSpeechPacket p1 = new TextToSpeechPacket(description);
      sendPacket(p1);

      FramePose point = offsetPointFromChestInWorldFrame(x, y, z, yaw, pitch, roll);

      HandTrajectoryMessage handTrajectoryMessage = new HandTrajectoryMessage(RobotSide.RIGHT, 2, point.getFramePointCopy().getPoint(),
            point.getFrameOrientationCopy().getQuaternion(), ReferenceFrame.getWorldFrame(), referenceFrames.getChestFrame());

      atlasPrimitiveActions.rightHandTrajectoryBehavior.setInput(handTrajectoryMessage);
   }

   @Override
   public void onBehaviorExited()
   {
   }

   @Override
   public void doControl()
   {
      pipeLine.doControl();
   }

   @Override
   public boolean isDone()
   {
      return pipeLine.isDone();
   }

   private FramePose offsetPointFromChestInWorldFrame(double x, double y, double z, double yaw, double pitch, double roll)
   {
      FramePoint point1 = new FramePoint(referenceFrames.getChestFrame(), x, y, z);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameOrientation orient = new FrameOrientation(referenceFrames.getChestFrame(), yaw, pitch, roll);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose pose = new FramePose(point1, orient);

      return pose;
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }

}