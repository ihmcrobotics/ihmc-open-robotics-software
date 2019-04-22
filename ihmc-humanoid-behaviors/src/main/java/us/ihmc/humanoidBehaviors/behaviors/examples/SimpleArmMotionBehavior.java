package us.ihmc.humanoidBehaviors.behaviors.examples;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.behaviors.complexBehaviors.ResetRobotBehavior;
import us.ihmc.humanoidBehaviors.behaviors.primitives.AtlasPrimitiveActions;
import us.ihmc.humanoidBehaviors.behaviors.simpleBehaviors.BehaviorAction;
import us.ihmc.humanoidBehaviors.taskExecutor.ArmTrajectoryTask;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.taskExecutor.PipeLine;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.sensorProcessing.frames.CommonReferenceFrameIds;
import us.ihmc.yoVariables.variable.YoDouble;

public class SimpleArmMotionBehavior extends AbstractBehavior
{
   private final PipeLine<AbstractBehavior> pipeLine = new PipeLine<>();

   private final AtlasPrimitiveActions atlasPrimitiveActions;
   private final HumanoidReferenceFrames referenceFrames;

   private final ResetRobotBehavior resetRobotBehavior;

   public SimpleArmMotionBehavior(String robotName, YoDouble yoTime, HumanoidReferenceFrames referenceFrames, Ros2Node ros2Node,
                                  AtlasPrimitiveActions atlasPrimitiveActions)
   {
      super(robotName, ros2Node);
      this.referenceFrames = referenceFrames;
      this.atlasPrimitiveActions = atlasPrimitiveActions;

      resetRobotBehavior = new ResetRobotBehavior(robotName, ros2Node, yoTime);

   }

   @Override
   public void onBehaviorEntered()
   {
      setupPipeline();
   }

   private void setupPipeline()
   {
      pipeLine.clearAll();
      BehaviorAction resetRobot = new BehaviorAction(resetRobotBehavior);

      //    MOVE_HAND_TO_APPROACH_POINT, using joint angles to make sure wrist is in proper turn location

      double[] approachPointLocation = new double[] {0.42441454428847003, -0.5829781169010966, 1.8387098771297432, -2.35619, 0.11468460263836734,
            1.0402909950400858, 0.9434293109027067};

      ArmTrajectoryMessage rightHandValveApproachMessage = HumanoidMessageTools.createArmTrajectoryMessage(RobotSide.RIGHT, 2, approachPointLocation);

      ArmTrajectoryTask moveHandToApproachPoint = new ArmTrajectoryTask(rightHandValveApproachMessage, atlasPrimitiveActions.rightArmTrajectoryBehavior)
      {
         @Override
         protected void setBehaviorInput()
         {
            super.setBehaviorInput();
            publishTextToSpeech("Joint Angles Movement");
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
      publishTextToSpeech(description);

      FramePose3D point = offsetPointFromChestInWorldFrame(x, y, z, yaw, pitch, roll);

      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.RIGHT, 2, point.getPosition(),
                                                                                                     point.getOrientation(),
                                                                                                     CommonReferenceFrameIds.CHEST_FRAME.getHashId());
      handTrajectoryMessage.getSe3Trajectory().getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));

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

   private FramePose3D offsetPointFromChestInWorldFrame(double x, double y, double z, double yaw, double pitch, double roll)
   {
      FramePoint3D point1 = new FramePoint3D(referenceFrames.getChestFrame(), x, y, z);
      point1.changeFrame(ReferenceFrame.getWorldFrame());
      FrameQuaternion orient = new FrameQuaternion(referenceFrames.getChestFrame(), yaw, pitch, roll);
      orient.changeFrame(ReferenceFrame.getWorldFrame());

      FramePose3D pose = new FramePose3D(point1, orient);

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