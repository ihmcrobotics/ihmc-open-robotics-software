package us.ihmc.humanoidBehaviors.behaviors.midLevel;

import us.ihmc.communication.packets.manipulation.HandRotateAboutAxisPacket;
import us.ihmc.humanoidBehaviors.behaviors.primitives.HandPoseBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.utilities.Axis;
import us.ihmc.utilities.humanoidRobot.model.FullRobotModel;
import us.ihmc.utilities.io.printing.PrintTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePose;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.PoseReferenceFrame;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;

public class RotateHandAboutAxisBehavior extends HandPoseBehavior
{
   private final ReferenceFrame world = ReferenceFrame.getWorldFrame();
   private final FullRobotModel fullRobotModel;

   public RotateHandAboutAxisBehavior(String namePrefix, OutgoingCommunicationBridgeInterface outgoingCommunicationBridge, FullRobotModel fullRobotModel,
         DoubleYoVariable yoTime)
   {
      super(namePrefix, outgoingCommunicationBridge, yoTime);
      this.fullRobotModel = fullRobotModel;
   }

   public void setInput(RobotSide robotSide, boolean lockCurrentHandOrientation, Axis axisOrientationInRotationAxisFrame, RigidBodyTransform rotationAxisFrameTransformToWorld,
         double totalRotationInRadians, double rotationRateRadPerSec, boolean stopHandIfCollision)
   {
      FramePose currentHandPoseInWorld = new FramePose();
      currentHandPoseInWorld.setToZero(fullRobotModel.getHandControlFrame(robotSide));
      currentHandPoseInWorld.changeFrame(world);

      setInput(robotSide, currentHandPoseInWorld, lockCurrentHandOrientation, axisOrientationInRotationAxisFrame, rotationAxisFrameTransformToWorld, totalRotationInRadians,
            rotationRateRadPerSec, stopHandIfCollision);
   }

   public void setInput(RobotSide robotSide, FramePose currentHandPoseInWorld, boolean controlHandOrientationAboutAxis, Axis axisOrientationInRotationAxisFrame,
         RigidBodyTransform rotationAxisFrameTransformToWorld, double totalRotationInRadians, double rotationRateRadPerSec, boolean stopHandIfCollision)
   {
      if (totalRotationInRadians == 0.0)
      {
         PrintTools.debug(this, "Desired angle of rotation, " + totalRotationInRadians + ", is not valid (must be non-zero).");
         hasInputBeenSet.set(false);
         return;
      }
      
      PoseReferenceFrame graspedObjectFrame = new PoseReferenceFrame("graspedObjectFrameBeforeRotation", world);
      graspedObjectFrame.setPoseAndUpdate(rotationAxisFrameTransformToWorld);

      double totalTrajectoryTime = Math.abs(totalRotationInRadians / rotationRateRadPerSec);

      FramePoint rotationAxisOrigin = new FramePoint(graspedObjectFrame, 0.0, 0.0, 0.0);
      FrameVector rotationAxis = null;

      if (axisOrientationInRotationAxisFrame == Axis.X)
      {
         rotationAxis = new FrameVector(graspedObjectFrame, 1.0, 0.0, 0.0, "rotationAxis");
      }
      else if (axisOrientationInRotationAxisFrame == Axis.Y)
      {
         rotationAxis = new FrameVector(graspedObjectFrame, 0.0, 1.0, 0.0, "rotationAxis");
      }
      else if (axisOrientationInRotationAxisFrame == Axis.Z)
      {
         rotationAxis = new FrameVector(graspedObjectFrame, 0.0, 0.0, 1.0, "rotationAxis");
      }

      rotationAxisOrigin.changeFrame(world);
      rotationAxis.changeFrame(world);

      HandRotateAboutAxisPacket handRotateAboutAxisPacket = new HandRotateAboutAxisPacket(robotSide, rotationAxisOrigin.getPointCopy(), rotationAxis.getVectorCopy(),
            totalRotationInRadians, totalTrajectoryTime, controlHandOrientationAboutAxis);
      
      this.robotSide = robotSide;
      this.stopHandIfCollision.set(stopHandIfCollision);

      startTime.set(yoTime.getDoubleValue());
      trajectoryTime.set(handRotateAboutAxisPacket.getTrajectoryTime());
      outgoingPacket = handRotateAboutAxisPacket;

      hasInputBeenSet.set(true);
   }
}
