package us.ihmc.avatar.stepAdjustment;

import controller_msgs.msg.dds.FootstepStatusMessage;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class SimpleStep
{
   private RobotSide swingSide;
   private double swingDuration;
   private double startTime;
   private FramePose3D stepPose = new FramePose3D();

   public SimpleStep()
   {
   }

   public SimpleStep(FootstepStatusMessage statusMessage, double startTime)
   {
      setSwingSide(RobotSide.fromByte(statusMessage.getRobotSide()));
      setSwingDuration(statusMessage.getSwingDuration());
      setStartTime(startTime);
      setStepPose(statusMessage.getDesiredFootPositionInWorld(), statusMessage.getDesiredFootOrientationInWorld());
   }

   public void setSwingSide(RobotSide swingSide)
   {
      this.swingSide = swingSide;
   }

   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   public void setStartTime(double startTime)
   {
      this.startTime = startTime;
   }

   public void setStepPose(Pose3DReadOnly stepPose)
   {
      setStepPose(stepPose.getPosition(), stepPose.getOrientation());
   }

   public void setStepPose(Point3DReadOnly stepPosition, Orientation3DReadOnly stepOrientation)
   {
      setStepPosition(stepPosition);
      setStepOrientation(stepOrientation);
   }

   public void setStepPosition(Point3DReadOnly stepPosition)
   {
      stepPose.getPosition().set(stepPosition);
   }

   public void setStepOrientation(Orientation3DReadOnly stepOrientation)
   {
      stepPose.getOrientation().set(stepOrientation);
   }


   public double getSwingDuration()
   {
      return swingDuration;
   }

   public double getStartTime()
   {
      return startTime;
   }

   public RobotSide getSwingSide()
   {
      return swingSide;
   }

   public FramePose3DReadOnly getStepPose()
   {
      return stepPose;
   }
}
