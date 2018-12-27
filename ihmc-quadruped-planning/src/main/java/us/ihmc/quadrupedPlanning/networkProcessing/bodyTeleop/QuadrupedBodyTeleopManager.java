package us.ihmc.quadrupedPlanning.networkProcessing.bodyTeleop;

import com.google.common.util.concurrent.AtomicDouble;
import controller_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.referenceFrames.QuadrupedReferenceFrames;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicLong;

public class QuadrupedBodyTeleopManager
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoDouble timestamp = new YoDouble("timestamp", registry);

   private final AtomicBoolean paused = new AtomicBoolean(false);
   private final AtomicDouble desiredPositionX = new AtomicDouble();
   private final AtomicDouble desiredPositionY = new AtomicDouble();
   private final AtomicDouble desiredOrientationYaw = new AtomicDouble();
   private final AtomicDouble desiredOrientationPitch = new AtomicDouble();
   private final AtomicDouble desiredOrientationRoll = new AtomicDouble();
   private final AtomicDouble desiredOrientationTime = new AtomicDouble();

   private final AtomicLong timestampNanos = new AtomicLong();

   private final QuadrupedReferenceFrames referenceFrames;

   private QuadrupedBodyOrientationMessage bodyOrientationMessage = null;
   private QuadrupedBodyTrajectoryMessage bodyTrajectoryMessage = null;

   public QuadrupedBodyTeleopManager(QuadrupedReferenceFrames referenceFrames, YoVariableRegistry parentRegistry)
   {
      this.referenceFrames = referenceFrames;

      parentRegistry.addChild(registry);
   }

   public void processTimestamp(long timestampInNanos)
   {
      this.timestampNanos.set(timestampInNanos);
   }

   public void setPaused(boolean pause)
   {
      paused.set(pause);
   }

   public void setDesiredBodyOrientation(double yaw, double pitch, double roll, double time)
   {
      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
   }

   public void setDesiredBodyPose(double x, double y, double yaw, double pitch, double roll, double time)
   {
      desiredPositionX.set(x);
      desiredPositionY.set(y);

      desiredOrientationYaw.set(yaw);
      desiredOrientationPitch.set(pitch);
      desiredOrientationRoll.set(roll);
      desiredOrientationTime.set(time);
   }

   public void update()
   {
      timestamp.set(Conversions.nanosecondsToSeconds(timestampNanos.get()));
      referenceFrames.updateFrames();

      bodyOrientationMessage = null;
      bodyTrajectoryMessage = null;

      if (paused.get())
         return;

      populateDesiredBodyTrajectory();
   }

   public boolean isPaused()
   {
      return paused.get();
   }

   public QuadrupedBodyOrientationMessage getBodyOrientationMessage()
   {
      return bodyOrientationMessage;
   }

   public QuadrupedBodyTrajectoryMessage getBodyTrajectoryMessage()
   {
      return bodyTrajectoryMessage;
   }

   private void populateDesiredBodyOrientation()
   {
      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

      if (!Double.isNaN(desiredYaw))
      {
         bodyOrientationMessage = new QuadrupedBodyOrientationMessage();
         bodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().clear();
         bodyOrientationMessage.setIsAnOffsetOrientation(true);
         SO3TrajectoryPointMessage trajectoryPointMessage = bodyOrientationMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().add();
         trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         trajectoryPointMessage.setTime(desiredTime);
      }
   }

   private void populateDesiredBodyTrajectory()
   {
      double desiredX = desiredPositionX.getAndSet(Double.NaN);
      double desiredY = desiredPositionY.getAndSet(Double.NaN);

      double desiredYaw = desiredOrientationYaw.getAndSet(Double.NaN);
      double desiredPitch = desiredOrientationPitch.getAndSet(Double.NaN);
      double desiredRoll = desiredOrientationRoll.getAndSet(Double.NaN);
      double desiredTime = desiredOrientationTime.getAndSet(Double.NaN);

      bodyTrajectoryMessage = new QuadrupedBodyTrajectoryMessage();
      SE3TrajectoryMessage se3Trajectory = bodyTrajectoryMessage.getSe3Trajectory();
      se3Trajectory.getTaskspaceTrajectoryPoints().clear();
      se3Trajectory.getFrameInformation().setDataReferenceFrameId(MessageTools.toFrameId(ReferenceFrame.getWorldFrame()));
      se3Trajectory.getAngularSelectionMatrix().setXSelected(false);
      se3Trajectory.getAngularSelectionMatrix().setYSelected(false);
      se3Trajectory.getAngularSelectionMatrix().setZSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setXSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setYSelected(false);
      se3Trajectory.getLinearSelectionMatrix().setZSelected(false);
      SE3TrajectoryPointMessage trajectoryPointMessage = se3Trajectory.getTaskspaceTrajectoryPoints().add();

      if (!Double.isNaN(desiredYaw))
      {
         trajectoryPointMessage.getOrientation().setYawPitchRoll(desiredYaw, desiredPitch, desiredRoll);
         se3Trajectory.getAngularSelectionMatrix().setXSelected(true);
         se3Trajectory.getAngularSelectionMatrix().setYSelected(true);
         se3Trajectory.getAngularSelectionMatrix().setZSelected(true);
      }
      if (!Double.isNaN(desiredX) && !Double.isNaN(desiredY))
      {
         trajectoryPointMessage.getPosition().setX(desiredX);
         trajectoryPointMessage.getPosition().setY(desiredY);
         se3Trajectory.getLinearSelectionMatrix().setXSelected(true);
         se3Trajectory.getLinearSelectionMatrix().setYSelected(true);
      }
      trajectoryPointMessage.setTime(desiredTime);
   }

}