package us.ihmc.robotics.math.trajectories;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoMutableFrameQuaternion;
import us.ihmc.yoVariables.registry.YoRegistry;


public class ConstantPoseTrajectoryGenerator implements PoseTrajectoryGenerator
{
   private final FramePoint3DBasics position;
   private final FrameQuaternionBasics orientation;

   public ConstantPoseTrajectoryGenerator(FramePoint3DBasics position, FrameQuaternionBasics orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      this.position = position;
      this.orientation = orientation;
   }

   public ConstantPoseTrajectoryGenerator(String namePrefix, ReferenceFrame referenceFrame, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(namePrefix + getClass().getSimpleName());
      position = new YoMutableFramePoint3D(namePrefix + "ConstantPosition", "", registry, referenceFrame);
      orientation = new YoMutableFrameQuaternion(namePrefix + "ConstantOrientation", "", registry, referenceFrame);
   }

   public void changeFrame(ReferenceFrame referenceFrame)
   {
      position.changeFrame(referenceFrame);
      orientation.changeFrame(referenceFrame);
   }

   public void switchTrajectoryFrame(ReferenceFrame referenceFrame)
   {
      position.setToZero(referenceFrame);
      orientation.setToZero(referenceFrame);
   }

   public void setConstantPose(FramePose3D constantPose)
   {
      position.checkReferenceFrameMatch(constantPose);
      position.set(constantPose.getX(), constantPose.getY(), constantPose.getZ());
      orientation.setYawPitchRoll(constantPose.getYaw(), constantPose.getPitch(), constantPose.getRoll());
   }

   public void setConstantPose(FramePoint3D constantPosition, FrameQuaternion constantOrientation)
   {
      this.position.set(position);
      this.orientation.set(orientation);
   }

   @Override
   public void initialize()
   {
      // Do nothing
   }

   @Override
   public void compute(double time)
   {
      // Do nothing
   }

   @Override
   public boolean isDone()
   {
      return true;
   }

   @Override
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setIncludingFrame(position);
   }

   @Override
   public void getVelocity(FrameVector3D velocityToPack)
   {
      velocityToPack.setToZero(position.getReferenceFrame());
   }

   @Override
   public void getAcceleration(FrameVector3D accelerationToPack)
   {
      accelerationToPack.setToZero(position.getReferenceFrame());
   }

   @Override
   public void getLinearData(FramePoint3D positionToPack, FrameVector3D velocityToPack, FrameVector3D accelerationToPack)
   {
      getPosition(positionToPack);
      getVelocity(velocityToPack);
      getAcceleration(accelerationToPack);
   }

   @Override
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setIncludingFrame(orientation);
   }

   @Override
   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      angularVelocityToPack.setToZero(orientation.getReferenceFrame());
   }

   @Override
   public void getAngularAcceleration(FrameVector3D angularAccelerationToPack)
   {
      angularAccelerationToPack.setToZero(orientation.getReferenceFrame());
   }

   @Override
   public void getAngularData(FrameQuaternion orientationToPack, FrameVector3D angularVelocityToPack, FrameVector3D angularAccelerationToPack)
   {
      getOrientation(orientationToPack);
      getAngularVelocity(angularVelocityToPack);
      getAngularAcceleration(angularAccelerationToPack);
   }

   @Override
   public void getPose(FramePose3D framePoseToPack)
   {
      framePoseToPack.changeFrame(position.getReferenceFrame());
      framePoseToPack.getPosition().set(position);
      framePoseToPack.getOrientation().set(orientation);
   }

   @Override
   public String toString()
   {
      String ret = "";
      ret += "Current position: " + position.toString();
      ret += "\nCurrent orientation: " + orientation.toString();
      return ret;
   }

   @Override
   public void showVisualization()
   {
   }

   @Override
   public void hideVisualization()
   {
   }
}