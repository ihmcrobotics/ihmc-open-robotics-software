package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.spatial.Twist;

public class CurrentRigidBodyStateProvider
{
   private final MovingReferenceFrame frameOfInterest;
   private final Twist twist = new Twist();

   public CurrentRigidBodyStateProvider(MovingReferenceFrame frameOfInterest)
   {
      this.frameOfInterest = frameOfInterest;
   }
   
   public void getPosition(FramePoint3D positionToPack)
   {
      positionToPack.setToZero(frameOfInterest);
   }
   
   public void getLinearVelocity(FrameVector3D linearVelocityToPack)
   {
      frameOfInterest.getTwistOfFrame(twist);
      linearVelocityToPack.setIncludingFrame(twist.getLinearPart());
   }
   
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setToZero(frameOfInterest);
   }
   
   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      frameOfInterest.getTwistOfFrame(twist);
      angularVelocityToPack.setIncludingFrame(twist.getAngularPart());
   }
}
