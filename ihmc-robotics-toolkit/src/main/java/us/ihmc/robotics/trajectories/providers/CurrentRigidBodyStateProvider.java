package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;

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
      twist.getLinearPart(linearVelocityToPack);
   }
   
   public void getOrientation(FrameQuaternion orientationToPack)
   {
      orientationToPack.setToZero(frameOfInterest);
   }
   
   public void getAngularVelocity(FrameVector3D angularVelocityToPack)
   {
      frameOfInterest.getTwistOfFrame(twist);
      twist.getAngularPart(angularVelocityToPack);
   }
}
