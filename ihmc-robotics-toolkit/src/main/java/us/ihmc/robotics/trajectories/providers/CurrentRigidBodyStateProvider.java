package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
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
   
   public void getPosition(FixedFramePoint3DBasics positionToPack)
   {
      positionToPack.setFromReferenceFrame(frameOfInterest);
   }
   
   public void getLinearVelocity(FixedFrameVector3DBasics linearVelocityToPack)
   {
      frameOfInterest.getTwistOfFrame(twist);
      linearVelocityToPack.setMatchingFrame(twist.getLinearPart());
   }
   
   public void getOrientation(FixedFrameQuaternionBasics orientationToPack)
   {
      orientationToPack.setFromReferenceFrame(frameOfInterest);
   }
   
   public void getAngularVelocity(FixedFrameVector3DBasics angularVelocityToPack)
   {
      frameOfInterest.getTwistOfFrame(twist);
      angularVelocityToPack.setMatchingFrame(twist.getAngularPart());
   }
}
