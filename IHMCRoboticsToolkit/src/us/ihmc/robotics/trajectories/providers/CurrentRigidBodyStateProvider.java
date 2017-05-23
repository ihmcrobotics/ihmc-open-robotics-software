package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
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
   
   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.setToZero(frameOfInterest);
   }
   
   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      frameOfInterest.getTwistOfFrame(twist);
      twist.getLinearPart(linearVelocityToPack);
   }
   
   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(frameOfInterest);
   }
   
   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      frameOfInterest.getTwistOfFrame(twist);
      twist.getAngularPart(angularVelocityToPack);
   }
}
