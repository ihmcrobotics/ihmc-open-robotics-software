package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class CurrentRigidBodyStateProvider
{
   private final ReferenceFrame frameOfInterest;
   private final RigidBody rigidBody;
   private final TwistCalculator twistCalculator;
   private final Twist twist = new Twist();

   public CurrentRigidBodyStateProvider(ReferenceFrame frameOfInterest, RigidBody rigidBody, TwistCalculator twistCalculator)
   {
      this.frameOfInterest = frameOfInterest;
      this.rigidBody = rigidBody;
      this.twistCalculator = twistCalculator;
   }
   
   public void getPosition(FramePoint positionToPack)
   {
      positionToPack.setToZero(frameOfInterest);
   }
   
   public void getLinearVelocity(FrameVector linearVelocityToPack)
   {
      twistCalculator.getTwistOfBody(rigidBody, twist);
      twist.changeFrame(frameOfInterest);
      twist.getLinearPart(linearVelocityToPack);
   }
   
   public void getOrientation(FrameOrientation orientationToPack)
   {
      orientationToPack.setToZero(frameOfInterest);
   }
   
   public void getAngularVelocity(FrameVector angularVelocityToPack)
   {
      twistCalculator.getTwistOfBody(rigidBody, twist);
      twist.changeFrame(frameOfInterest);
      twist.getAngularPart(angularVelocityToPack);
   }
}
