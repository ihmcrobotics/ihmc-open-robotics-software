package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;

public class CurrentAngularVelocityProvider implements VectorProvider
{
   private final MovingReferenceFrame referenceFrame;
   private final Twist twist = new Twist();

   public CurrentAngularVelocityProvider(MovingReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void get(FrameVector frameVectorToPack)
   {
      referenceFrame.getTwistOfFrame(twist);
      twist.getAngularPart(frameVectorToPack);
   }
}
