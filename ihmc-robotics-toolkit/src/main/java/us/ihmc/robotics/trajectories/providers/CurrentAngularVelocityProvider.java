package us.ihmc.robotics.trajectories.providers;

import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.mecano.spatial.Twist;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;

public class CurrentAngularVelocityProvider implements VectorProvider
{
   private final MovingReferenceFrame referenceFrame;
   private final Twist twist = new Twist();

   public CurrentAngularVelocityProvider(MovingReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void get(FrameVector3D frameVectorToPack)
   {
      referenceFrame.getTwistOfFrame(twist);
      frameVectorToPack.setIncludingFrame(twist.getAngularPart());
   }
}
