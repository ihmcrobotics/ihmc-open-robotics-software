package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.screwTheory.MovingReferenceFrame;
import us.ihmc.robotics.screwTheory.Twist;

public class CurrentLinearVelocityProvider implements VectorProvider
{
   private final MovingReferenceFrame referenceFrame;
   private final Twist twist = new Twist();

   public CurrentLinearVelocityProvider(MovingReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   public void get(FrameVector frameVectorToPack)
   {
      referenceFrame.getTwistOfFrame(twist);
      twist.getLinearPart(frameVectorToPack);
   }
}
