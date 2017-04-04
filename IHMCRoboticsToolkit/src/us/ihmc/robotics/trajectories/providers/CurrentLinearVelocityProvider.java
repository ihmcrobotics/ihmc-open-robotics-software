package us.ihmc.robotics.trajectories.providers;

import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;

public class CurrentLinearVelocityProvider implements VectorProvider
{
   private final ReferenceFrame referenceFrame;
   private final RigidBody rigidBody;
   private final TwistCalculator twistCalculator;
   private final Twist twist = new Twist();

   public CurrentLinearVelocityProvider(ReferenceFrame referenceFrame, RigidBody rigidBody, TwistCalculator twistCalculator)
   {
      this.referenceFrame = referenceFrame;
      this.rigidBody = rigidBody;
      this.twistCalculator = twistCalculator;
   }

   public void get(FrameVector frameVectorToPack)
   {
      twistCalculator.getTwistOfBody(rigidBody, twist);
      twist.changeFrame(referenceFrame);
      twist.getLinearPart(frameVectorToPack);
   }
}
