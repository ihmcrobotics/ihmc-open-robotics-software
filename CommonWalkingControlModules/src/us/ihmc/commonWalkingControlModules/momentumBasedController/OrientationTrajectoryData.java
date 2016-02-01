package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class OrientationTrajectoryData
{
   private final FrameOrientation orientation = new FrameOrientation(ReferenceFrame.getWorldFrame());
   private final FrameVector angularVelocity = new FrameVector(ReferenceFrame.getWorldFrame());
   private final FrameVector angularAcceleration = new FrameVector(ReferenceFrame.getWorldFrame());

   public void set(FrameOrientation orientation, FrameVector angularVelocity, FrameVector angularAcceleration)
   {
      this.orientation.setIncludingFrame(orientation);
      this.angularVelocity.setIncludingFrame(angularVelocity);
      this.angularAcceleration.setIncludingFrame(angularAcceleration);
   }

   public FrameOrientation getOrientation()
   {
      return orientation;
   }

   public FrameVector getAngularVelocity()
   {
      return angularVelocity;
   }

   public FrameVector getAngularAcceleration()
   {
      return angularAcceleration;
   }
}
