package us.ihmc.robotics.geometry;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.referenceFrame.FrameGeometryObject;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public class FrameOrientation2d extends FrameGeometryObject<FrameOrientation2d, Orientation2D>
{
   private final Orientation2D orientation;

   public FrameOrientation2d()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   public FrameOrientation2d(FrameOrientation2d frameOrientation)
   {
      this(frameOrientation.getReferenceFrame(), new Orientation2D(frameOrientation.getGeometryObject()));
   }

   public FrameOrientation2d(ReferenceFrame referenceFrame)
   {
      this(referenceFrame, 0.0);
   }

   public FrameOrientation2d(ReferenceFrame referenceFrame, double yaw)
   {
      this(referenceFrame, new Orientation2D(yaw));
   }

   public FrameOrientation2d(ReferenceFrame referenceFrame, Orientation2D orientation)
   {
      super(referenceFrame, orientation);
      this.orientation = getGeometryObject();
   }

   public void interpolate(FrameOrientation2d frameOrientation1, FrameOrientation2d frameOrientation2, double alpha)
   {
      checkReferenceFrameMatch(frameOrientation1);
      frameOrientation1.checkReferenceFrameMatch(frameOrientation2);
      orientation.interpolate(frameOrientation1.orientation, frameOrientation2.orientation, alpha);
   }

   public double getYaw()
   {
      return orientation.getYaw();
   }

   public void setIncludingFrame(ReferenceFrame referenceFrame, double yaw)
   {
      this.referenceFrame = referenceFrame;
      setYaw(yaw);
   }

   public void setYaw(double yaw)
   {
      if (Double.isNaN(yaw))
      {
         throw new RuntimeException("Orientation.setYaw() yaw = " + yaw);
      }

      orientation.setYaw(yaw);
   }

   public double difference(FrameOrientation2d other)
   {
      checkReferenceFrameMatch(other);
      return orientation.difference(other.orientation);
   }
}
