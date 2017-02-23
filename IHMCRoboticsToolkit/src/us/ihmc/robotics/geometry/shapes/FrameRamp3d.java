package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameRamp3d extends FrameShape3d<FrameRamp3d, Ramp3d>
{
   private Ramp3d ramp3d;

   public FrameRamp3d(FrameRamp3d other)
   {
      this(other.referenceFrame, other.ramp3d);
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, Ramp3d ramp3d)
   {
      super(referenceFrame, new Ramp3d(ramp3d));
      this.ramp3d = getGeometryObject();
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, double width, double length, double height)
   {
      super(referenceFrame, new Ramp3d(width, length, height));
      ramp3d = getGeometryObject();
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, RigidBodyTransform configuration, double width, double length, double height)
   {
      super(referenceFrame, new Ramp3d(configuration, width, length, height));
      ramp3d = getGeometryObject();
   }

   public Ramp3d getRamp3d()
   {
      return ramp3d;
   }

   public void setTransform(RigidBodyTransform transform3D)
   {
      ramp3d.setPose(transform3D);
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceFrame: " + referenceFrame + ")\n");
      builder.append(ramp3d.toString());

      return builder.toString();
   }
}
