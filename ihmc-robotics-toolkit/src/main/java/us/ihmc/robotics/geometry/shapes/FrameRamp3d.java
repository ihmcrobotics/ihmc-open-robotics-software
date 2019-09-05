package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.transform.RigidBodyTransform;

public class FrameRamp3d extends FrameShape3d<FrameRamp3d, Ramp3D>
{
   private Ramp3D ramp3d;

   public FrameRamp3d(FrameRamp3d other)
   {
      this(other.referenceFrame, other.ramp3d);
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, Ramp3D ramp3d)
   {
      super(referenceFrame, new Ramp3D(ramp3d));
      this.ramp3d = getGeometryObject();
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, double width, double length, double height)
   {
      super(referenceFrame, new Ramp3D(length, width, height));
      ramp3d = getGeometryObject();
   }

   public FrameRamp3d(ReferenceFrame referenceFrame, RigidBodyTransform configuration, double width, double length, double height)
   {
      super(referenceFrame, new Ramp3D(configuration, length, width, height));
      ramp3d = getGeometryObject();
   }

   public Ramp3D getRamp3d()
   {
      return ramp3d;
   }

   public void setTransform(RigidBodyTransform transform3D)
   {
      ramp3d.getPose().set(transform3D);
   }
}
