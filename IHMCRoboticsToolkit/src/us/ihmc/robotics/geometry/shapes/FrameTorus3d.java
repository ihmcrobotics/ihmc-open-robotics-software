package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameTorus3d extends FrameShape3d<FrameTorus3d, Torus3d>
{
   private Torus3d torus3d;

   public FrameTorus3d(FrameTorus3d other)
   {
      this(other.referenceFrame, other.torus3d);
   }

   public FrameTorus3d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Torus3d());
      torus3d = getGeometryObject();
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, Torus3d torus3d)
   {
      super(referenceFrame, new Torus3d(torus3d));
      torus3d = getGeometryObject();
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, double radius, double thickness)
   {
      super(referenceFrame, new Torus3d(radius, thickness));
      torus3d = getGeometryObject();
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, RigidBodyTransform transform, double radius, double thickness)
   {
      super(referenceFrame, new Torus3d(transform, radius, thickness));
      torus3d = getGeometryObject();
   }

   public Torus3d getTorus3d()
   {
      return torus3d;
   }

   public double getRadius()
   {
      return torus3d.getRadius();
   }

   public double getThickness()
   {
      return torus3d.getThickness();
   }

   @Override
   public String toString()
   {
      StringBuilder builder = new StringBuilder();

      builder.append("ReferenceFrame: " + referenceFrame + ")\n");
      builder.append(torus3d.toString());

      return builder.toString();
   }
}
