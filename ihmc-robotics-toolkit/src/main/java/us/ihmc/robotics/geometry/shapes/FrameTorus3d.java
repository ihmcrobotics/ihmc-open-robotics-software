package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Torus3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameTorus3d extends FrameShape3d<FrameTorus3d, Torus3D>
{
   private Torus3D torus3d;

   public FrameTorus3d(FrameTorus3d other)
   {
      this(other.referenceFrame, other.torus3d);
   }

   public FrameTorus3d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Torus3D());
      torus3d = getGeometryObject();
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, Torus3D torus3d)
   {
      super(referenceFrame, new Torus3D(torus3d));
      torus3d = getGeometryObject();
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, double radius, double thickness)
   {
      super(referenceFrame, new Torus3D(radius, thickness));
      torus3d = getGeometryObject();
   }

   public FrameTorus3d(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double radius, double thickness)
   {
      super(referenceFrame, new Torus3D(position, axis, radius, thickness));
      torus3d = getGeometryObject();
   }

   public Torus3D getTorus3d()
   {
      return torus3d;
   }

   public double getRadius()
   {
      return torus3d.getRadius();
   }

   public double getThickness()
   {
      return torus3d.getTubeRadius();
   }
}
