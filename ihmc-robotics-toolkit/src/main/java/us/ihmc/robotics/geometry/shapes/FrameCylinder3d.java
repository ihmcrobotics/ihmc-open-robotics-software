package us.ihmc.robotics.geometry.shapes;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class FrameCylinder3d extends FrameShape3d<FrameCylinder3d, Cylinder3D>
{
   private Cylinder3D cylinder3d;

   public FrameCylinder3d(FrameCylinder3d other)
   {
      this(other.referenceFrame, other.cylinder3d);
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Cylinder3D());
      cylinder3d = getGeometryObject();
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame, Cylinder3D cylinder3d)
   {
      super(referenceFrame, new Cylinder3D(cylinder3d));
      this.cylinder3d = getGeometryObject();
   }

   public FrameCylinder3d(ReferenceFrame referenceFrame, double length, double radius)
   {
      super(referenceFrame, new Cylinder3D(length, radius));
      cylinder3d = getGeometryObject();
   }
   
   public FrameCylinder3d(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      super(referenceFrame, new Cylinder3D(position, axis, length, radius));
      cylinder3d = getGeometryObject();
   }

   public Cylinder3D getCylinder3d()
   {
      return cylinder3d;
   }

   public double getRadius()
   {
      return cylinder3d.getRadius();
   }

   public double getLength()
   {
      return cylinder3d.getLength();
   }
}
