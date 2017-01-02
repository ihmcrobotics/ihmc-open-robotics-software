package us.ihmc.robotics.geometry.shapes;

import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FrameEllipsoid3d extends FrameShape3d<FrameEllipsoid3d, Ellipsoid3d>
{
   private final Ellipsoid3d ellipsoid;

   public FrameEllipsoid3d(FrameEllipsoid3d other)
   {
      this(other.referenceFrame, other.ellipsoid);
   }

   public FrameEllipsoid3d(ReferenceFrame referenceFrame, Ellipsoid3d ellipsoid)
   {
      super(referenceFrame, new Ellipsoid3d(ellipsoid));
      this.ellipsoid = getGeometryObject();
   }

   public FrameEllipsoid3d(ReferenceFrame referenceFrame, double xRadius, double yRadius, double zRadius)
   {
      super(referenceFrame, new Ellipsoid3d(xRadius, yRadius, zRadius));
      ellipsoid = getGeometryObject();
   }
   
   public FrameEllipsoid3d(ReferenceFrame referenceFrame, double xRadius, double yRadius, double zRadius, RigidBodyTransform transform)
   {
      super(referenceFrame, new Ellipsoid3d(xRadius, yRadius, zRadius, transform));
      ellipsoid = getGeometryObject();
   }

   public Ellipsoid3d getEllipsoid3d()
   {
      return ellipsoid;
   }
}
