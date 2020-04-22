package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.FrameBox3D;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameCylinder3D;
import us.ihmc.euclid.referenceFrame.FrameEllipsoid3D;
import us.ihmc.euclid.referenceFrame.FramePointShape3D;
import us.ihmc.euclid.referenceFrame.FrameRamp3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.polytope.FrameConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;

public class PhysicsEngineTools
{
   public static FrameShape3DBasics toFrameShape3DBasics(ReferenceFrame referenceFrame, Shape3DReadOnly shape)
   {
      if (shape instanceof Box3DReadOnly)
         return new FrameBox3D(referenceFrame, (Box3DReadOnly) shape);
      if (shape instanceof Capsule3DReadOnly)
         return new FrameCapsule3D(referenceFrame, (Capsule3DReadOnly) shape);
      if (shape instanceof ConvexPolytope3DReadOnly)
         return new FrameConvexPolytope3D(referenceFrame, (ConvexPolytope3DReadOnly) shape);
      if (shape instanceof Cylinder3DReadOnly)
         return new FrameCylinder3D(referenceFrame, (Cylinder3DReadOnly) shape);
      if (shape instanceof Ellipsoid3DReadOnly)
         return new FrameEllipsoid3D(referenceFrame, (Ellipsoid3DReadOnly) shape);
      if (shape instanceof PointShape3DReadOnly)
         return new FramePointShape3D(referenceFrame, (PointShape3DReadOnly) shape);
      if (shape instanceof Ramp3DReadOnly)
         return new FrameRamp3D(referenceFrame, (Ramp3DReadOnly) shape);
      if (shape instanceof Sphere3DReadOnly)
         return new FrameSphere3D(referenceFrame, (Sphere3DReadOnly) shape);

      throw new UnsupportedOperationException("Unsupported shape for conversion: " + shape.getClass().getSimpleName());
   }
}
