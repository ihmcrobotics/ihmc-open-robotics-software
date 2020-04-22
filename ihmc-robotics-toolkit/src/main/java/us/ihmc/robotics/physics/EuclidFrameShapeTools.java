package us.ihmc.robotics.physics;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.Box3D;
import us.ihmc.euclid.shape.primitives.Capsule3D;
import us.ihmc.euclid.shape.primitives.Cylinder3D;
import us.ihmc.euclid.shape.primitives.Ellipsoid3D;
import us.ihmc.euclid.shape.primitives.PointShape3D;
import us.ihmc.euclid.shape.primitives.Ramp3D;
import us.ihmc.euclid.shape.primitives.Sphere3D;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;

public class EuclidFrameShapeTools
{
   public static Box3D changeFrame(Box3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Box3D transformed = new Box3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Capsule3D changeFrame(Capsule3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Capsule3D transformed = new Capsule3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Cylinder3D changeFrame(Cylinder3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Cylinder3D transformed = new Cylinder3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Ellipsoid3D changeFrame(Ellipsoid3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Ellipsoid3D transformed = new Ellipsoid3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static PointShape3D changeFrame(PointShape3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      PointShape3D transformed = new PointShape3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Ramp3D changeFrame(Ramp3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Ramp3D transformed = new Ramp3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   public static Sphere3D changeFrame(Sphere3DReadOnly original, ReferenceFrame from, ReferenceFrame to)
   {
      Sphere3D transformed = new Sphere3D(original);
      from.transformFromThisToDesiredFrame(to, transformed);
      return transformed;
   }

   static interface FrameChanger<A extends Shape3DReadOnly>
   {
      A changeFrame(A original, ReferenceFrame from, ReferenceFrame to);
   }
}
