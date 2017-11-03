package us.ihmc.robotics.robotDescription;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.geometry.polytope.DCELPolytope.ExtendedConvexPolytope;

public class ConvexPolytopeDescriptionReadOnly implements ConvexShapeDescription
{
   //TODO: Trying to create a read only version by copying a full version. Should be a more efficient way to do this...
   private final ExtendedConvexPolytope convexPolytope;

   public ConvexPolytopeDescriptionReadOnly(ExtendedConvexPolytope polytope, RigidBodyTransform rigidBodyTransform)
   {
      this.convexPolytope = new ExtendedConvexPolytope(polytope);
      this.convexPolytope.applyTransform(rigidBodyTransform);
   }

   public ExtendedConvexPolytope getConvexPolytope()
   {
      return new ExtendedConvexPolytope(convexPolytope);
   }

}
