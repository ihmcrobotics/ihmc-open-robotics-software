package us.ihmc.robotics.robotDescription;

import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class ConvexPolytopeDescriptionReadOnly implements ConvexShapeDescription
{
   //TODO: Trying to create a read only version by copying a full version. Should be a more efficient way to do this...
   private final ConvexPolytope convexPolytope;

   public ConvexPolytopeDescriptionReadOnly(ConvexPolytope polytope, RigidBodyTransform rigidBodyTransform)
   {
      this.convexPolytope = new ConvexPolytope(polytope);
      this.convexPolytope.applyTransform(rigidBodyTransform);
   }

   public ConvexPolytope getConvexPolytope()
   {
      return new ConvexPolytope(convexPolytope);
   }

}
