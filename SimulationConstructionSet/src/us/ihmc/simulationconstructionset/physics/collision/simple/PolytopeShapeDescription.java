package us.ihmc.simulationconstructionset.physics.collision.simple;

import us.ihmc.geometry.polytope.ConvexPolytope;
import us.ihmc.simulationconstructionset.physics.CollisionShapeDescription;

public class PolytopeShapeDescription implements CollisionShapeDescription
{
   private final ConvexPolytope polytope;

   public PolytopeShapeDescription(ConvexPolytope polytope)
   {
      this.polytope = polytope;
   }

   public ConvexPolytope getPolytope()
   {
      return polytope;
   }

}
