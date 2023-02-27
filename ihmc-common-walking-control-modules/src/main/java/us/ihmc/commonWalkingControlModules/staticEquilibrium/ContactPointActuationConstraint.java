package us.ihmc.commonWalkingControlModules.staticEquilibrium;

import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;

class ContactPointActuationConstraint
{
   private double maxNormalForce;
   private final ConvexPolytope3D polytopeConstraint = new ConvexPolytope3D();

   public ContactPointActuationConstraint()
   {
      clear();
   }

   public void clear()
   {
      maxNormalForce = Double.NaN;
      polytopeConstraint.setToNaN();
   }

   public void setToMaxNormalForce(double rhoMax)
   {
      clear();
      this.maxNormalForce = rhoMax;
   }

   public void setToPolytopeConstraint(ConvexPolytope3DReadOnly polytopeConstraint)
   {
      clear();
      this.polytopeConstraint.set(polytopeConstraint);
   }

   public double getMaxNormalForce()
   {
      return maxNormalForce;
   }

   public ConvexPolytope3D getPolytopeConstraint()
   {
      return polytopeConstraint;
   }

   public boolean isMaxNormalForceConstraint()
   {
      return !Double.isNaN(maxNormalForce);
   }

   public int getNumberOfConstraints()
   {
      if (isMaxNormalForceConstraint())
      {
         return 1;
      }
      else
      {
         return polytopeConstraint.getNumberOfFaces();
      }
   }
}
