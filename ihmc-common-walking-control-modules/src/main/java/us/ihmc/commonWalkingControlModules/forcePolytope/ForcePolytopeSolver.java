package us.ihmc.commonWalkingControlModules.forcePolytope;

import org.ejml.data.DMatrixRMaj;
import us.ihmc.euclid.shape.convexPolytope.ConvexPolytope3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

import java.util.List;

public interface ForcePolytopeSolver
{
   /**
    * Given a point-jacobian on the robot, this solves for the force polytope.
    * The lower and upper limits are enforced, and should account for gravity compensation if needed.
    *
    */
   void solve(DMatrixRMaj jacobian, DMatrixRMaj tauLowerLimit, DMatrixRMaj tauUpperLimit, ConvexPolytope3D polytopeToPack);

   /**
    * All vertices added to the polytope, for debugging
    */
   default List<Point3D> getVertices()
   {
      return null;
   }

   /**
    * Interior point solve parameters
    */
   Point3D[] directionsToOptimize = SpiralBasedAlgorithm.generatePointsOnSphere(1.0, 50);

   /**
    * For implementations using SVD, this is the cutoff for singular vectors
    */
   double singularValueThreshold = 0.025;

}
