package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class ConvexStepConstraintOptimizer
{
   private static final boolean DEBUG = false;
   private static final boolean coldStart = true;

   /**
    * Weight associated with moving into the polygon.
    */
   private static final double polygonWeight = 1.0e6;
   /**
    * Regularization weight preferring a zero solution.
    */
   private static final double regularization = 1.0e-10;
   /**
    * Weight associated with moving the polygon.
    */
   private static final double moveWeight = 1.0;

   private static int[] emptyArray = new int[0];

   private final DenseMatrix64F G = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F g = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F J = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F j = new DenseMatrix64F(0, 0);

   /**
    * If x is contained in the polygon, this can be expressed by Ax <= b
    */
   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F b = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F Aineq = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F bineq = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F p = new DenseMatrix64F(2, 1);

   private final DenseMatrix64F bForPoint = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solution = new DenseMatrix64F(1, 6);

   private final JavaQuadProgSolver solver = new JavaQuadProgSolver();

   private final DenseMatrix64F identity = new DenseMatrix64F(0, 0);
   private final RigidBodyTransform transformToReturn = new RigidBodyTransform();

   public RigidBodyTransformReadOnly findConstraintTransform(ConvexPolygon2DReadOnly polygonToWiggle,
                                                             ConvexPolygon2DReadOnly planeToWiggleInto,
                                                             ConstraintOptimizerParameters parameters)
   {
      return findConstraintTransform(polygonToWiggle, planeToWiggleInto, parameters, emptyArray);
   }

   public RigidBodyTransformReadOnly findConstraintTransform(ConvexPolygon2DReadOnly polygonToWiggle,
                                                             ConvexPolygon2DReadOnly planeToWiggleInto,
                                                             ConstraintOptimizerParameters parameters,
                                                             int[] startingVerticesToIgnore)
   {
      int numberOfPoints = polygonToWiggle.getNumberOfVertices();

      // This creates inequality constraints for points to lie inside the desired polygon.
      PolygonWiggler.convertToInequalityConstraints(planeToWiggleInto, A, b, parameters.getDesiredDistanceInside(), startingVerticesToIgnore);

      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int slackVariables = constraintsPerPoint * numberOfPoints;
      int totalVariables = variables + slackVariables;
      int constraints = constraintsPerPoint * numberOfPoints;
      int boundConstraints = parameters.getConstrainMaxAdjustment() ? 4 : 0;

      Aineq.reshape(constraints + boundConstraints, totalVariables);
      bineq.reshape(constraints + boundConstraints, 1);
      // add limits on allowed translation // FIXME get rid of this?
      if (parameters.getConstrainMaxAdjustment())
         PolygonWiggler.addTranslationConstraint(Aineq, bineq, constraints, parameters.getMaxX(), -parameters.getMaxX(), parameters.getMaxY(), -parameters.getMaxY());

      // The inequality constraints of form
      // Ax <= b
      // are converted to new constraints with a new optimization vector s:
      // Ax - s - b == 0.0
      // s <= 0
      // The equality constraint will be converted to an objective causing the wiggler to do the best it can instead of failing when the wiggle is not possible.
      J.reshape(constraints, totalVariables);
      j.reshape(constraints, 1);
      identity.reshape(constraints, slackVariables);
      CommonOps.setIdentity(identity);
      CommonOps.insert(identity, Aineq, 0, variables);
      CommonOps.scale(-1.0, identity);
      CommonOps.insert(identity, J, 0, variables);

      bForPoint.reshape(constraintsPerPoint, 1);

      for (int i = 0; i < numberOfPoints; i++)
      {
         polygonToWiggle.getVertex(i).get(p);

         // inequality constraint becomes A*x <= b - A*p
         bForPoint.set(b);
         CommonOps.multAdd(-1.0, A, p, bForPoint);

         CommonOps.insert(A, J, constraintsPerPoint * i, 0);
         CommonOps.insert(bForPoint, j, constraintsPerPoint * i, 0);
      }

      // Convert the inequality constraint for being inside the polygon to an objective.
      G.reshape(totalVariables, totalVariables);
      g.reshape(totalVariables, 1);
      CommonOps.multInner(J, G);
      CommonOps.multTransA(-polygonWeight, J, j, g);
      CommonOps.scale(polygonWeight, G);

      // Add regularization
      MatrixTools.addDiagonal(G, regularization);

      // Add movement weight
      for (int i = 0; i < variables; i++)
         G.add(i, i, moveWeight);

      solution.reshape(totalVariables, 1);
      try
      {
         solver.setQuadraticCostFunction(G, g, 0.0);
         solver.setLinearInequalityConstraints(Aineq, bineq);
         solver.setUseWarmStart(!coldStart);
         int iterations = solver.solve(solution);

         if (DEBUG)
         {
            LogTools.info("Iterations: " + iterations);
            LogTools.info("Result: " + solution);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }

      if (MatrixTools.containsNaN(solution))
      {
         LogTools.info("Could not wiggle!");
         return null;
      }

      // assemble the transform
      transformToReturn.getTranslation().set(solution.get(0), solution.get(1), 0.0);

      return transformToReturn;
   }
}
