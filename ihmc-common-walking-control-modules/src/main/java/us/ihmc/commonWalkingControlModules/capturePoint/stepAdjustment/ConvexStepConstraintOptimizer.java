package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.convexOptimization.quadraticProgram.JavaQuadProgSolver;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;

/**
 * The objective of this class is to provide an optimization scheme to constrain a convex polygon within another convex polygon using only linear translations.
 * This is then applied to constrain a step within a step constraint region in the environment.
 *
 * It does so use a garbage free implementation and a quadratic program to satisfy the inequality constraints as best as possible. If there is no possible
 * solution to satisfy one polygon completely within another, it returns a "best-effort" solution that maximizes the amount of area inside the constraining
 * polygon.
 *
 * It is able to achieve this objective efficiently by not requiring a solution be found in a single control tick. The maximum number of iterations in the
 * quadratic program is set fairly low, so as to reduce total time. Then, the solver is "warm started" from the previous active set, allowing the optimization
 * to iteratively find a solution over multiple control ticks.
 */
public class ConvexStepConstraintOptimizer
{
   private static final boolean DEBUG = false;
   private static final boolean warmStart = true;

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
   private final DenseMatrix64F zeros = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F p = new DenseMatrix64F(2, 1);

   private final DenseMatrix64F bFull = new DenseMatrix64F(0, 1);
   private final DenseMatrix64F solution = new DenseMatrix64F(1, 6);

   private final JavaQuadProgSolver solver = new JavaQuadProgSolver();

   private final RigidBodyTransform transformToReturn = new RigidBodyTransform();

   private boolean invariantConstraintRegionValuesComputed = false;
   private boolean invariantOptimizationValuesComputed = false;

   public void reset()
   {
      solver.resetActiveSet();
      invariantConstraintRegionValuesComputed = false;
      invariantOptimizationValuesComputed = false;
   }

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
      if (!invariantConstraintRegionValuesComputed)
         computeInvariantConstraintValues(planeToWiggleInto, parameters, startingVerticesToIgnore, numberOfPoints);

      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int slackVariables = constraintsPerPoint * numberOfPoints;
      int totalVariables = variables + slackVariables;
      int constraints = constraintsPerPoint * numberOfPoints;
      int boundConstraints = parameters.getConstrainMaxAdjustment() ? 4 : 0;

      // The inequality constraints of form
      // Ax <= b
      // are converted to new constraints with a new optimization vector s:
      // Ax - s - b == 0.0
      // s <= 0
      // The equality constraint will be converted to an objective causing the wiggler to do the best it can instead of failing when the wiggle is not possible.

      j.set(bFull);

      for (int i = 0; i < numberOfPoints; i++)
      {
         polygonToWiggle.getVertex(i).get(p);

         // inequality constraint becomes A*x <= b - A*p
         MatrixTools.multAddBlock(-1.0, A, p, j, constraintsPerPoint * i, 0);
      }

      // Check to see if the optimization actually needs to run. Most of the time, probably not, but if so, there's no need to run the optimizer
      zeros.reshape(totalVariables, 1);
      solution.reshape(constraints, 1);
      zeros.zero();
      CommonOps.scale(-1.0, j, solution);
      CommonOps.multAdd(J, zeros, solution); // FIXME there's no way this does anything
      boolean areConstraintsAlreadyValid = true;
      for (int i = 0; i < solution.numRows; i++)
      {
         if (solution.get(i, 0) > 1e-5)
         {
            areConstraintsAlreadyValid = false;
            break;
         }
      }

      if (areConstraintsAlreadyValid)
      {
         transformToReturn.setToZero();
         return transformToReturn;
      }


      // TODO move these invariant setters into the static method later
      Aineq.reshape(constraints + boundConstraints, totalVariables);
      bineq.reshape(constraints + boundConstraints, 1);
      // add limits on allowed translation
      if (parameters.getConstrainMaxAdjustment())
         PolygonWiggler.addTranslationConstraint(Aineq, bineq, constraints, parameters.getMaxX(), -parameters.getMaxX(), parameters.getMaxY(), -parameters.getMaxY());

      MatrixMissingTools.setDiagonalValues(Aineq, 1.0, 0, variables);


      if (!invariantOptimizationValuesComputed)
         computeInvariantOptimizationValues(planeToWiggleInto , parameters, startingVerticesToIgnore, numberOfPoints);

      // Convert the inequality constraint for being inside the polygon to an objective.
      g.reshape(totalVariables, 1);
      CommonOps.multTransA(-polygonWeight, J, j, g);

      try
      {
         solver.setMaxNumberOfIterations(parameters.getMaxIterations());
         solver.setQuadraticCostFunction(G, g, 0.0);
         solver.setLinearInequalityConstraints(Aineq, bineq);
         solver.setUseWarmStart(warmStart);
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
         LogTools.info("Could not find transform!");
         return null;
      }

      // assemble the transform
      transformToReturn.getTranslation().set(solution.get(0), solution.get(1), 0.0);

      return transformToReturn;
   }

   private void computeInvariantConstraintValues(ConvexPolygon2DReadOnly planeToWiggleInto,
                                                 ConstraintOptimizerParameters parameters,
                                                 int[] startingVerticesToIgnore, int numberOfPoints)
   {
      PolygonWiggler.convertToInequalityConstraints(planeToWiggleInto, A, b, parameters.getDesiredDistanceInside(), startingVerticesToIgnore);

      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int slackVariables = constraintsPerPoint * numberOfPoints;
      int totalVariables = variables + slackVariables;
      int constraints = constraintsPerPoint * numberOfPoints;
      int boundConstraints = parameters.getConstrainMaxAdjustment() ? 4 : 0;


      J.reshape(constraints, totalVariables);
      bFull.reshape(constraints, 1);

      MatrixMissingTools.setDiagonalValues(J, -1.0, 0, variables);

      for (int i = 0; i < numberOfPoints; i++)
      {
         CommonOps.insert(A, J, constraintsPerPoint * i, 0);
         CommonOps.insert(b, bFull, constraintsPerPoint * i, 0);
      }

      invariantConstraintRegionValuesComputed = true;
   }

   private void computeInvariantOptimizationValues(ConvexPolygon2DReadOnly planeToWiggleInto,
                                                   ConstraintOptimizerParameters parameters,
                                                   int[] startingVerticesToIgnore, int numberOfPoints)
   {
      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int slackVariables = constraintsPerPoint * numberOfPoints;
      int totalVariables = variables + slackVariables;
      int constraints = constraintsPerPoint * numberOfPoints;
      int boundConstraints = parameters.getConstrainMaxAdjustment() ? 4 : 0;

//      Aineq.reshape(constraints + boundConstraints, totalVariables);
//      bineq.reshape(constraints + boundConstraints, 1);
//       add limits on allowed translation
//      if (parameters.getConstrainMaxAdjustment())
//         PolygonWiggler.addTranslationConstraint(Aineq, bineq, constraints, parameters.getMaxX(), -parameters.getMaxX(), parameters.getMaxY(), -parameters.getMaxY());


      Aineq.reshape(constraints + boundConstraints, totalVariables);
      bineq.reshape(constraints + boundConstraints, 1);
      // add limits on allowed translation
      if (parameters.getConstrainMaxAdjustment())
         PolygonWiggler.addTranslationConstraint(Aineq, bineq, constraints, parameters.getMaxX(), -parameters.getMaxX(), parameters.getMaxY(), -parameters.getMaxY());


      G.reshape(totalVariables, totalVariables);
      CommonOps.multInner(J, G);
      CommonOps.scale(polygonWeight, G);

      // Add regularization
      MatrixTools.addDiagonal(G, regularization);

      // Add movement weight
      for (int i = 0; i < variables; i++)
         G.add(i, i, moveWeight);

      invariantOptimizationValuesComputed = true;
   }
}
