package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.ejml.data.DMatrixRMaj;
import org.ejml.dense.row.CommonOps_DDRM;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commons.MathTools;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.MatrixMissingTools;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

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
   private boolean warmStart = true;

   /**
    * Weight associated with moving into the polygon. This is typically an inequality constraint, but is implemented as a cost
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

   private final DMatrixRMaj G = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj g = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj J = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj j = new DMatrixRMaj(0, 0);

   /**
    * If x is contained in the polygon, this can be expressed by Ax <= b
    */
   private final DMatrixRMaj A = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj b = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj Aineq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj bineq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj Aeq = new DMatrixRMaj(0, 0);
   private final DMatrixRMaj beq = new DMatrixRMaj(0, 0);

   private final DMatrixRMaj p = new DMatrixRMaj(2, 1);

   private final DMatrixRMaj bFull = new DMatrixRMaj(0, 1);
   private final DMatrixRMaj solution = new DMatrixRMaj(1, 6);

   private final QuadProgSolver solver = new QuadProgSolver();

   private final RigidBodyTransform transformToReturn = new RigidBodyTransform();

   private boolean invariantConstraintRegionValuesComputed = false;
   private boolean invariantOptimizationValuesComputed = false;

   private final YoInteger iterations;

   public ConvexStepConstraintOptimizer(YoRegistry registry)
   {
      iterations = new YoInteger("StepConstrationOptimizerIterations", registry);
   }

   public void reset()
   {
      iterations.set(0);
      invariantConstraintRegionValuesComputed = false;
      invariantOptimizationValuesComputed = false;
   }

   public void setWarmStart(boolean warmStart)
   {
      this.warmStart = warmStart;
   }

   public RigidBodyTransformReadOnly findConstraintTransform(ConvexPolygon2DReadOnly polygonToWiggle,
                                                             ConvexPolygon2DReadOnly planeToWiggleInto,
                                                             ConstraintOptimizerParametersReadOnly parameters)
   {
      return findConstraintTransform(polygonToWiggle, planeToWiggleInto, parameters, emptyArray);
   }

   public RigidBodyTransformReadOnly findConstraintTransform(ConvexPolygon2DReadOnly polygonToWiggle,
                                                             ConvexPolygon2DReadOnly planeToWiggleInto,
                                                             ConstraintOptimizerParametersReadOnly parameters,
                                                             int[] startingVerticesToIgnore)
   {
      iterations.set(-1);
      if (!parameters.shouldPerformOptimization())
      {
         transformToReturn.setToZero();
         return transformToReturn;
      }

      int numberOfPoints = polygonToWiggle.getNumberOfVertices();

      boolean parametersChanged = parameters.pollParametersChanged();

      // This creates inequality constraints for points to lie inside the desired polygon.
      if (!invariantConstraintRegionValuesComputed || parametersChanged)
      {
         computeInvariantConstraintValues(planeToWiggleInto, parameters, startingVerticesToIgnore, numberOfPoints);
      }

      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int slackVariables = constraintsPerPoint * numberOfPoints;
      int totalVariables = variables + slackVariables;

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

      // Check to see if the optimization actually needs to run. Most of the time, probably not, and if so, there's no need to run the optimizer
      solution.reshape(numberOfPoints, 1);
      CommonOps_DDRM.scale(-1.0, j, solution);
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

      if (!invariantOptimizationValuesComputed)
      {
         computeInvariantOptimizationValues(numberOfPoints);
         computeInequalityConstraints(parameters, totalVariables, numberOfPoints);
      }
      else if (parametersChanged)
      {
         computeInequalityConstraints(parameters, totalVariables, numberOfPoints);
      }

      // Convert the inequality constraint for being inside the polygon to an objective.
      g.reshape(totalVariables, 1);
      CommonOps_DDRM.multTransA(-polygonWeight, J, j, g);

      try
      {
         solution.reshape(totalVariables, 1);
         iterations.set(solver.solve(G, g, Aeq, beq, Aineq, bineq, solution, !warmStart));

         if (DEBUG)
         {
            LogTools.info("Iterations: " + iterations);
            LogTools.info("Result: " + solution);
         }
      }
      catch (Throwable e)
      {
         e.printStackTrace();
         return null;
      }

      if (MatrixTools.containsNaN(solution))
      {
         LogTools.info("Could not find transform!");
         return null;
      }

      // enforce the constraints manually
      if (parameters.getConstrainMaxAdjustment())
      {
         solution.set(0, 0, MathTools.clamp(solution.get(0), parameters.getMaxX()));
         solution.set(1, 0, MathTools.clamp(solution.get(1), parameters.getMaxY()));
      }
      // assemble the transform
      transformToReturn.getTranslation().set(solution.get(0), solution.get(1), 0.0);

      return transformToReturn;
   }

   public int getIterationsInOptimization()
   {
      return iterations.getIntegerValue();
   }

   private void computeInvariantConstraintValues(ConvexPolygon2DReadOnly planeToWiggleInto,
                                                 ConstraintOptimizerParametersReadOnly parameters,
                                                 int[] startingVerticesToIgnore, int numberOfPoints)
   {
      PolygonWiggler.convertToInequalityConstraints(planeToWiggleInto, A, b, parameters.getDesiredDistanceInside(), startingVerticesToIgnore);

      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int slackVariables = constraintsPerPoint * numberOfPoints;
      int totalVariables = variables + slackVariables;
      int constraints = constraintsPerPoint * numberOfPoints;

      J.reshape(constraints, totalVariables);
      bFull.reshape(constraints, 1);

      MatrixMissingTools.setDiagonalValues(J, -1.0, 0, variables);

      for (int i = 0; i < numberOfPoints; i++)
      {
         CommonOps_DDRM.insert(A, J, constraintsPerPoint * i, 0);
         CommonOps_DDRM.insert(b, bFull, constraintsPerPoint * i, 0);
      }

      invariantConstraintRegionValuesComputed = true;
   }

   private void computeInvariantOptimizationValues(int numberOfPoints)
   {
      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int slackVariables = constraintsPerPoint * numberOfPoints;
      int totalVariables = variables + slackVariables;

      G.reshape(totalVariables, totalVariables);
      CommonOps_DDRM.multInner(J, G);
      CommonOps_DDRM.scale(polygonWeight, G);

      // Add regularization
      MatrixTools.addDiagonal(G, regularization);

      // Add movement weight
      for (int i = 0; i < variables; i++)
         G.add(i, i, moveWeight);

      invariantOptimizationValuesComputed = true;
   }

   private void computeInequalityConstraints(ConstraintOptimizerParametersReadOnly parameters, int totalVariables, int numberOfPoints)
   {
      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int constraints = constraintsPerPoint * numberOfPoints;
      int boundConstraints = parameters.getConstrainMaxAdjustment() ? 4 : 0;

      Aeq.reshape(0, totalVariables);
      Aineq.reshape(constraints + boundConstraints, totalVariables);
      bineq.reshape(constraints + boundConstraints, 1);
      // add limits on allowed translation
      if (parameters.getConstrainMaxAdjustment())
         PolygonWiggler.addTranslationConstraint(Aineq, bineq, constraints, parameters.getMaxX(), -parameters.getMaxX(), parameters.getMaxY(), -parameters.getMaxY());

      MatrixMissingTools.setDiagonalValues(Aineq, 1.0, 0, variables);
   }
}
