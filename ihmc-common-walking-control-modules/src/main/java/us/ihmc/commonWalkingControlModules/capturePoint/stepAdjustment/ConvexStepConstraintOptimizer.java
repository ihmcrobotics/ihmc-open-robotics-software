package us.ihmc.commonWalkingControlModules.capturePoint.stepAdjustment;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;
import us.ihmc.commonWalkingControlModules.polygonWiggling.PolygonWiggler;
import us.ihmc.commonWalkingControlModules.polygonWiggling.WiggleParameters;
import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.log.LogTools;
import us.ihmc.matrixlib.MatrixTools;
import us.ihmc.robotics.RegionInWorldInterface;
import us.ihmc.robotics.geometry.PlanarRegion;

public class ConvexStepConstraintOptimizer
{
   private static final boolean DEBUG = false;
   private static final boolean coldStart = true;

   /** Weight associated with moving into the polygon. */
   private static final double polygonWeight = 1.0e6;
   /** Regularization weight preferring a zero solution. */
   private static final double regularization = 1.0e-10;
   /** Weight associated with moving the polygon. */
   private static final double moveWeight = 1.0;

   private static int[] emptyArray = new int[0];

   /**
    * If x is contained in the polygon, this can be expressed by Ax <= b
    */
   private final DenseMatrix64F A = new DenseMatrix64F(0, 0);
   private final DenseMatrix64F b = new DenseMatrix64F(0, 0);

   private final DenseMatrix64F V = new DenseMatrix64F(2, 3);
   private final DenseMatrix64F p = new DenseMatrix64F(2, 1);

   DenseMatrix64F bForPoint = new DenseMatrix64F(0, 1);

   private final QuadProgSolver solver = new QuadProgSolver();

   private final Vector2D vectorToPoint = new Vector2D();

   public RigidBodyTransform wigglePolygonIntoConvexHullOfRegion(ConvexPolygon2DReadOnly polygonToWiggleInRegionFrame,
                                                                 PlanarRegion regionToWiggleInto,
                                                                 WiggleParameters parameters)
   {
      return findWiggleTransform(polygonToWiggleInRegionFrame, regionToWiggleInto.getConvexHull(), parameters);
   }

   public RigidBodyTransform findWiggleTransform(ConvexPolygon2DReadOnly polygonToWiggle,
                                                 ConvexPolygon2DReadOnly planeToWiggleInto,
                                                 WiggleParameters parameters)
   {
      return findWiggleTransform(polygonToWiggle, planeToWiggleInto, parameters, emptyArray);
   }

   public RigidBodyTransform findWiggleTransform(ConvexPolygon2DReadOnly polygonToWiggle,
                                                 ConvexPolygon2DReadOnly planeToWiggleInto,
                                                 WiggleParameters parameters,
                                                 int[] startingVerticesToIgnore)
   {
      int numberOfPoints = polygonToWiggle.getNumberOfVertices();

      // This creates inequality constraints for points to lie inside the desired polygon.
      PolygonWiggler.convertToInequalityConstraints(planeToWiggleInto, A, b, parameters.deltaInside, startingVerticesToIgnore);

      int constraintsPerPoint = A.getNumRows();

      int variables = 2;
      int slackVariables = constraintsPerPoint * numberOfPoints;
      int totalVariables = variables + slackVariables;
      int constraints = constraintsPerPoint * numberOfPoints;
      int boundConstraints = 4;
      DenseMatrix64F A_full = new DenseMatrix64F(constraints + boundConstraints, totalVariables);
      DenseMatrix64F b_full = new DenseMatrix64F(constraints + boundConstraints, 1);
      // add limits on allowed translation
      PolygonWiggler.addTranslationConstraint(A_full, b_full, constraints, parameters);


      // The inequality constraints of form
      // Ax <= b
      // are converted to new constraints with a new optimization vector s:
      // Ax - s - b == 0.0
      // s <= 0
      // The equality constraint will be converted to an objective causing the wiggler to do the best it can instead of failing when the wiggle is not possible.
      DenseMatrix64F Aeq = new DenseMatrix64F(constraints, totalVariables);
      DenseMatrix64F beq = new DenseMatrix64F(constraints, 1);
      DenseMatrix64F identity = new DenseMatrix64F(constraints, slackVariables);
      CommonOps.setIdentity(identity);
      CommonOps.insert(identity, A_full, 0, variables);
      CommonOps.scale(-1.0, identity);
      CommonOps.insert(identity, Aeq, 0, variables);

      bForPoint.reshape(constraintsPerPoint, 1);

      for (int i = 0; i < numberOfPoints; i++)
      {
         polygonToWiggle.getVertex(i).get(p);

         // inequality constraint becomes A*x <= b - A*p
         bForPoint.set(b);
         CommonOps.multAdd(-1.0, A, p, bForPoint);

         CommonOps.insert(A, Aeq, constraintsPerPoint * i, 0);
         CommonOps.insert(bForPoint, beq, constraintsPerPoint * i, 0);
      }

      // Convert the inequality constraint for being inside the polygon to an objective.
      // why 3?
      DenseMatrix64F costMatrix = new DenseMatrix64F(totalVariables, totalVariables);
      DenseMatrix64F costVector = new DenseMatrix64F(totalVariables, 1);
      CommonOps.multInner(Aeq, costMatrix);
      CommonOps.multTransA(-1.0, Aeq, beq, costVector);
      CommonOps.scale(polygonWeight, costMatrix);
      CommonOps.scale(polygonWeight, costVector);

      // Add regularization
      MatrixTools.addDiagonal(costMatrix, regularization);

      // Add movement weight
      for (int i = 0; i < variables; i++)
         costMatrix.add(i, i, moveWeight);

      DenseMatrix64F result = new DenseMatrix64F(totalVariables, 1);
      Aeq = new DenseMatrix64F(0, totalVariables);
      beq = new DenseMatrix64F(0, totalVariables);
      try
      {
         int iterations = solver.solve(costMatrix, costVector, Aeq, beq, A_full, b_full, result, coldStart);
         if (DEBUG)
         {
            LogTools.info("Iterations: " + iterations);
            LogTools.info("Result: " + result);
         }
      }
      catch (Exception e)
      {
         e.printStackTrace();
         return null;
      }

      if (Double.isInfinite(solver.getCost()))
      {
         LogTools.info("Could not wiggle!");
         return null;
      }

      // assemble the transform
      RigidBodyTransform translationTransform = new RigidBodyTransform();
      translationTransform.appendTranslation(result.get(0), result.get(1), 0.0);

      return translationTransform;
   }
}
