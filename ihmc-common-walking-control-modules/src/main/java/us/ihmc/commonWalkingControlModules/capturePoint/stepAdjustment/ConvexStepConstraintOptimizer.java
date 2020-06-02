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

   DenseMatrix64F AforPoint = new DenseMatrix64F(0, 3);
   DenseMatrix64F bForPoint = new DenseMatrix64F(0, 1);

   private final QuadProgSolver solver = new QuadProgSolver();

   private final Vector2D vectorToPoint = new Vector2D();

   public RigidBodyTransform wigglePolygonIntoConvexHullOfRegion(ConvexPolygon2DReadOnly polygonToWiggleInRegionFrame, PlanarRegion regionToWiggleInto, WiggleParameters parameters)
   {
      return findWiggleTransform(polygonToWiggleInRegionFrame, regionToWiggleInto.getConvexHull(), parameters);
   }

   public RigidBodyTransform findWiggleTransform(ConvexPolygon2DReadOnly polygonToWiggle, ConvexPolygon2DReadOnly planeToWiggleInto, WiggleParameters parameters)
   {
      return findWiggleTransform(polygonToWiggle, planeToWiggleInto, parameters, emptyArray);
   }

   public RigidBodyTransform findWiggleTransform(ConvexPolygon2DReadOnly polygonToWiggle, ConvexPolygon2DReadOnly planeToWiggleInto, WiggleParameters parameters,
                                                        int[] startingVerticesToIgnore)
   {
      int numberOfPoints = polygonToWiggle.getNumberOfVertices();
      Point2DReadOnly pointToRotateAbout = polygonToWiggle.getCentroid();

      // This creates inequality constraints for points to lie inside the desired polygon.
      PolygonWiggler.convertToInequalityConstraints(planeToWiggleInto, A, b, parameters.deltaInside, startingVerticesToIgnore);

      int constraintsPerPoint = A.getNumRows();

      int boundConstraints = 6;
      DenseMatrix64F A_full = new DenseMatrix64F(constraintsPerPoint * numberOfPoints + boundConstraints, 3 + constraintsPerPoint * numberOfPoints);
      DenseMatrix64F b_full = new DenseMatrix64F(constraintsPerPoint * numberOfPoints + boundConstraints, 1);
      // add limits on allowed rotation and translation
      PolygonWiggler.addRotationAndTranslationConstraint(A_full, b_full, constraintsPerPoint * numberOfPoints, parameters);

      // The inequality constraints of form
      // Ax <= b
      // are converted to new constraints with a new optimization vector s:
      // Ax - s - b == 0.0
      // s <= 0
      // The equality constraint will be converted to an objective causing the wiggler to do the best it can instead of failing when the wiggle is not possible.
      DenseMatrix64F Aeq = new DenseMatrix64F(constraintsPerPoint * numberOfPoints, 3 + constraintsPerPoint * numberOfPoints);
      DenseMatrix64F beq = new DenseMatrix64F(constraintsPerPoint * numberOfPoints, 1);
      DenseMatrix64F indentity = new DenseMatrix64F(constraintsPerPoint * numberOfPoints, constraintsPerPoint * numberOfPoints);
      CommonOps.setIdentity(indentity);
      CommonOps.insert(indentity, A_full, 0, 3);
      CommonOps.scale(-1.0, indentity);
      CommonOps.insert(indentity, Aeq, 0, 3);

      AforPoint.reshape(constraintsPerPoint, 3);
      bForPoint.reshape(constraintsPerPoint, 1);

      for (int i = 0; i < numberOfPoints; i++)
      {
         polygonToWiggle.getVertex(i).get(p);

         // inequality constraint becomes A*V * x <= b - A*p
         Point2DReadOnly point = polygonToWiggle.getVertex(i);
         vectorToPoint.sub(point, pointToRotateAbout);
         V.set(0, 0, 1.0);
         V.set(0, 2, -point.getY());
         V.set(1, 1, 1.0);
         V.set(1, 2, point.getX());

         CommonOps.mult(A, V, AforPoint);
         CommonOps.mult(A, p, bForPoint);
         CommonOps.changeSign(bForPoint);
         CommonOps.add(b, bForPoint, bForPoint);

         CommonOps.insert(AforPoint, Aeq, constraintsPerPoint * i, 0);
         CommonOps.insert(bForPoint, beq, constraintsPerPoint * i, 0);
      }

      // Convert the inequality constraint for being inside the polygon to an objective.
      DenseMatrix64F costMatrix = new DenseMatrix64F(3 + constraintsPerPoint * numberOfPoints, 3 + constraintsPerPoint * numberOfPoints);
      CommonOps.multInner(Aeq, costMatrix);
      DenseMatrix64F costVector = new DenseMatrix64F(3 + constraintsPerPoint * numberOfPoints, 1);
      CommonOps.multTransA(Aeq, beq, costVector);
      CommonOps.changeSign(costVector);
      CommonOps.scale(polygonWeight, costMatrix);
      CommonOps.scale(polygonWeight, costVector);

      // Add regularization
      indentity.reshape(3 + constraintsPerPoint * numberOfPoints, 3 + constraintsPerPoint * numberOfPoints);
      CommonOps.setIdentity(indentity);
      CommonOps.scale(regularization, indentity);
      CommonOps.add(costMatrix, indentity, costMatrix);

      // Add movement weight
      costMatrix.add(0, 0, moveWeight);
      costMatrix.add(1, 1, moveWeight);
      costMatrix.add(2, 2, moveWeight * parameters.rotationWeight);

      DenseMatrix64F result = new DenseMatrix64F(3 + constraintsPerPoint * numberOfPoints, 1);
      Aeq = new DenseMatrix64F(0, 3 + constraintsPerPoint * numberOfPoints);
      beq = new DenseMatrix64F(0, 3 + constraintsPerPoint * numberOfPoints);
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
      double theta = result.get(2);
      Vector3D translation = new Vector3D(result.get(0), result.get(1), 0.0);
      Vector3D offset = new Vector3D(pointToRotateAbout.getX(), pointToRotateAbout.getY(), 0.0);

      RigidBodyTransform toOriginTransform = new RigidBodyTransform();
      toOriginTransform.setTranslationAndIdentityRotation(offset);

      RigidBodyTransform rotationTransform = new RigidBodyTransform();
      rotationTransform.appendYawRotation(theta);

      RigidBodyTransform fullTransform = new RigidBodyTransform(toOriginTransform);
      fullTransform.multiply(rotationTransform);
      toOriginTransform.invert();
      fullTransform.multiply(toOriginTransform);

      RotationMatrix rotationMatrix = new RotationMatrix();
      rotationMatrix.set(rotationTransform.getRotation());
      rotationMatrix.transpose();
      rotationMatrix.transform(translation);
      RigidBodyTransform translationTransform = new RigidBodyTransform();
      translationTransform.setTranslationAndIdentityRotation(translation);
      fullTransform.multiply(translationTransform);

      return fullTransform;
   }
}
