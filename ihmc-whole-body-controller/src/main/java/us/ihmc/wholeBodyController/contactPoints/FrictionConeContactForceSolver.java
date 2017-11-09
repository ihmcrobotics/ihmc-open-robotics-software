package us.ihmc.wholeBodyController.contactPoints;

import java.util.List;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.convexOptimization.quadraticProgram.QuadProgSolver;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public class FrictionConeContactForceSolver
{
   private static final double regWeight = 1.0e-10;

   private final List<Point2D> contactPoints;
   private final FrictionConeRotationCalculator coneCalculator;

   private final QuadProgSolver solver = new QuadProgSolver();

   private final DenseMatrix64F x = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Q = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F f = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Aeq = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F beq = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F Ain = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F bin = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F objective = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F J = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F W = new DenseMatrix64F(1, 1);

   private final DenseMatrix64F temp1 = new DenseMatrix64F(1, 1);
   private final DenseMatrix64F temp2 = new DenseMatrix64F(1, 1);

   private final Vector3D offset = new Vector3D();
   private final Vector3D unitTorque = new Vector3D();
   private final Vector3D unitForce = new Vector3D();
   private final Vector3D direction = new Vector3D();
   private final Vector3D offsetToCop = new Vector3D();
   private final Vector3D unitTorqueAtCop = new Vector3D();
   private final Vector3D tempForce = new Vector3D();

   public FrictionConeContactForceSolver(List<Point2D> contactPoints, FrictionConeRotationCalculator coneCalculator)
   {
      this.contactPoints = contactPoints;
      this.coneCalculator = coneCalculator;
   }

   /**
    * Solves an optimization problem to achieve a provided x-y force at the location of the cop. To achieve this force
    * friction cone approximations with the provided resolution are placed at each contact point.
    * <p>
    * The constraints on the optimization are:</br>
    *  - the z force is 1.0N</br>
    *  - the cop is at the provided location</br>
    *  - the z torque at the cop is zero</br>
    *  - the achieved force is parallel to the desired force
    * <p>
    * Returns true if the optimization succeeded.
    */
   public boolean solveForFixedCoPAndDirection(int vectorsPerPoint, double friction, Point2D cop, Vector2D force, Vector2D achievedForce)
   {
      int size = vectorsPerPoint * contactPoints.size();

      // initialize x as vector with the grf magnitudes
      x.reshape(size, 1);
      CommonOps.fill(x, 0.0);

      // set up inequality constraints such that Ain * x < bin makes sure all x are positive
      Ain.reshape(size, size);
      CommonOps.setIdentity(Ain);
      CommonOps.scale(-1.0, Ain);
      bin.reshape(size, 1);
      CommonOps.fill(bin, 0.0);

      // regulate the resulting forces: add small diagonal to the Q matrix
      Q.reshape(size, size);
      CommonOps.setIdentity(Q);
      CommonOps.scale(regWeight, Q);

      // set all W equal
      W.reshape(2, 2);
      CommonOps.setIdentity(W);

      // build the task jacobian J and the objective o such that the goal is to minimize (Jx - objective)
      J.reshape(2, size);
      Aeq.reshape(5, size);
      CommonOps.fill(Aeq, 0.0);
      for (int contactIdx = 0; contactIdx < contactPoints.size(); contactIdx++)
      {
         // vector from sole to contact
         Point2D contactPoint = contactPoints.get(contactIdx);
         offset.set(contactPoint.getX(), contactPoint.getY(), 0.0);

         offsetToCop.setX(contactPoint.getX() - cop.getX());
         offsetToCop.setY(contactPoint.getY() - cop.getY());
         offsetToCop.setZ(0.0);

         for (int vectorIdx = 0; vectorIdx < vectorsPerPoint; vectorIdx++)
         {
            coneCalculator.packVector(contactPoints, contactIdx, vectorsPerPoint, vectorIdx, friction, direction);

            unitForce.set(direction);
            unitTorque.cross(offset, unitForce);
            unitTorqueAtCop.cross(offsetToCop, unitForce);

            J.set(0, contactIdx * vectorsPerPoint + vectorIdx, unitForce.getX());
            J.set(1, contactIdx * vectorsPerPoint + vectorIdx, unitForce.getY());

            Aeq.set(0, contactIdx * vectorsPerPoint + vectorIdx, unitTorque.getX());
            Aeq.set(1, contactIdx * vectorsPerPoint + vectorIdx, unitTorque.getY());
            Aeq.set(2, contactIdx * vectorsPerPoint + vectorIdx, unitForce.getZ());
            Aeq.set(3, contactIdx * vectorsPerPoint + vectorIdx, unitTorqueAtCop.getZ());

            Aeq.add(4, contactIdx * vectorsPerPoint + vectorIdx, force.getY() * unitForce.getX());
            Aeq.add(4, contactIdx * vectorsPerPoint + vectorIdx, -force.getX() * unitForce.getY());
         }
      }

      objective.reshape(2, 1);
      objective.set(0, force.getX());
      objective.set(1, force.getY());

      double fz = 1.0;
      double tauX = cop.getY() * fz;
      double tauY = -cop.getX() * fz;
      double tauZAtCop = 0.0;

      beq.reshape(5, 1);
      beq.set(0, tauX);
      beq.set(1, tauY);
      beq.set(2, fz);
      beq.set(3, tauZAtCop);

      beq.set(4, 0.0);

      // assemble the matrices for the optimization such that we can minimize 0.5 x'Qx + x'f
      // the task is minimizing
      // 0.5 * (Jx - objective)' W (Jx - objective) = 0.5 * x'J'WJx - x'J'W objective
      // temp1 = J'W
      temp1.reshape(size, 2);
      CommonOps.multTransA(J, W, temp1);
      // temp2 = temp1 * J = J'WJ
      temp2.reshape(size, size);
      CommonOps.mult(temp1, J, temp2);

      // add J'WJ to Q
      CommonOps.add(Q, temp2, Q);

      // set f to -J'W
      f.reshape(size, 1);
      CommonOps.fill(f, 0.0);
      CommonOps.mult(temp1, objective, f);
      CommonOps.scale(-1.0, f);

      // fix equality constraints
      for (int i = 0; i < beq.getNumRows(); i++)
      {
         boolean rowIzero = true;
         for (int j = 0; j < size; j++)
         {
            if (Math.abs(Aeq.get(i, j)) > 1.0e-10)
            {
               rowIzero = false;
               break;
            }
         }
         if (rowIzero)
         {
            MatrixTools.removeRow(Aeq, i);
            MatrixTools.removeRow(beq, i);
         }
      }

      try
      {
         solver.solve(Q, f, Aeq, beq, Ain, bin, x, false);
      }
      catch (Exception e)
      {
         return false;
      }

      if (Double.isInfinite(solver.getCost()))
      {
         return false;
      }

      achievedForce.setToZero();

      for (int contactIdx = 0; contactIdx < contactPoints.size(); contactIdx++)
      {
         Point2D contactPoint = contactPoints.get(contactIdx);
         offset.set(contactPoint.getX(), contactPoint.getY(), 0.0);

         for (int vectorIdx = 0; vectorIdx < vectorsPerPoint; vectorIdx++)
         {
            coneCalculator.packVector(contactPoints, contactIdx, vectorsPerPoint, vectorIdx, friction, direction);

            double rho = x.get(contactIdx * vectorsPerPoint + vectorIdx);
            tempForce.set(direction);
            tempForce.scale(rho);
            achievedForce.add(tempForce.getX(), tempForce.getY());
         }
      }

      return true;
   }
}
