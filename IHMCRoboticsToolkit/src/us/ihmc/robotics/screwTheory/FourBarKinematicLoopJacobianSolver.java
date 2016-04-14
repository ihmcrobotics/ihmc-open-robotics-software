package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;

public class FourBarKinematicLoopJacobianSolver
{
   private static final boolean DEBUG = false;

   private final FourBarCalculatorWithDerivatives fourBarCalculator;
   private final GeometricJacobian jacobian;
   private final InverseDynamicsJoint[] joints;

   private final DenseMatrix64F geometricJacobianToColumnJacobian, columnJacobian;

   public FourBarKinematicLoopJacobianSolver(FourBarCalculatorWithDerivatives fourBarCalculator, InverseDynamicsJoint[] fourbarJoints)
   {
      this.fourBarCalculator = fourBarCalculator;
      this.joints = fourbarJoints;
      jacobian = new GeometricJacobian(fourbarJoints, joints[2].getFrameAfterJoint());
      geometricJacobianToColumnJacobian = new DenseMatrix64F(3, 1);
      columnJacobian = new DenseMatrix64F(6, 1);
   }

   public DenseMatrix64F computeJacobian(PassiveRevoluteJoint fourBarOutputJoint)
   {
      // Geometric Jacobian 
      jacobian.compute();
//      jacobian.changeFrame(worldFrame);

      // Vector containing angular velocity of passive joints for angular velocity of input joint (master) equal 1
      double dqA_functionOfqA = 1.0;
      fourBarCalculator.updateAnglesAndVelocitiesGivenAngleDAB(fourBarCalculator.getAngleDAB(), 1.0);
      double dqB_functionOfqA = fourBarCalculator.getAngleDtABC();
      double dqC_functionOfqA = fourBarCalculator.getAngleDtBCD();
      geometricJacobianToColumnJacobian.setData(new double[] {dqA_functionOfqA, dqB_functionOfqA, dqC_functionOfqA});

      // Column Jacobian - fourbars are a 1DOF system so the jacobian is a column vector
      CommonOps.mult(jacobian.getJacobianMatrix(), geometricJacobianToColumnJacobian, columnJacobian);

      if (DEBUG)
      {
         System.out.println("Geometric jacobian size: " + jacobian.getJacobianMatrix().getNumRows() + " , " + jacobian.getJacobianMatrix().getNumCols());
         System.out.println("Column jacobian size: " + columnJacobian.getNumRows() + " , " + columnJacobian.getNumCols());
      }

      return columnJacobian;
   }

   public void solveLinearVelFromAngularVel(DenseMatrix64F jacobian, double inputJointVelocity)
   {
      CommonOps.scale(inputJointVelocity, jacobian);
   }
}
