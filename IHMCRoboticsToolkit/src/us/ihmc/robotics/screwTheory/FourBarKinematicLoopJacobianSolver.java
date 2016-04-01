package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;

public class FourBarKinematicLoopJacobianSolver
{
   private final FourBarCalculatorWithDerivatives fourBarCalculator;
   private DenseMatrix64F jacobian;

   public FourBarKinematicLoopJacobianSolver(FourBarCalculatorWithDerivatives fourBarCalculator)
   {
      this.fourBarCalculator = fourBarCalculator;
      jacobian = new DenseMatrix64F();
   }

   public DenseMatrix64F computeJacobian(double fourBarInputJoint_q)
   {
      double ab = fourBarCalculator.getAB();
      double bc = fourBarCalculator.getBC();
      double cd = fourBarCalculator.getCD();
      double da = fourBarCalculator.getDA();

      double j11 = ab * Math.cos(fourBarInputJoint_q);
      double j21 = bc * Math.sin(fourBarInputJoint_q);
      double j31 = 0.5;

      jacobian = new DenseMatrix64F(3, 1, true, new double[]{j11,j21, j31});
      return jacobian;
   }

   public void solveLinearVelFromAngularVel(DenseMatrix64F jacobian, DenseMatrix64F fourBarInputJoint_qd, DenseMatrix64F resultToPack)
   {
      CommonOps.mult(jacobian, fourBarInputJoint_qd, resultToPack); 
      System.out.println(resultToPack);
   }
}
