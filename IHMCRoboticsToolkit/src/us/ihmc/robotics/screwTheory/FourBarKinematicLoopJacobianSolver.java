package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;

public class FourBarKinematicLoopJacobianSolver
{
   private final FourBarCalculatorWithDerivatives fourBarCalculator;
   private DenseMatrix64F jacobian;
   double j11, j21, j31, j12, j22, j32, j13, j23, j33;

   public FourBarKinematicLoopJacobianSolver(FourBarCalculatorWithDerivatives fourBarCalculator)
   {
      this.fourBarCalculator = fourBarCalculator;
      jacobian = new DenseMatrix64F();
   }

   public DenseMatrix64F computeJacobian(PassiveRevoluteJoint fourBarOutputJoint, PassiveRevoluteJoint jointB, PassiveRevoluteJoint jointC, PassiveRevoluteJoint jointD)
   {
      double ab = fourBarCalculator.getAB();
      double bc = fourBarCalculator.getBC();
      double cd = fourBarCalculator.getCD();
      double da = fourBarCalculator.getDA();
      
      double A = fourBarCalculator.getAngleDAB();
      double B = fourBarCalculator.getAngleABC();
      double C = fourBarCalculator.getAngleBCD();
      double D = fourBarCalculator.getAngleCDA();

      if (fourBarOutputJoint == jointD)
      {
         j11 = - ab * Math.sin(A) - bc * Math.sin(A + B) - cd * Math.sin(A + B + C);
         j12 = - bc * Math.sin(A + B) - cd * Math.sin(A + B + C);
         j13 = - cd * Math.sin(A + B + C);
         j21 = 0.0;
         j22 = 0.0;
         j23 = 0.0;
         j31 = ab * Math.cos(A) + bc * Math.cos(A + B) + cd * Math.cos(A + B + C);
         j32 = bc * Math.cos(A + B) + cd * Math.cos(A + B + C);
         j33 = cd * Math.cos(A + B + C);
      }
      else if (fourBarOutputJoint == jointC)
      {
         j11 = 1.0;
         j21 = 1.0;
         j31 = 1.0;
      }
      else
      {
         j11 = 1.0;
         j21 = 1.0;
         j31 = 1.0;
      }

      jacobian = new DenseMatrix64F(3, 3, true, new double[] {j11, j12, j13, j21, j22, j23, j31, j32, j33});
      return jacobian;
   }

   public void solveLinearVelFromAngularVel(DenseMatrix64F jacobian, DenseMatrix64F jointVelocities, DenseMatrix64F resultToPack)
   {
      CommonOps.mult(jacobian, jointVelocities, resultToPack);
   }
}
