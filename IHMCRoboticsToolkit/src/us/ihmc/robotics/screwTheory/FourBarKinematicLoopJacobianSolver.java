package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoopJacobianSolver
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FourBarCalculatorWithDerivatives fourBarCalculator;
   private final GeometricJacobian jacobian;
   private final InverseDynamicsJoint[] joints;
   
   private final DenseMatrix64F geometricJacobianToColumnJacobian;

   public FourBarKinematicLoopJacobianSolver(FourBarCalculatorWithDerivatives fourBarCalculator, InverseDynamicsJoint[] fourbarJoints)
   {
      this.fourBarCalculator = fourBarCalculator;
      this.joints = fourbarJoints;
      jacobian = new GeometricJacobian(joints, worldFrame);
      geometricJacobianToColumnJacobian = new DenseMatrix64F(3, 1);
   }

   public DenseMatrix64F computeJacobian(PassiveRevoluteJoint fourBarOutputJoint)
   {
      double ab = fourBarCalculator.getAB();
      double bc = fourBarCalculator.getBC();
      double cd = fourBarCalculator.getCD();
      double da = fourBarCalculator.getDA();

      double A = fourBarCalculator.getAngleDAB();
      double B = fourBarCalculator.getAngleABC();
      double C = fourBarCalculator.getAngleBCD();
      double D = fourBarCalculator.getAngleCDA();

      jacobian.compute();
      
      //      if (fourBarOutputJoint == joints[3])
      //      {
      //      }
      //      else if (fourBarOutputJoint == joints[2])
      //      {
      //         jacobian.compute();
      //      }
      //      else
      //      {
      //         jacobian.compute();
      //      }

      System.out.println(jacobian.getJacobianMatrix().getNumRows() + " , " + jacobian.getJacobianMatrix().getNumCols() );
      return jacobian.getJacobianMatrix();
   }

   public void solveLinearVelFromAngularVel(DenseMatrix64F jacobian, DenseMatrix64F jointVelocities, DenseMatrix64F resultToPack)
   {
      CommonOps.mult(jacobian, jointVelocities, resultToPack);
   }
}
