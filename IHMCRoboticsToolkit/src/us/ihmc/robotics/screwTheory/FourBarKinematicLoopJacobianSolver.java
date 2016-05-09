package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoopJacobianSolver
{
   private static final boolean DEBUG = false;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final FourBarCalculatorWithDerivatives fourBarCalculator;
   private final GeometricJacobian geometricJacobian;
   private final InverseDynamicsJoint[] jointsForJacobianCalculation;

   private final DenseMatrix64F geometricJacobianToColumnJacobian, columnJacobian;

   public FourBarKinematicLoopJacobianSolver(FourBarCalculatorWithDerivatives fourBarCalculator, InverseDynamicsJoint[] jointsForJacobianCalculation)
   {
      this.fourBarCalculator = fourBarCalculator;
      this.jointsForJacobianCalculation = jointsForJacobianCalculation;
      
      geometricJacobianToColumnJacobian = computeVectorToGoFromGeometricToColumnJacobian();
      columnJacobian = new DenseMatrix64F(6, 1);
      
      ReferenceFrame jacobianFrame = jointsForJacobianCalculation[jointsForJacobianCalculation.length - 1].getFrameAfterJoint();
      geometricJacobian = new GeometricJacobian(jointsForJacobianCalculation, jacobianFrame);
   }

   public DenseMatrix64F computeJacobian(PassiveRevoluteJoint fourBarOutputJoint)
   {
      // Geometric Jacobian 
      geometricJacobian.compute();
      geometricJacobian.changeFrame(worldFrame);

     
      // Column Jacobian - fourbars are a 1DOF system so the jacobian is a column vector
      CommonOps.mult(geometricJacobian.getJacobianMatrix(), geometricJacobianToColumnJacobian, columnJacobian);

      if (DEBUG)
      {
         System.out.println("Geometric jacobian size: " + geometricJacobian.getJacobianMatrix().getNumRows() + " , " + geometricJacobian.getJacobianMatrix().getNumCols());
         System.out.println("Column jacobian size: " + columnJacobian.getNumRows() + " , " + columnJacobian.getNumCols());
      }

      return columnJacobian;
   }

   public void solveLinearVelFromAngularVel(DenseMatrix64F jacobian, double inputJointVelocity)
   {
      CommonOps.scale(inputJointVelocity, jacobian);
   }
   
   private DenseMatrix64F computeVectorToGoFromGeometricToColumnJacobian()
   {
      // Vector containing angular velocity of passive joints for angular velocity of input joint (master) equal 1
      double dqA_functionOfqA = 1.0;
      fourBarCalculator.updateAnglesAndVelocitiesGivenAngleDAB(fourBarCalculator.getAngleDAB(), 1.0);
      double dqB_functionOfqA = fourBarCalculator.getAngleDtABC();
      double dqC_functionOfqA = fourBarCalculator.getAngleDtBCD();
      
      if (jointsForJacobianCalculation.length == 3) // Output joint is D
      {         
        return new DenseMatrix64F(3, 1, true, new double[] {dqA_functionOfqA, dqB_functionOfqA, dqC_functionOfqA});
      }
      else if (jointsForJacobianCalculation.length == 2) // Output joint is C
      {
         return new DenseMatrix64F(2, 1, true, new double[] {dqA_functionOfqA, dqB_functionOfqA});
      }
      else // Output joint is B
      {
         return new DenseMatrix64F(1, 1, true, new double[] {dqA_functionOfqA});
      }
   }
}
