package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.kinematics.fourbar.FourBarCalculatorWithDerivatives;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class FourBarKinematicLoopJacobianSolver
{
   private static final boolean DEBUG = false;

   private final FourBarCalculatorWithDerivatives fourBarCalculator;
  
   private final GeometricJacobian geometricJacobian;
   private final InverseDynamicsJoint[] jointsForJacobianCalculation;
   private final ReferenceFrame geometricJacobianFrame;
   private final DenseMatrix64F geometricJacobianToColumnJacobian;

   public FourBarKinematicLoopJacobianSolver(FourBarCalculatorWithDerivatives fourBarCalculator, InverseDynamicsJoint[] jointsForJacobianCalculation, PassiveRevoluteJoint outputJoint)
   {
      this.fourBarCalculator = new FourBarCalculatorWithDerivatives(fourBarCalculator);
      this.jointsForJacobianCalculation = jointsForJacobianCalculation;
      this.geometricJacobianToColumnJacobian = createGeometricJacobianToColumnJacobianMatrix(jointsForJacobianCalculation);

      this.geometricJacobianFrame = outputJoint.getFrameAfterJoint();
      this.geometricJacobian = new GeometricJacobian(jointsForJacobianCalculation, geometricJacobianFrame);

      if(DEBUG)
      {
         System.out.println("four bar output joint = " + outputJoint.getName());
         System.out.println("joints for jacobian calculation:");

         for(InverseDynamicsJoint joint : jointsForJacobianCalculation)
         {
            System.out.println(joint.getName());
         }
      }
   }

   public void computeJacobian(DenseMatrix64F jacobianToPack)
   {      
      // Geometric Jacobian - open kinematic chain Jacobian
      geometricJacobian.compute();
      geometricJacobian.changeFrame(geometricJacobianFrame);

      // Vector to go from open loop to closed loop Jacobian
      computeVectorTransformGeometricToColumnJacobian();
      
      // Column Jacobian - fourbars are a 1DOF system so the Jacobian is a column vector
      CommonOps.mult(geometricJacobian.getJacobianMatrix(), geometricJacobianToColumnJacobian, jacobianToPack);

      if (DEBUG)
      {
         System.out.println("Geometric jacobian size: " + geometricJacobian.getJacobianMatrix().getNumRows() + " , " + geometricJacobian.getJacobianMatrix().getNumCols());
         System.out.println("Column vector open to closed loop jacobian size: " + geometricJacobianToColumnJacobian.getNumRows() + " , " + geometricJacobianToColumnJacobian.getNumCols());
         System.out.println("Jacobian size: " + jacobianToPack.getNumRows() + " , " + jacobianToPack.getNumCols());
      }
   }

   public void solveLinearVelFromAngularVel(DenseMatrix64F jacobian, double inputJointVelocity)
   {
      CommonOps.scale(inputJointVelocity, jacobian);
   }

   private void computeVectorTransformGeometricToColumnJacobian()
   {
      // Vector containing angular velocity of passive joints for angular velocity of input joint (master) equal 1
      double dqA_functionOfqA = 1.0;
      fourBarCalculator.updateAnglesAndVelocitiesGivenAngleDAB(fourBarCalculator.getAngleDAB(), 1.0);
      double dqB_functionOfqA = fourBarCalculator.getAngleDtABC();
      double dqC_functionOfqA = fourBarCalculator.getAngleDtBCD();

      if (jointsForJacobianCalculation.length == 3) // Output joint is D
      {
         geometricJacobianToColumnJacobian.set(3, 1, true, new double[] {dqA_functionOfqA, dqB_functionOfqA, dqC_functionOfqA});
      }
      else if (jointsForJacobianCalculation.length == 2) // Output joint is C
      {
         geometricJacobianToColumnJacobian.set(2, 1, true, new double[] {dqA_functionOfqA, dqB_functionOfqA});
      }
      else // Output joint is B
      {
         geometricJacobianToColumnJacobian.set(1, 1, true, new double[]{dqA_functionOfqA});
      }
   }

   private DenseMatrix64F createGeometricJacobianToColumnJacobianMatrix(InverseDynamicsJoint[] jointsForJacobianCalculation)
   {
      int numberOfJointsForJacobianCalculation = jointsForJacobianCalculation.length;

      if(numberOfJointsForJacobianCalculation < 1 || numberOfJointsForJacobianCalculation > 3)
      {
         throw new RuntimeException("Illegal number of joints for jacobian calculation. Expected 1, 2, or 3 and got " + numberOfJointsForJacobianCalculation);
      }

      return new DenseMatrix64F(3, jointsForJacobianCalculation.length, true);
   }
}
