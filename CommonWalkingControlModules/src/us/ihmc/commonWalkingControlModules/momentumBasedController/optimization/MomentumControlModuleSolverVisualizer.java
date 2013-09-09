package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;

public class MomentumControlModuleSolverVisualizer implements MomentumControlModuleSolverListener
{
   private static final boolean printForViz = false; 
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final DoubleYoVariable largestCheckJQEqualsZeroNumber = new DoubleYoVariable("largestCheckJQEqualsZeroNumber", registry);
   
   private DenseMatrix64F centroidalMomentumMatrix, momentumDotEquationRightHandSide;
   private DenseMatrix64F jPrimary, pPrimary, primaryMotionConstraintCheck, checkJQEqualsZeroAfterSetConstraint;
   private DenseMatrix64F jSecondary, pSecondary, weightMatrixSecondary;
   private DenseMatrix64F jointAccelerations;
   private double optimizationValue;
   
   public MomentumControlModuleSolverVisualizer(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }
   
   public void setCentroidalMomentumMatrix(DenseMatrix64F a)
   {
      centroidalMomentumMatrix = a;
     if (printForViz) System.out.println("CentroidalMomentumMatrix = " + a);
   }

   public void setMomentumDotEquationRightHandSide(DenseMatrix64F b)
   {
      momentumDotEquationRightHandSide = b;
      if (printForViz) System.out.println("MomentumDotEquationRightHandSide = " + b);
   }

   public void setPrimaryMotionConstraintJMatrix(DenseMatrix64F jPrimary)
   {
      this.jPrimary = jPrimary;
      if (printForViz) System.out.println("PrimaryMotionConstraintJMatrix = " + jPrimary);
   }

   public void setPrimaryMotionConstraintPVector(DenseMatrix64F pPrimary)
   {
      this.pPrimary = pPrimary;
      if (printForViz) System.out.println("PrimaryMotionConstraintPVector = " + pPrimary);
   }

   public void setSecondaryMotionConstraintJMatrix(DenseMatrix64F jSecondary)
   {
      this.jSecondary = jSecondary;
      if (printForViz) System.out.println("SecondaryMotionConstraintJMatrix = " + jSecondary);
   }

   public void setSecondaryMotionConstraintPVector(DenseMatrix64F pSecondary)
   {
      this.pSecondary = pSecondary;
      if (printForViz) System.out.println("SecondaryMotionConstraintPVector = " + pSecondary);
   }

   public void setSecondaryMotionConstraintWeightMatrix(DenseMatrix64F weightMatrixSecondary)
   {
      this.weightMatrixSecondary = weightMatrixSecondary;
      if (printForViz) System.out.println("SecondaryMotionConstraintWeightMatrix = " + weightMatrixSecondary);
   }

   public void setJointAccelerationSolution(DenseMatrix64F jointAccelerations)
   {      
      this.jointAccelerations = jointAccelerations;
      if (printForViz) System.out.println("JointAccelerationSolution = " + jointAccelerations);
   }

   public void setOptimizationValue(double optimizationValue)
   {
      this.optimizationValue = optimizationValue;
      if (printForViz) System.out.println("OptimizationValue = " + optimizationValue);
   }
   
   public void reviewSolution()
   {
      DenseMatrix64F ATimesVDot = new DenseMatrix64F(centroidalMomentumMatrix.getNumRows(), jointAccelerations.getNumCols());
      CommonOps.mult(centroidalMomentumMatrix, jointAccelerations, ATimesVDot);
      
      if (printForViz) System.out.println("\n\nATimesVDot = " + ATimesVDot);
      if (printForViz) System.out.println("MomentumDotEquationRightHandSide = " + momentumDotEquationRightHandSide);

   }

   public void setPrimaryMotionConstraintCheck(DenseMatrix64F check)
   {
      this.primaryMotionConstraintCheck = check;
      if (printForViz) System.out.println("PrimaryMotionConstraintCheck = " + primaryMotionConstraintCheck);

   }

   public void setCheckJQEqualsZeroAfterSetConstraint(DenseMatrix64F checkJQEqualsZeroAfterSetConstraint)
   {      
      this.checkJQEqualsZeroAfterSetConstraint = checkJQEqualsZeroAfterSetConstraint;
      if (printForViz) System.out.println("CheckJQEqualsZeroAfterSetConstraint = " + checkJQEqualsZeroAfterSetConstraint);
      
      largestCheckJQEqualsZeroNumber.set(getMaxAbsoluteElement(checkJQEqualsZeroAfterSetConstraint));
   }

   private double getMaxAbsoluteElement(DenseMatrix64F matrix)
   {
      int rows = matrix.getNumRows();
      int columns = matrix.getNumCols();
      
      double maxValue = Double.NEGATIVE_INFINITY;
      
      for (int row = 0; row<rows; row++)
      {
         for (int column=0; column<columns; column++)
         {
            double element = Math.abs(matrix.get(row, column));
            
            if (element > maxValue) maxValue = element;
         }
      }
      
      return maxValue;
   }
   
   

}
