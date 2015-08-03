package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;


public class MomentumControlModuleSolverVisualizer implements MomentumControlModuleSolverListener
{
   private static final boolean printForViz = false; 
   
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   
   private final DoubleYoVariable largestCheckJQEqualsZeroNumber = new DoubleYoVariable("largestCheckJQEqualsZeroNumber", registry);
   
   private final DenseMatrix64F centroidalMomentumMatrixCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F momentumDotEquationRightHandSideCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F momentumSubspaceCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F jPrimaryCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F pPrimaryCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F primaryMotionConstraintCheckCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F checkJQEqualsZeroAfterSetConstraintCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F jSecondaryCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F pSecondaryCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F weightMatrixSecondaryCopy = new DenseMatrix64F(1,1);
   private final DenseMatrix64F jointAccelerationsCopy = new DenseMatrix64F(1,1);
   private double optimizationValue;
   
   public MomentumControlModuleSolverVisualizer(YoVariableRegistry parentRegistry)
   {
      parentRegistry.addChild(registry);
   }
   
   public void setCentroidalMomentumMatrix(DenseMatrix64F centroidalMomentumMatrix, DenseMatrix64F momentumDotEquationRightHandSide, DenseMatrix64F momentumSubspace)
   {
     this.centroidalMomentumMatrixCopy.set(centroidalMomentumMatrix);
     if (printForViz) System.out.println("CentroidalMomentumMatrix = " + centroidalMomentumMatrixCopy);
     
     this.momentumDotEquationRightHandSideCopy.set(momentumDotEquationRightHandSide);
     if (printForViz) System.out.println("MomentumDotEquationRightHandSide = " + momentumDotEquationRightHandSideCopy);
     
     this.momentumSubspaceCopy.set(momentumSubspace);
     if (printForViz) System.out.println("momentumSubspace = " + momentumSubspaceCopy);
   }


   public void setPrimaryMotionConstraintJMatrix(DenseMatrix64F jPrimary)
   {
      this.jPrimaryCopy.set(jPrimary);
      if (printForViz) System.out.println("PrimaryMotionConstraintJMatrix = " + jPrimaryCopy);
   }

   public void setPrimaryMotionConstraintPVector(DenseMatrix64F pPrimary)
   {
      this.pPrimaryCopy.set(pPrimary);
      if (printForViz) System.out.println("PrimaryMotionConstraintPVector = " + pPrimaryCopy);
   }

   public void setSecondaryMotionConstraintJMatrix(DenseMatrix64F jSecondary)
   {
      this.jSecondaryCopy.set(jSecondary);
      if (printForViz) System.out.println("SecondaryMotionConstraintJMatrix = " + jSecondaryCopy);
   }

   public void setSecondaryMotionConstraintPVector(DenseMatrix64F pSecondary)
   {
      this.pSecondaryCopy.set(pSecondary);
      if (printForViz) System.out.println("SecondaryMotionConstraintPVector = " + pSecondaryCopy);
   }

   public void setSecondaryMotionConstraintWeightMatrix(DenseMatrix64F weightMatrixSecondary)
   {
      this.weightMatrixSecondaryCopy.set(weightMatrixSecondary);
      if (printForViz) System.out.println("SecondaryMotionConstraintWeightMatrix = " + weightMatrixSecondaryCopy);
   }

   public void setJointAccelerationSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerations)
   {      
      this.jointAccelerationsCopy.set(jointAccelerations);
      if (printForViz) System.out.println("JointAccelerationSolution = " + jointAccelerationsCopy);
   }

   public void setOptimizationValue(double optimizationValue)
   {
      this.optimizationValue = optimizationValue;
      if (printForViz) System.out.println("OptimizationValue = " + optimizationValue);
   }
   
   public void reviewSolution()
   {
      DenseMatrix64F ATimesVDot = new DenseMatrix64F(centroidalMomentumMatrixCopy.getNumRows(), jointAccelerationsCopy.getNumCols());
      CommonOps.mult(centroidalMomentumMatrixCopy, jointAccelerationsCopy, ATimesVDot);
      
      if (printForViz) System.out.println("\n\nATimesVDot = " + ATimesVDot);
      if (printForViz) System.out.println("MomentumDotEquationRightHandSide = " + momentumDotEquationRightHandSideCopy);

   }

   public void setPrimaryMotionConstraintCheck(DenseMatrix64F check)
   {
      this.primaryMotionConstraintCheckCopy.set(check);
      if (printForViz) System.out.println("PrimaryMotionConstraintCheck = " + primaryMotionConstraintCheckCopy);

   }

   public void setCheckJQEqualsZeroAfterSetConstraint(DenseMatrix64F checkJQEqualsZeroAfterSetConstraint)
   {      
      this.checkJQEqualsZeroAfterSetConstraintCopy.set(checkJQEqualsZeroAfterSetConstraint);
      if (printForViz) System.out.println("CheckJQEqualsZeroAfterSetConstraint = " + checkJQEqualsZeroAfterSetConstraintCopy);
      
      largestCheckJQEqualsZeroNumber.set(getMaxAbsoluteElement(checkJQEqualsZeroAfterSetConstraintCopy));
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
