package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.CommonOps;

import us.ihmc.robotics.linearAlgebra.MatrixTools;

public abstract class AbstractActiveSetQPSolver
{
   protected DenseMatrix64F linearEqualityConstraintA, linearEqualityConstraintATranspose, linearEqualityConstraintB;
   protected DenseMatrix64F linearInequalityConstraintA, linearInequalityConstraintB;
   protected DenseMatrix64F activeSetA, activeSetB;
   protected DenseMatrix64F quadraticCostGMatrix, quadraticCostFVector;
   protected double quadraticCostScalar;


   protected int numberOfVariablesToSolve;

//   protected double[][] quadraticCostFunctionWMatrix;
//   protected double[] quadraticCostFunctionGVector;
//
//   protected double[][] linearEqualityConstraintsAMatrix;
//   protected double[] linearEqualityConstraintsBVector;
//
//   protected double[][] linearInequalityConstraintPVectors;
//   protected double[] linearInequalityConstraintFs;

   protected boolean[] linearInequalityActiveSet;


   public AbstractActiveSetQPSolver()
   {
      numberOfVariablesToSolve = -1; //indicate unknown size
   }

   public int getNumEqualityConstraints()
   {
      return linearEqualityConstraintA.numRows;
   }

   public void setQuadraticCostFunction(double[][] quadraticCostFunctionWMatrix, double[] quadraticCostFunctionGVector, double quadraticCostScalar)
   {
      assertCorrectSize(quadraticCostFunctionWMatrix);
      assertCorrectSize(quadraticCostFunctionGVector);
      setQuadraticCostFunction(new DenseMatrix64F(quadraticCostFunctionWMatrix), MatrixTools.createVector(quadraticCostFunctionGVector), quadraticCostScalar);
   }
   
   public void setQuadraticCostFunction(DenseMatrix64F costQuadraticMatrix, DenseMatrix64F costLinearVector, double quadraticCostScalar)
   {
      setAndAssertCorrectNumberOfVariablesToSolve(costQuadraticMatrix.numCols);
      setAndAssertCorrectNumberOfVariablesToSolve(costQuadraticMatrix.numRows);
      setAndAssertCorrectNumberOfVariablesToSolve(costLinearVector.numRows);
      DenseMatrix64F symCostQuadraticMatrix = new DenseMatrix64F(costQuadraticMatrix);
      CommonOps.transpose(symCostQuadraticMatrix);
      CommonOps.add(costQuadraticMatrix, symCostQuadraticMatrix, symCostQuadraticMatrix);
      CommonOps.scale(0.5, symCostQuadraticMatrix);
      this.quadraticCostGMatrix = symCostQuadraticMatrix;
      this.quadraticCostFVector = costLinearVector;
      this.quadraticCostScalar = quadraticCostScalar;
   }

   public void setLinearEqualityConstraints(double[][] linearEqualityConstraintsAMatrix, double[] linearEqualityConstraintsBVector)
   {
      assertCorrectColumnSize(linearEqualityConstraintsAMatrix);
         if (linearEqualityConstraintsAMatrix.length != linearEqualityConstraintsBVector.length)
            throw new RuntimeException();
      setLinearEqualityConstraints(new DenseMatrix64F(linearEqualityConstraintsAMatrix), MatrixTools.createVector(linearEqualityConstraintsBVector));
   }

   public void setLinearEqualityConstraints(DenseMatrix64F linearEqualityConstraintsAMatrix, DenseMatrix64F linearEqualityConstraintsBVector)
   {
      setAndAssertCorrectNumberOfVariablesToSolve(linearEqualityConstraintsAMatrix.numCols);
      linearEqualityConstraintB = linearEqualityConstraintsBVector;
      linearEqualityConstraintA = linearEqualityConstraintsAMatrix;
      linearEqualityConstraintATranspose=CommonOps.transpose(linearEqualityConstraintA, null);
   }

   public void setLinearInequalityConstraints(double[][] linearInequalityConstraintPVectors, double[] linearInequalityConstraintFs)
   {
      assertCorrectColumnSize(linearInequalityConstraintPVectors);
      if (linearInequalityConstraintPVectors.length != linearInequalityConstraintFs.length)
         throw new RuntimeException();
      setLinearInequalityConstraints(new DenseMatrix64F(linearInequalityConstraintPVectors), MatrixTools.createVector(linearInequalityConstraintFs));
   }
   
   public void setLinearInequalityConstraints(DenseMatrix64F inequalityA, DenseMatrix64F inequalityB)
   {
      linearInequalityConstraintA=inequalityA;
      linearInequalityConstraintB=inequalityB;
      linearInequalityActiveSet = new boolean[inequalityA.getNumRows()];
      setAndAssertCorrectNumberOfVariablesToSolve(linearInequalityConstraintA.numCols);
   }

   protected int getActiveSetSize()
   {
      if (linearInequalityActiveSet == null)
         return 0;

      int ret = 0;

      for (int i = 0; i < linearInequalityActiveSet.length; i++)
      {
         if (linearInequalityActiveSet[i])
            ret++;
      }
      return ret;
   }
   
   protected int getLinearEqualityConstraintsSize()
   {
      if (linearEqualityConstraintA == null)
         return 0;

      return linearEqualityConstraintA.numRows;
   }

   protected int getLinearInequalityConstraintsSize()
   {
      if (linearInequalityConstraintA == null)
         return 0;

      return linearInequalityConstraintA.numRows;
   }
  

   private void assertCorrectSize(double[][] matrix)
   {
      if (numberOfVariablesToSolve == -1)
      {
         numberOfVariablesToSolve = matrix.length;
      }

      if (matrix.length != numberOfVariablesToSolve)
         throw new RuntimeException("matrix.length = " + matrix.length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
      if (matrix[0].length != numberOfVariablesToSolve)
         throw new RuntimeException("matrix[0].length = " + matrix[0].length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
   }

   private void assertCorrectColumnSize(double[][] matrix)
   {
      if (numberOfVariablesToSolve == -1)
      {
         numberOfVariablesToSolve = matrix[0].length;
      }

      if (matrix[0].length != numberOfVariablesToSolve)
         throw new RuntimeException("matrix[0].length = " + matrix[0].length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
   }
   
   protected void setAndAssertCorrectNumberOfVariablesToSolve(int n)
   {
      if(numberOfVariablesToSolve==-1)
      {
         numberOfVariablesToSolve = n;
      }
      
      if(n!=numberOfVariablesToSolve)
         throw new RuntimeException("incorrect NumberOfVariables size");
   }


   private void assertCorrectSize(double[] vector)
   {
      if (numberOfVariablesToSolve == -1)
      {
         numberOfVariablesToSolve = vector.length;
      }

      if (vector.length != numberOfVariablesToSolve)
         throw new RuntimeException("vector.length = " + vector.length + " != numberOfVariablesToSolve = " + numberOfVariablesToSolve);
   }


   public abstract double[] solve();
   
   

   protected static void setPartialMatrix(double[][] fromMatrix, int startRow, int startColumn, DenseMatrix64F toMatrix)
   {
      for (int i = 0; i < fromMatrix.length; i++)
      {
         for (int j = 0; j < fromMatrix[0].length; j++)
         {
            toMatrix.set(startRow + i, startColumn + j, fromMatrix[i][j]);
         }
      }
   }


   protected static void setPartialVector(double[] fromVector, int startRow, DenseMatrix64F toVector)
   {
      for (int i = 0; i < fromVector.length; i++)
      {
         toVector.set(startRow + i, 0, fromVector[i]);
      }
   }




   
   
   public void setLinearInequalityActiveSet(boolean[] linearInequalityActiveSet)
   {
      if (this.linearInequalityActiveSet.length != linearInequalityActiveSet.length) throw new RuntimeException();
      
      for (int i=0; i<linearInequalityActiveSet.length; i++)
      {
         this.linearInequalityActiveSet[i] = linearInequalityActiveSet[i];
      }
   }

   DenseMatrix64F obj=new DenseMatrix64F(1,1);
   public double getObjectiveCost(DenseMatrix64F x)
   {
      MatrixTools.multQuad(x, quadraticCostGMatrix, obj);
      CommonOps.scale(0.5, obj);
      CommonOps.multAddTransA(quadraticCostFVector, x, obj);
      return obj.get(0,0) + 0.5*quadraticCostScalar;
   }
   
   public void displayProblem()
   {
      setZeroSizeMatrixForNullFields();
      System.out.println("----------------------------------------------------------------------------------------------------");
      System.out.println("equalityA:"+ linearEqualityConstraintA);
      System.out.println("equalityB:"+ linearEqualityConstraintB);
      System.out.println("inequalityA:"+ linearInequalityConstraintA);
      System.out.println("inequalityB:"+ linearInequalityConstraintB);
      System.out.println("costQuadQ:"+quadraticCostGMatrix);
      System.out.println("costLinearF:"+quadraticCostFVector);
      System.out.println("costLinearScalar:"+quadraticCostScalar);
      System.out.println("----------------------------------------------------------------------------------------------------");
   }
   
   boolean isNullFieldSet=false; 
   public void setZeroSizeMatrixForNullFields()
   {
      if(isNullFieldSet)
         return;
      assert(numberOfVariablesToSolve>0);
      if(linearEqualityConstraintA ==null)
      {
         linearEqualityConstraintA = new DenseMatrix64F(0, numberOfVariablesToSolve);
         linearEqualityConstraintATranspose = new DenseMatrix64F(numberOfVariablesToSolve,0);
         linearEqualityConstraintB = new DenseMatrix64F(0,1);
      }
      
      if(linearInequalityConstraintA == null)
      {
         linearInequalityConstraintA = new DenseMatrix64F(0,numberOfVariablesToSolve);
         linearInequalityConstraintB = new DenseMatrix64F(0,1);
      }
      
      if(quadraticCostGMatrix==null)
      {
         quadraticCostGMatrix = new DenseMatrix64F(numberOfVariablesToSolve,numberOfVariablesToSolve);
      }
      if(quadraticCostFVector==null)
      {
         quadraticCostFVector = new DenseMatrix64F(numberOfVariablesToSolve,1);
      }
   }
   
   
   

}
