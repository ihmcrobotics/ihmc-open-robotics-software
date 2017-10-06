package us.ihmc.convexOptimization.quadraticProgram;

import org.ejml.data.DenseMatrix64F;
import org.ejml.factory.LinearSolverFactory;
import org.ejml.interfaces.linsol.LinearSolver;
import org.ejml.ops.CommonOps;

public class BlockDiagSquareMatrix extends DenseMatrix64F
{
   private static final long serialVersionUID = 8813856249678942997L;
   
   int[] blockSizes;
   int[] blockStarts;
   
   DenseMatrix64F[] tmpMatrix;

   public BlockDiagSquareMatrix(int... blockSizes)
   {
      super(0);
      this.blockSizes = blockSizes;
      this.blockStarts = new int[getNumBlocks() + 1];
      this.tmpMatrix = new DenseMatrix64F[getNumBlocks()];
      int matrixRows = 0;
      for (int i = 0; i < getNumBlocks(); i++)
      {
         tmpMatrix[i] = new DenseMatrix64F(blockSizes[i],blockSizes[i]);
         blockStarts[i] = matrixRows;
         matrixRows += blockSizes[i];
      }

      blockStarts[blockStarts.length - 1] = matrixRows;
      super.reshape(matrixRows,matrixRows);
   }
   public int getNumBlocks()
   {
      return blockSizes.length;
   }

   public void setBlock(DenseMatrix64F srcBlock, int blockId)
   {
      setBlock(srcBlock, blockId, this);
   }

   public void setBlock(DenseMatrix64F srcBlock, int blockId, DenseMatrix64F dstMatrix)
   {
      dstMatrix.reshape(this.numRows, this.numCols);
      int startIndex = blockStarts[blockId];
      CommonOps.insert(srcBlock, dstMatrix, startIndex, startIndex);
   }

   public void packBlock(DenseMatrix64F dstBlock, int blockId, int destX0, int destY0)
   {
      int startIndex = blockStarts[blockId];
      int endIndex = blockStarts[blockId + 1];
      CommonOps.extract(this, startIndex, endIndex, startIndex, endIndex, dstBlock, destX0, destY0);
   }
   
   
   public void packInverse(LinearSolver<DenseMatrix64F> solver, BlockDiagSquareMatrix matrixToPack)
   {
      for(int i=0;i<blockSizes.length;i++)
      {
         tmpMatrix[i].reshape(blockSizes[i] , blockSizes[i]);
         packBlock(tmpMatrix[i], i, 0, 0);
         solver.setA(tmpMatrix[i]);
         solver.invert(tmpMatrix[i]);
         matrixToPack.setBlock(tmpMatrix[i] ,i);
      }
   }

   public void packInverse(LinearSolver<DenseMatrix64F> solver, DenseMatrix64F matrixToPack)
   {
      matrixToPack.zero();
      for(int i=0;i<blockSizes.length;i++)
      {
         tmpMatrix[i].reshape(blockSizes[i] , blockSizes[i]);
         packBlock(tmpMatrix[i], i, 0, 0);
         solver.setA(tmpMatrix[i]);
         solver.invert(tmpMatrix[i]);
         setBlock(tmpMatrix[i] ,i, matrixToPack);
      }
   }
   
   /**
    * c = this*b<sup>T</sup>
    * @param b
    * @param c
    */
   DenseMatrix64F multTempB = new DenseMatrix64F(0);
   DenseMatrix64F multTempC = new DenseMatrix64F(0);
   public void multTransB(DenseMatrix64F b, DenseMatrix64F c)
   {
      for(int i=0;i<blockSizes.length;i++)
      {
         for(int crow=blockStarts[i];crow<blockStarts[i+1];crow++)
         {
            int aIndex0=this.getIndex(crow,blockStarts[i]);
            for(int ccol=0; ccol<c.numCols;ccol++)
            {
               double val = 0.0;
               int aIndex=aIndex0;
               int bIndex=b.getIndex(ccol,blockStarts[i]);
               int bEnd = bIndex + blockSizes[i];
               while(bIndex < bEnd)
                     val+=this.data[aIndex++]*b.data[bIndex++];

               c.set(crow, ccol,val);                  
            }
         /*
         tmpMatrix[i].reshape(blockSizes[i] , blockSizes[i]);
         packBlock(tmpMatrix[i], i, 0, 0);
         multTempB.reshape(b.numRows, blockSizes[i]);
         multTempC.reshape(blockSizes[i], c.numCols);
         CommonOps.extract(b,  0, b.numRows, blockStarts[i], blockStarts[i+1], multTempB, 0, 0);
         CommonOps.multTransB(tmpMatrix[i], multTempB, multTempC);
         CommonOps.insert(multTempC, c, blockStarts[i], 0);         
         */
         }
      }

   }
   
   /**
    * c = this*b
    * @param b
    * @param c
    */
   public void mult(double alpha, DenseMatrix64F b, DenseMatrix64F c)
   {
      for(int i=0;i<blockSizes.length;i++)
      {
         tmpMatrix[i].reshape(blockSizes[i] , blockSizes[i]);
         packBlock(tmpMatrix[i], i, 0, 0);
         multTempB.reshape(blockSizes[i], b.numCols);
         multTempC.reshape(blockSizes[i], c.numCols);
         CommonOps.extract(b, blockStarts[i], blockStarts[i+1], 0, b.numCols, multTempB, 0, 0);
         CommonOps.mult(alpha, tmpMatrix[i], multTempB, multTempC);
         CommonOps.insert(multTempC, c, blockStarts[i], 0);
      }
   }

   public static void main(String[] arg)
   {
      BlockDiagSquareMatrix m = new BlockDiagSquareMatrix(1, 2);
      DenseMatrix64F b1 = new DenseMatrix64F(1, 1, true, 1);
      DenseMatrix64F b2 = new DenseMatrix64F(2, 2, true, 2, 3, 4, 5);

      m.setBlock(b1, 0);
      m.setBlock(b2, 1);

      System.out.println(m);
      
      m.packInverse(LinearSolverFactory.general(m.numRows, m.numCols), m);
      b1.zero();
      b2.zero();
      
      m.packBlock(b1, 0, 0,0);
      m.packBlock(b2, 1, 0,0);
      
      System.out.println(b1);
      System.out.println(b2);
      System.out.println("m=\n"+m);
   }

}
