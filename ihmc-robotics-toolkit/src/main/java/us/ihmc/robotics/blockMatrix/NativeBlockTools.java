package us.ihmc.robotics.blockMatrix;

import us.ihmc.matrixlib.NativeMatrix;

public class NativeBlockTools
{
   private final NativeMatrixBlock bBlock = new NativeMatrixBlock(0, 0);

   public void mult(NativeMatrix dest, NativeBlockMatrixRMaj a, NativeBlockMatrixRMaj b)
   {
      dest.zero();

      for (int blockIdx = 0; blockIdx < a.getNumberOfBlocks(); blockIdx++)
      {
         NativeMatrixBlock aBlock = a.getBlock(blockIdx);
         int bBlockStartRow = aBlock.getStartCol();
         int bBlockEndRow = aBlock.getEndCol();
         int bBlockStartCol = aBlock.getStartRow();
         int bBlockEndCol = aBlock.getEndRow();
         b.getSubMatrix(bBlockStartRow, bBlockEndRow, bBlockStartCol, bBlockEndCol, bBlock);

         dest.multAddBlock(aBlock, bBlock, aBlock.getStartRow(), bBlock.getStartCol());
      }
   }
}
