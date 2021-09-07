package us.ihmc.robotics.blockMatrix;

import us.ihmc.matrixlib.NativeMatrix;

import java.util.List;

public class NativeMatrixBlockTools
{
   private final NativeMatrix bBlock = new NativeMatrix(0, 0);

   public void mult(NativeMatrix dest, NativeBlockMatrixRMaj a, NativeBlockMatrixRMaj b)
   {
      dest.zero();

      int bCols = b.getNumCols();
      for (int blockIdx = 0; blockIdx < a.getNumberOfBlocks(); blockIdx++)
      {
         NativeMatrixBlock aBlock = a.getBlock(blockIdx);
         int bBlockStartRow = aBlock.getStartCol();
         int bBlockEndRow = aBlock.getEndCol();

         List<NativeMatrixBlock> blocks = b.getBlocksThatAreVisible(bBlockStartRow, bBlockEndRow, 0, bCols);

         int rowsInWindow = aBlock.getNumCols();

         for (int otherBlockIdx = 0; otherBlockIdx < blocks.size(); otherBlockIdx++)
         {
            NativeMatrixBlock fullBlock = blocks.get(otherBlockIdx);

            int dstRow0 = Math.max(0, fullBlock.getStartRow() - bBlockStartRow);
            // FIXME this is wrong

            int srcRow0 = Math.max(0, bBlockStartRow - fullBlock.getStartRow());
            int srcRow1 = Math.min(srcRow0 + rowsInWindow - dstRow0, fullBlock.getEndRow() - bBlockStartRow + srcRow0 + 1);

            // FIXME this could be way bigger than if we copied the a window as well
            bBlock.reshape(rowsInWindow, fullBlock.getNumCols());
            bBlock.zero();
            bBlock.insert(fullBlock, srcRow0, srcRow1, 0, fullBlock.getNumCols(), dstRow0, 0);

            dest.multAddBlock(aBlock, bBlock, aBlock.getStartRow(), fullBlock.getStartCol());
         }
      }
   }
}
