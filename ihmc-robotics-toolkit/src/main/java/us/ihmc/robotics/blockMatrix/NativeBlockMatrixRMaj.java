package us.ihmc.robotics.blockMatrix;

import gnu.trove.list.array.TIntArrayList;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.matrixlib.NativeMatrix;

import java.util.ArrayList;
import java.util.List;

/**
 * This separates the matrices into blocks, where there is only one block per set of rows
 */
public class NativeBlockMatrixRMaj
{
   private final RecyclingArrayList<NativeMatrixBlock> matrixBlocks = new RecyclingArrayList<>(() -> new NativeMatrixBlock(0, 0));

   public void clear()
   {
      matrixBlocks.clear();
   }

   public int getNumberOfBlocks()
   {
      return matrixBlocks.size();
   }

   public NativeMatrixBlock getBlock(int blockIndex)
   {
      return matrixBlocks.get(blockIndex);
   }

   public boolean getSubMatrix(int rowStart, int rowEnd, int colStart, int colEnd, NativeMatrixBlock matrixToPack)
   {
      List<NativeMatrixBlock> blocks = getBlocksThatAreVisible(rowStart, rowEnd, colStart, colEnd);

      if (blocks.size() == 0)
         return false;

      matrixToPack.reshape(rowEnd - rowStart, colEnd - colStart);
      matrixToPack.zero();
      matrixToPack.setStartRow(rowStart);
      matrixToPack.setStartCol(colStart);

      for (int i = 0; i < blocks.size(); i++)
      {
         NativeMatrixBlock block = blocks.get(i);

         int srcX0 = ;
         int srcX1 = ;
         int srcY0 = ;
         int srcY1 = ;
         int dstX0;
         int dstY0;
         matrixToPack.insert(block, srcY0, srcY1, srcX0, srcX1, dstY0, dstX0);
      }
   }

   private final List<NativeMatrixBlock> blocksInRange = new ArrayList<>();

   private List<NativeMatrixBlock> getBlocksThatAreVisible(int rowStart, int rowEnd, int colStart, int colEnd)
   {
      blocksInRange.clear();

      for (int i = 0; i < matrixBlocks.size(); i++)
      {
         NativeMatrixBlock block = matrixBlocks.get(i);
         boolean inRowRange = isBlockWithinRange(block.getStartRow(), block.getEndRow(), rowStart, rowEnd);
         if (!inRowRange)
            continue;
         if (isBlockWithinRange(block.getStartCol(), block.getEndCol(), colStart, colEnd))
            blocksInRange.add(block);
      }

      return blocksInRange;
   }

   private boolean isBlockWithinRange(int blockStart, int blockEnd, int start, int end)
   {
      if (blockStart > end)
         return false;
      if (blockEnd < start)
         return false;

      return true;
   }

}
