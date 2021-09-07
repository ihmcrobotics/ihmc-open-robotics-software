package us.ihmc.robotics.blockMatrix;

import gnu.trove.list.array.TIntArrayList;
import org.ejml.data.DMatrixRMaj;
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

   public int getNumRows()
   {
      int maxEnd = 0;
      for (int i = 0; i < getNumberOfBlocks(); i++)
      {
         maxEnd = Math.max(maxEnd, getBlock(i).getEndRow());
      }

      return maxEnd;
   }

   public int getNumCols()
   {
      int maxEnd = 0;
      for (int i = 0; i < getNumberOfBlocks(); i++)
      {
         maxEnd = Math.max(maxEnd, getBlock(i).getEndCol());
      }

      return maxEnd;
   }

   public void addBlock(DMatrixRMaj block, int blockStartRow, int blockStartCol)
   {
      NativeMatrixBlock matrixBlock = matrixBlocks.add();
      matrixBlock.set(block);
      matrixBlock.setStartRow(blockStartRow);
      matrixBlock.setStartCol(blockStartCol);
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

      int dstCol0 = 0;
      int dstRow0 = 0;
      for (int i = 0; i < blocks.size(); i++)
      {
         NativeMatrixBlock block = blocks.get(i);

         int srcCol0 = Math.max(0, colStart - block.getStartCol());
         int srcCol1 = Math.min(block.getNumCols(), colEnd - block.getStartCol());
         int srcRow0 = Math.max(0, rowStart - block.getStartRow());
         int srcRow1 = Math.min(block.getNumRows(), rowEnd - block.getStartRow());

         matrixToPack.insert(block, srcRow0, srcRow1, srcCol0, srcCol1, dstRow0, dstCol0);

         dstRow0 += block.getNumRows();
         dstCol0 += block.getNumCols();
      }

      return true;
   }

   private final List<NativeMatrixBlock> blocksInRange = new ArrayList<>();

   public List<NativeMatrixBlock> getBlocksThatAreVisible(int rowStart, int rowEnd, int colStart, int colEnd)
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
