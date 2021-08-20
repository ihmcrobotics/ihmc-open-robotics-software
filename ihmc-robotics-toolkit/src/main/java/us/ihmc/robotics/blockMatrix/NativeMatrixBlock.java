package us.ihmc.robotics.blockMatrix;

import us.ihmc.matrixlib.NativeMatrix;

public class NativeMatrixBlock extends NativeMatrix
{
   private int startRow;
   private int startCol;

   public NativeMatrixBlock(int rows, int cols)
   {
      super(rows, cols);
   }

   public void setStartRow(int startRow)
   {
      this.startRow = startRow;
   }

   public void setStartCol(int startCol)
   {
      this.startCol = startCol;
   }

   public int getStartRow()
   {
      return startRow;
   }

   public int getStartCol()
   {
      return startCol;
   }

   public int getEndRow()
   {
      return startRow + getNumRows();
   }

   public int getEndCol()
   {
      return startCol + getNumCols();
   }
}
