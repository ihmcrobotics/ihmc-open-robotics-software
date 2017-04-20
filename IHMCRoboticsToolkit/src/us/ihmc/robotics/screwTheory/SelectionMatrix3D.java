package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixDimensionException;

import us.ihmc.robotics.geometry.FrameMatrix3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SelectionMatrix3D
{
   private ReferenceFrame selectionFrame = null;
   private boolean xSelected = true;
   private boolean ySelected = true;
   private boolean zSelected = true;

   private final FrameMatrix3D frameMatrix = new FrameMatrix3D();

   public SelectionMatrix3D()
   {
   }

   public void clearSelectionFrame()
   {
      setSelectionFrame(null);
   }

   public void setSelectionFrame(ReferenceFrame selectionFrame)
   {
      this.selectionFrame = selectionFrame;
   }

   public void resetSelection()
   {
      selectionFrame = null;
      xSelected = true;
      ySelected = true;
      zSelected = true;
   }

   public void clearSelection()
   {
      selectionFrame = null;
      xSelected = false;
      ySelected = false;
      zSelected = false;
   }

   public void setAxisSelection(boolean xSelected, boolean ySelected, boolean zSelected)
   {
      this.xSelected = xSelected;
      this.ySelected = ySelected;
      this.zSelected = zSelected;
   }

   public void selectXAxis(boolean select)
   {
      xSelected = select;
   }

   public void selectYAxis(boolean select)
   {
      ySelected = select;
   }

   public void selectZAxis(boolean select)
   {
      zSelected = select;
   }

   public void getFullSelectionMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      getFullSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
   }

   public void getFullSelectionMatrixInFrame(ReferenceFrame destinationFrame, int startRow, int startColumn, DenseMatrix64F selectionMatrixToPack)
   {
      int numRows = selectionMatrixToPack.getNumRows();
      int numCols = selectionMatrixToPack.getNumCols();
      if (numRows < startRow + 3 || numCols < startColumn + 3)
         throw new MatrixDimensionException("The selection matrix has to be at least a " + (startRow + 3) + "-by-" + (startColumn + 3) + " but was instead a "
               + numRows + "-by-" + numCols + " matrix.");

      if (canIgnoreSelectionFrame(destinationFrame))
      {
         for (int row = startRow; row < startRow + 3; row++)
         {
            for (int column = startColumn; column < startColumn + 3; column++)
            {
               selectionMatrixToPack.set(row, column, 0.0);
            }
         }
         selectionMatrixToPack.set(startRow++, startColumn++, xSelected ? 1.0 : 0.0);
         selectionMatrixToPack.set(startRow++, startColumn++, ySelected ? 1.0 : 0.0);
         selectionMatrixToPack.set(startRow, startColumn, zSelected ? 1.0 : 0.0);
      }
      else
      {
         frameMatrix.setToZero(selectionFrame);
         frameMatrix.setM00(xSelected ? 1.0 : 0.0);
         frameMatrix.setM11(ySelected ? 1.0 : 0.0);
         frameMatrix.setM22(zSelected ? 1.0 : 0.0);
         frameMatrix.changeFrame(destinationFrame);

         frameMatrix.getDenseMatrix(selectionMatrixToPack, startRow, startColumn);
      }
   }

   public void getEfficientSelectionMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      getEfficientSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
   }

   public void getEfficientSelectionMatrixInFrame(ReferenceFrame destinationFrame, int startRow, int startColumn, DenseMatrix64F selectionMatrixToPack)
   {
      int numRows = selectionMatrixToPack.getNumRows();
      int numCols = selectionMatrixToPack.getNumCols();
      if (numRows < startRow + 3 || numCols < startColumn + 3)
         throw new MatrixDimensionException("The selection matrix has to be at least a " + (startRow + 3) + "-by-" + (startColumn + 3) + " but was instead a "
               + numRows + "-by-" + numCols + " matrix.");

      if (canIgnoreSelectionFrame(destinationFrame))
      {
         for (int row = startRow; row < startRow + 3; row++)
         {
            for (int column = startColumn; column < startColumn + 3; column++)
            {
               selectionMatrixToPack.set(row, column, 0.0);
            }
         }
         if (!zSelected)
            MatrixTools.removeRow(selectionMatrixToPack, startRow + 2);
         else
            selectionMatrixToPack.set(startRow + 2, startColumn + 2, 1.0);

         if (!ySelected)
            MatrixTools.removeRow(selectionMatrixToPack, startRow + 1);
         else
            selectionMatrixToPack.set(startRow + 1, startColumn + 1, 1.0);

         if (!xSelected)
            MatrixTools.removeRow(selectionMatrixToPack, startRow);
         else
            selectionMatrixToPack.set(startRow, startColumn, 1.0);
      }
      else
      {
         frameMatrix.setToZero(selectionFrame);
         frameMatrix.setM00(xSelected ? 1.0 : 0.0);
         frameMatrix.setM11(ySelected ? 1.0 : 0.0);
         frameMatrix.setM22(zSelected ? 1.0 : 0.0);
         frameMatrix.changeFrame(destinationFrame);

         frameMatrix.getDenseMatrix(selectionMatrixToPack, startRow, startColumn);
         MatrixTools.removeZeroRows(selectionMatrixToPack, startRow, startRow + 2, 1.0e-7);
      }
   }

   private boolean canIgnoreSelectionFrame(ReferenceFrame destinationFrame)
   {
      if (selectionFrame == null)
         return true;
      if (selectionFrame == destinationFrame)
         return true;
      if (xSelected && ySelected && zSelected)
         return true;
      if (!xSelected && !ySelected && !zSelected)
         return true;
      return false;
   }

   public boolean isXSelected()
   {
      return xSelected;
   }

   public boolean isYSelected()
   {
      return ySelected;
   }

   public boolean isZSelected()
   {
      return zSelected;
   }

   public ReferenceFrame getSelectionFrame()
   {
      return selectionFrame;
   }
}
