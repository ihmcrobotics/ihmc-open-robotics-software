package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixDimensionException;

import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

/**
 * The {@code SelectionMatrix3D} provides a simple way to define for a given application what are
 * the axes of interest.
 * <p>
 * Given the set of axes of interest and a reference frame to which these axes refer to, the
 * {@code SelectionMatrix3D} is then able compute the corresponding 3-by-3 selection matrix.
 * </p>
 * <p>
 * The principal use-case is for the controller core notably used for the walking controller. This
 * class can be used to clearly define what the axes to be controlled for a given end-effector.
 * </p>
 * <p>
 * Note that the {@link #selectionFrame} is optional. It is preferable to provide it when possible,
 * but when it is absent, i.e. equal to {@code null}, the selection matrix will then be generated
 * assuming the destination frame is the same as the selection frame.
 * </p>
 * 
 * @author Sylvain Bertrand
 *
 */
public class SelectionMatrix3D
{
   /**
    * When selecting the axes of interest, these axes refer to the selection frame axes. This frame
    * is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal
    * to {@code null}, the selection matrix will then be generated assuming the destination frame is
    * the same as the selection frame.
    * <p>
    * Note that if all the axes are selected or none of them is, the selection matrix becomes
    * independent from its selection frame.
    * </p>
    */
   private ReferenceFrame selectionFrame = null;
   /** Specifies whether the x-axis of the selection frame is an axis of interest. */
   private boolean xSelected = true;
   /** Specifies whether the y-axis of the selection frame is an axis of interest. */
   private boolean ySelected = true;
   /** Specifies whether the z-axis of the selection frame is an axis of interest. */
   private boolean zSelected = true;

   /**
    * Internal object used only to convert the three booleans into an actual 3-by-3 selection
    * matrix.
    */
   private final transient FrameMatrix3D frameMatrix = new FrameMatrix3D();

   /**
    * Creates a new selection matrix. This selection matrix is initialized with all the axes
    * selected. Until the selection is changed, this selection matrix is independent from its
    * selection frame.
    */
   public SelectionMatrix3D()
   {
   }

   /**
    * Creates and initializes a new selection matrix.
    *
    * @param selectionFrame the new frame to which the axes selection is referring to.
    * @param xSelected whether the x-axis is an axis of interest.
    * @param ySelected whether the y-axis is an axis of interest.
    * @param zSelected whether the z-axis is an axis of interest.
    * @see #setSelectionFrame(ReferenceFrame)
    * @see #setAxisSelection(boolean, boolean, boolean)
    */
   public SelectionMatrix3D(ReferenceFrame selectionFrame, boolean xSelected, boolean ySelected, boolean zSelected)
   {
      setSelectionFrame(selectionFrame);
      setAxisSelection(xSelected, ySelected, zSelected);
   }

   /**
    * Copy constructor.
    * 
    * @param other the selection matrix to copy. Not modified.
    */
   public SelectionMatrix3D(SelectionMatrix3D other)
   {
      set(other);
   }

   /**
    * Sets the selection frame to {@code null}.
    * <p>
    * When the selection frame is {@code null}, the conversion into a 3-by-3 selection matrix will
    * be done regardless of the destination frame.
    * </p>
    */
   public void clearSelectionFrame()
   {
      setSelectionFrame(null);
   }

   /**
    * Sets the selection frame such that the selection of the axes of interest now refers to the
    * axes of the given frame.
    * 
    * @param selectionFrame the new frame to which the axes selection is referring to.
    */
   public void setSelectionFrame(ReferenceFrame selectionFrame)
   {
      this.selectionFrame = selectionFrame;
   }

   /**
    * Selects all the axes and clears the selection frame.
    * <p>
    * Until the selection is changed, this selection matrix is independent from its selection frame.
    * </p>
    */
   public void resetSelection()
   {
      selectionFrame = null;
      xSelected = true;
      ySelected = true;
      zSelected = true;
   }

   /**
    * Deselects all the axes and clears the selection frame.
    * <p>
    * Until the selection is changed, this selection matrix is independent from its selection frame.
    * </p>
    */
   public void clearSelection()
   {
      selectionFrame = null;
      xSelected = false;
      ySelected = false;
      zSelected = false;
   }

   /**
    * Sets this selection matrix to equal {@code other}.
    * 
    * @param other the other selection matrix. Not modified.
    */
   public void set(SelectionMatrix3D other)
   {
      selectionFrame = other.selectionFrame;
      xSelected = other.xSelected;
      ySelected = other.ySelected;
      zSelected = other.zSelected;
   }

   /**
    * Updates the selection of the axes of interest.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param xSelected whether the x-axis is an axis of interest.
    * @param ySelected whether the y-axis is an axis of interest.
    * @param zSelected whether the z-axis is an axis of interest.
    */
   public void setAxisSelection(boolean xSelected, boolean ySelected, boolean zSelected)
   {
      this.xSelected = xSelected;
      this.ySelected = ySelected;
      this.zSelected = zSelected;
   }

   /**
    * Updates the selection state for the x-axis, does not change the y and z components.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the x-axis is an axis of interest.
    */
   public void selectXAxis(boolean select)
   {
      xSelected = select;
   }

   /**
    * Updates the selection state for the y-axis, does not change the x and z components.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the y-axis is an axis of interest.
    */
   public void selectYAxis(boolean select)
   {
      ySelected = select;
   }

   /**
    * Updates the selection state for the z-axis, does not change the x and y components.
    * <p>
    * Note that it is preferable to also set selection frame to which this selection is referring
    * to.
    * </p>
    * 
    * @param select whether the z-axis is an axis of interest.
    */
   public void selectZAxis(boolean select)
   {
      zSelected = select;
   }

   /**
    * Selects an axis based on {@code axisIndex} and updates update the selection state to
    * {@code select}.
    * <p>
    * For an {@code axisIndex} of 0, the corresponding component is {@code x}, 1 it is {@code y}, 2
    * it is {@code z}.
    * </p>
    *
    * @param axisIndex the index of the axis to update the selection state of.
    * @param select whether the chosen axis is an axis of interest.
    * @throws IndexOutOfBoundsException if {@code axisIndex} &notin; [0, 2].
    */
   public void selectAxis(int axisIndex, boolean select)
   {
      switch (axisIndex)
      {
      case 0:
         selectXAxis(select);
         break;
      case 1:
         selectYAxis(select);
         break;
      case 2:
         selectZAxis(select);
         break;
      default:
         throw new IndexOutOfBoundsException(Integer.toString(axisIndex));
      }
   }

   /**
    * Applies this selection matrix on the given vector:<br>
    * v' = S * v<br>
    * where v is the given vector, S this selection matrix, and v' the result of the selection.
    * 
    * @param vectorToBeModified the vector on which this selection matrix to be applied. Modified.
    */
   public void applySelection(FrameVector3D vectorToBeModified)
   {
      ReferenceFrame vectorFrame = vectorToBeModified.getReferenceFrame();
      boolean canIgnoreSelectionFrame = canIgnoreSelectionFrame(vectorFrame);

      if (!canIgnoreSelectionFrame)
         vectorToBeModified.changeFrame(selectionFrame);

      vectorToBeModified.setX(xSelected ? vectorToBeModified.getX() : 0.0);
      vectorToBeModified.setY(ySelected ? vectorToBeModified.getY() : 0.0);
      vectorToBeModified.setZ(zSelected ? vectorToBeModified.getZ() : 0.0);

      if (!canIgnoreSelectionFrame)
         vectorToBeModified.changeFrame(vectorFrame);
   }

   /**
    * Converts this into an actual 3-by-3 selection matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * Only the block (row=0, column=0) to (row=2, column=2) of {@code selectionMatrixToPack} is
    * edited to insert the selection matrix. The given dense-matrix is not reshaped.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the selection matrix is to be used.
    * @param selectionMatrixToPack the dense-matrix into which the 3-by-3 selection matrix is to be
    *           inserted. Modified.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getFullSelectionMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      getFullSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
   }

   /**
    * Converts this into an actual 3-by-3 selection matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * Only the block (row=startRow, column=startColumn) to (row=startRow + 2, column=startColumn+2)
    * of {@code selectionMatrixToPack} is edited to insert the selection matrix. The given
    * dense-matrix is not reshaped.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the selection matrix is to be used.
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param startColumn the first column index to start writing in the dense-matrix.
    * @param selectionMatrixToPack the dense-matrix into which the 3-by-3 selection matrix is to be
    *           inserted. Modified.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
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

         frameMatrix.get(startRow, startColumn, selectionMatrixToPack);
      }
   }

   /**
    * Converts this into an actual 3-by-3 selection matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * In addition to what {@link #getFullSelectionMatrixInFrame(ReferenceFrame, DenseMatrix64F)}
    * does, this method also removes the zero-rows of the given selection matrix.
    * </p>
    * <p>
    * Only the block (row=0, column=0) to (row=2, column=2) of {@code selectionMatrixToPack} is
    * edited to insert the selection matrix. The given dense-matrix is not reshaped.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the selection matrix is to be used.
    * @param selectionMatrixToPack the dense-matrix into which the 3-by-3 selection matrix is to be
    *           inserted.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getEfficientSelectionMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F selectionMatrixToPack)
   {
      getCompactSelectionMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
   }

   /**
    * Converts this into an actual 3-by-3 selection matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * In addition to what
    * {@link #getFullSelectionMatrixInFrame(ReferenceFrame, int, int, DenseMatrix64F)} does, this
    * method also removes the zero-rows of the given selection matrix.
    * </p>
    * <p>
    * Only the block (row=startRow, column=startColumn) to (row=startRow + 2, column=startColumn+2)
    * of {@code selectionMatrixToPack} is edited to insert the selection matrix. The given
    * dense-matrix is not reshaped.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the selection matrix is to be used.
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param startColumn the first column index to start writing in the dense-matrix.
    * @param selectionMatrixToPack the dense-matrix into which the 3-by-3 selection matrix is to be
    *           inserted.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getCompactSelectionMatrixInFrame(ReferenceFrame destinationFrame, int startRow, int startColumn, DenseMatrix64F selectionMatrixToPack)
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

         frameMatrix.get(startRow, startColumn, selectionMatrixToPack);
         MatrixTools.removeZeroRows(selectionMatrixToPack, startRow, startRow + 2, 1.0e-7);
      }
   }

   /**
    * Internal method to determine if this selection frame is frame independent or not.
    * 
    * @param destinationFrame the frame into which the 3-by-3 selection matrix is about to be
    *           converted.
    * @return {@code true} if this is frame independent and thus there is no need to consider
    *         changing the frame to {@code destinationFrame}, {@code false} if the change of frame
    *         has to be performed.
    */
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

   /**
    * Whether the x-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the x-axis.
    */
   public boolean isXSelected()
   {
      return xSelected;
   }

   /**
    * Whether the y-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the y-axis.
    */
   public boolean isYSelected()
   {
      return ySelected;
   }

   /**
    * Whether the z-axis of the current selection frame has been selected.
    * 
    * @return the selection state of the z-axis.
    */
   public boolean isZSelected()
   {
      return zSelected;
   }

   /**
    * Whether the {@code axisIndex}<sup>th</sup> axis of the current selection frame has been selected.
    * <p>
    * For an {@code axisIndex} of 0, the corresponding component is {@code x}, 1 it is {@code y}, 2
    * it is {@code z}.
    * </p>
    *
    * @param axisIndex the index of the axis to get the selection state of.
    * @return the selection state of the chosen axis.
    * @throws IndexOutOfBoundsException if {@code axisIndex} &notin; [0, 2].
    */
   public boolean isAxisSelected(int axisIndex)
   {
      switch (axisIndex)
      {
      case 0:
         return isXSelected();
      case 1:
         return isYSelected();
      case 2:
         return isZSelected();
      default:
         throw new IndexOutOfBoundsException(Integer.toString(axisIndex));
      }
   }

   /**
    * Computes the number of selections. Will be a number between 0 and 3.
    * @return the number of selected axes.
    */
   public int getNumberOfSelectedAxes()
   {
      return (isXSelected() ? 1 : 0) + (isYSelected() ? 1 : 0) + (isZSelected() ? 1 : 0);
   }

   /**
    * The reference frame to which the axis selection is referring to.
    * <p>
    * This selection frame can be {@code null}.
    * </p>
    * 
    * @return the current selection frame.
    */
   public ReferenceFrame getSelectionFrame()
   {
      return selectionFrame;
   }

   @Override
   public String toString()
   {
      return "(" + xSelected + ", " + ySelected + ", " + zSelected + ") " + selectionFrame;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((selectionFrame == null) ? 0 : selectionFrame.hashCode());
      result = prime * result + (xSelected ? 1231 : 1237);
      result = prime * result + (ySelected ? 1231 : 1237);
      result = prime * result + (zSelected ? 1231 : 1237);
      return result;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      SelectionMatrix3D other = (SelectionMatrix3D) obj;
      if (selectionFrame == null ^ other.selectionFrame == null)
      {
         return false;
      }
      else if (selectionFrame != null && selectionFrame.hashCode() != other.selectionFrame.hashCode())
         return false;
      if (xSelected != other.xSelected)
         return false;
      if (ySelected != other.ySelected)
         return false;
      if (zSelected != other.zSelected)
         return false;
      return true;
   }

}
