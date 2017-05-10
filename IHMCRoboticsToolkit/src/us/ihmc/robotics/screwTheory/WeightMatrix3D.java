package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixDimensionException;

import us.ihmc.robotics.geometry.FrameMatrix3D;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

/**
 * The {@code WeightMatrix3D} provides a simple way to define weights in a particular frame, which typically make up the main diagonal of a matrix. 
 * <p>
 * Given the set of weights about particular axis of interest and a reference frame to which these axes refer to, the
 * {@code WeightMatrix3D} is then able compute the corresponding 3-by-3 selection matrix.
 * </p>
 * <p>
 * The principal use-case of this class is to help users define the QP weights of each axis for a given end-effector and what frame they should be 
 * expressed in. 
 * </p>
 * <p>
 * Note that the {@link #selectionFrame} is optional. It is preferable to provide it when possible,
 * but when it is absent, i.e. equal to {@code null}, the weight matrix will then be generated
 * assuming the destination frame is the same as the selection frame.
 * </p>
 */
public class WeightMatrix3D
{
   /**
    * This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal
    * to {@code null}, the selection matrix will then be generated assuming the destination frame is
    * the same as the selection frame.
    * <p>
    * Note that if all of the weights are NAN, (they are all nan by default) the selection matrix becomes
    * independent from its selection frame.
    * </p>
    */
   private ReferenceFrame selectionFrame = null;
   /** The x-axis weight expressed in the selection frame. */
   private double xWeight = Double.NaN;
   /** The y-axis weight expressed in the selection frame. */
   private double yWeight = Double.NaN;
   /** The z-axis weight expressed in the selection frame. */
   private double zWeight = Double.NaN;

   /**
    * Internal object used for frame changes and to store the weights into an actual 3-by-3 selection
    * matrix.
    */
   private final FrameMatrix3D frameMatrix = new FrameMatrix3D();

   /**
    * Creates a new weight matrix. This weight matrix is initialized with all the weights
    * set to Double.NAN. Until the selection is changed, this selection matrix is independent from its
    * selection frame.
    */
   public WeightMatrix3D()
   {
   }

   /**
    * Copy constructor.
    * 
    * @param other the weight matrix to copy. Not modified.
    */
   public WeightMatrix3D(WeightMatrix3D other)
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
    * sets the frame the weights are expressed in
    * 
    * @param selectionFrame the new frame to which the weights are expressed in
    */
   public void setSelectionFrame(ReferenceFrame selectionFrame)
   {
      this.selectionFrame = selectionFrame;
   }

   /**
    * sets all the axes to Double.NAN and clears the selection frame.
    * <p>
    * Until the selection is changed, this selection matrix is independent from its selection frame.
    * </p>
    */
   public void clear()
   {
      selectionFrame = null;
      xWeight = Double.NaN;
      yWeight = Double.NaN;
      zWeight = Double.NaN;
   }

   /**
    * Sets this selection matrix to equal {@code other}.
    * 
    * @param other the other selection matrix. Not modified.
    */
   public void set(WeightMatrix3D other)
   {
      selectionFrame = other.selectionFrame;
      xWeight = other.xWeight;
      yWeight = other.yWeight;
      zWeight = other.zWeight;
   }

   /**
    * Sets the weights.
    * <p>
    * Note that it is preferable to also set selection frame to which these weights are referring
    * to.
    * </p>
    * 
    * @param xWeight the weight of the x Axis motion.
    * @param yWeight the weight of the y Axis motion.
    * @param zWeight the weight of the z Axis motion.
    */
   public void setWeights(double xWeight, double yWeight, double zWeight)
   {
      this.xWeight = xWeight;
      this.yWeight = yWeight;
      this.zWeight = zWeight;
   }

   /**
    * sets the weight of the x-axis motion, does not change the y and z components.
    * <p>
    * Note that it is preferable to also set selection frame to which this weight is referring
    * to.
    * </p>
    * 
    * @param weight the weight of the x Axis motion.
    */
   public void setXAxisWeight(double weight)
   {
      xWeight = weight;
   }

   /**
    * sets the weight of the y-axis motion, does not change the x and z components.
    * <p>
    * Note that it is preferable to also set selection frame to which this weight is referring
    * to.
    * </p>
    * 
    * @param weight the weight of the y Axis motion.
    */
   public void setYAxisWeight(double weight)
   {
      yWeight = weight;
   }

   /**
    * sets the weight of the z-axis motion, does not change the x and y components.
    * <p>
    * Note that it is preferable to also set selection frame to which this weight is referring
    * to.
    * </p>
    * 
    * @param weight the weight of the z Axis motion.
    */
   public void setZAxisWeight(double weight)
   {
      zWeight = weight;
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
      getFullWeightMatrixInFrame(destinationFrame, 0, 0, selectionMatrixToPack);
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
   public void getFullWeightMatrixInFrame(ReferenceFrame destinationFrame, int startRow, int startColumn, DenseMatrix64F selectionMatrixToPack)
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
         selectionMatrixToPack.set(startRow++, startColumn++, xWeight);
         selectionMatrixToPack.set(startRow++, startColumn++, yWeight);
         selectionMatrixToPack.set(startRow, startColumn, zWeight);
      }
      else
      {
         frameMatrix.setToZero(selectionFrame);
         frameMatrix.setM00(xWeight);
         frameMatrix.setM11(yWeight);
         frameMatrix.setM22(zWeight);
         frameMatrix.changeFrame(destinationFrame);

         frameMatrix.getDenseMatrix(selectionMatrixToPack, startRow, startColumn);
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
    * {@link #getFullWeightMatrixInFrame(ReferenceFrame, int, int, DenseMatrix64F)} does, this
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
         selectionMatrixToPack.set(startRow, startColumn, xWeight);
         selectionMatrixToPack.set(startRow + 1, startColumn + 1, yWeight);
         selectionMatrixToPack.set(startRow + 2, startColumn + 2, zWeight);
         MatrixTools.removeZeroRows(selectionMatrixToPack, startRow, startRow + 2, 1.0e-7);
      }
      else
      {
         frameMatrix.setToZero(selectionFrame);
         frameMatrix.setM00(xWeight);
         frameMatrix.setM11(yWeight);
         frameMatrix.setM22(zWeight);
         frameMatrix.changeFrame(destinationFrame);

         frameMatrix.getDenseMatrix(selectionMatrixToPack, startRow, startColumn);
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
      if (Math.abs(xWeight) < 1.0e-6 && Math.abs(yWeight) < 1.0e-6 && Math.abs(zWeight) < 1.0e-6)
         return true;
      if (Double.isNaN(xWeight) && Double.isNaN(yWeight) && Double.isNaN(zWeight))
         return true;
      return false;
   }

   /**
    * Get the X axis weight.
    * 
    * @return the X axis weight.
    */
   public double getXAxisWeight()
   {
      return xWeight;
   }

   /**
    * Get the Y axis weight.
    * 
    * @return the Y axis weight.
    */
   public double getYAxisWeight()
   {
      return yWeight;
   }

   /**
    * Get the Z axis weight.
    * 
    * @return the Z axis weight.
    */
   public double getZAxisWeight()
   {
      return zWeight;
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
      return "(" + xWeight + ", " + yWeight + ", " + zWeight + ") " + selectionFrame;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((frameMatrix == null) ? 0 : frameMatrix.hashCode());
      result = prime * result + ((selectionFrame == null) ? 0 : selectionFrame.hashCode());
      long temp;
      temp = Double.doubleToLongBits(xWeight);
      result = prime * result + (int) (temp ^ (temp >>> 32));
      temp = Double.doubleToLongBits(yWeight);
      result = prime * result + (int) (temp ^ (temp >>> 32));
      temp = Double.doubleToLongBits(zWeight);
      result = prime * result + (int) (temp ^ (temp >>> 32));
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
      WeightMatrix3D other = (WeightMatrix3D) obj;
      if (selectionFrame == null)
      {
         if (other.selectionFrame != null)
            return false;
      }
      else if (!selectionFrame.equals(other.selectionFrame))
         return false;

      if(Double.isNaN(xWeight) ^ Double.isNaN(other.xWeight)) // xor is correct
      {
         return false;
      }

      if(Double.isNaN(yWeight) ^ Double.isNaN(other.yWeight)) // xor is correct
      {
         return false;
      }
      
      if(Double.isNaN(zWeight) ^ Double.isNaN(other.zWeight)) // xor is correct
      {
         return false;
      }
      
      if (!Double.isNaN(xWeight) && !Double.isNaN(other.xWeight) && xWeight != other.xWeight)
      {
         return false;
      }
      if (!Double.isNaN(yWeight) && !Double.isNaN(other.yWeight) && yWeight != other.yWeight)
      {
         return false;
      }
      if (!Double.isNaN(zWeight) && !Double.isNaN(other.zWeight) && zWeight != other.zWeight)
      {
         return false;
      }

      return true;
   }
}
