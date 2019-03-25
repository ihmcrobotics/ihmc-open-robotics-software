package us.ihmc.robotics.weightMatrices;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixDimensionException;

import us.ihmc.euclid.referenceFrame.FrameMatrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

/**
 * The {@code WeightMatrix3D} provides a simple way to define weights in a particular frame, which typically make up the main diagonal of a matrix. 
 * <p>
 * Given the set of weights about particular axis of interest and a reference frame to which these axes refer to, the
 * {@code WeightMatrix3D} is then able compute the corresponding 3-by-3 weight matrix.
 * </p>
 * <p>
 * The principal use-case of this class is to help users define the QP weights of each axis for a given end-effector and what frame they should be 
 * expressed in. 
 * </p>
 * <p>
 * Note that the {@link #weightFrame} is optional. It is preferable to provide it when possible,
 * but when it is absent, i.e. equal to {@code null}, the weight matrix will then be generated
 * assuming the destination frame is the same as the weight frame.
 * </p>
 */
public class WeightMatrix3D implements Tuple3DReadOnly
{
   private static final double EPSILON = 1.0e-7;
   /**
    * This frame is optional. It is preferable to provide it when possible, but when it is absent, i.e. equal
    * to {@code null}, the weight matrix will then be generated assuming the destination frame is
    * the same as the weight frame.
    * <p>
    * Note that if all of the weights are NaN or all the weights are the same, (they are all NaN by default) the weight matrix becomes
    * independent from its weight frame.
    * </p>
    */
   private ReferenceFrame weightFrame = null;
   /** The x-axis weight expressed in the weight frame. */
   private double xWeight = Double.NaN;
   /** The y-axis weight expressed in the weight frame. */
   private double yWeight = Double.NaN;
   /** The z-axis weight expressed in the weight frame. */
   private double zWeight = Double.NaN;

   /**
    * Internal object used for frame changes and to store the weights into an actual 3-by-3 weight
    * matrix.
    */
   private final transient FrameMatrix3D frameMatrix = new FrameMatrix3D();

   /**
    * Creates a new weight matrix. This weight matrix is initialized with all the weights
    * set to Double.NAN. Until the weight is changed, this weight matrix is independent from its
    * weight frame.
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
    * Sets the weight frame to {@code null}.
    * <p>
    * When the weight frame is {@code null}, the conversion into a 3-by-3 weight matrix will
    * be done regardless of the destination frame.
    * </p>
    */
   public void clearWeightFrame()
   {
      setWeightFrame(null);
   }

   /**
    * sets the frame the weights are expressed in
    * 
    * @param weightFrame the new frame to which the weights are expressed in
    */
   public void setWeightFrame(ReferenceFrame weightFrame)
   {
      this.weightFrame = weightFrame;
   }

   /**
    * sets all the axes to Double.NAN and clears the weight frame.
    * <p>
    * Until the weights are changed, this weight matrix is independent from its weight frame.
    * </p>
    */
   public void clear()
   {
      weightFrame = null;
      xWeight = Double.NaN;
      yWeight = Double.NaN;
      zWeight = Double.NaN;
   }

   /**
    * Sets this weight matrix to equal {@code other}.
    * 
    * @param other the other weight matrix. Not modified.
    */
   public void set(WeightMatrix3D other)
   {
      weightFrame = other.weightFrame;
      xWeight = other.xWeight;
      yWeight = other.yWeight;
      zWeight = other.zWeight;
   }

   /**
    * Sets the weights.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring
    * to.
    * </p>
    * 
    * @param xWeight the weight of the x Axis.
    * @param yWeight the weight of the y Axis.
    * @param zWeight the weight of the z Axis.
    */
   public void setWeights(double xWeight, double yWeight, double zWeight)
   {
      this.xWeight = xWeight;
      this.yWeight = yWeight;
      this.zWeight = zWeight;
   }

   /**
    * sets the weight of the x-axis, does not change the y and z components.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring
    * to.
    * </p>
    * 
    * @param weight the weight of the x Axis.
    */
   public void setXAxisWeight(double weight)
   {
      xWeight = weight;
   }

   /**
    * sets the weight of the y-axis, does not change the x and z components.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring
    * to.
    * </p>
    * 
    * @param weight the weight of the y Axis.
    */
   public void setYAxisWeight(double weight)
   {
      yWeight = weight;
   }

   /**
    * sets the weight of the z-axis, does not change the x and y components.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring
    * to.
    * </p>
    * 
    * @param weight the weight of the z Axis.
    */
   public void setZAxisWeight(double weight)
   {
      zWeight = weight;
   }

   /**
    * Converts this into an actual 3-by-3 weight matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * Only the block (row=0, column=0) to (row=2, column=2) of {@code weightMatrixToPack} is
    * edited to insert the weight matrix. The given dense-matrix is not reshaped.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the weight matrix is to be used.
    * @param weightMatrixToPack the dense-matrix into which the 3-by-3 weight matrix is to be
    *           inserted. Modified.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getFullWeightMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F weightMatrixToPack)
   {
      getFullWeightMatrixInFrame(destinationFrame, 0, 0, weightMatrixToPack);
   }

   /**
    * Converts this into an actual 3-by-3 weight matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * Only the block (row=startRow, column=startColumn) to (row=startRow + 2, column=startColumn+2)
    * of {@code weightMatrixToPack} is edited to insert the weight matrix. The given
    * dense-matrix is not reshaped.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the weight matrix is to be used.
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param startColumn the first column index to start writing in the dense-matrix.
    * @param weightMatrixToPack the dense-matrix into which the 3-by-3 weight matrix is to be
    *           inserted. Modified.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getFullWeightMatrixInFrame(ReferenceFrame destinationFrame, int startRow, int startColumn, DenseMatrix64F weightMatrixToPack)
   {
      int numRows = weightMatrixToPack.getNumRows();
      int numCols = weightMatrixToPack.getNumCols();
      if (numRows < startRow + 3 || numCols < startColumn + 3)
         throw new MatrixDimensionException("The weight matrix has to be at least a " + (startRow + 3) + "-by-" + (startColumn + 3) + " but was instead a "
               + numRows + "-by-" + numCols + " matrix.");

      if (canIgnoreWeightFrame(destinationFrame))
      {
         for (int row = startRow; row < startRow + 3; row++)
         {
            for (int column = startColumn; column < startColumn + 3; column++)
            {
               weightMatrixToPack.set(row, column, 0.0);
            }
         }
         weightMatrixToPack.set(startRow++, startColumn++, xWeight);
         weightMatrixToPack.set(startRow++, startColumn++, yWeight);
         weightMatrixToPack.set(startRow, startColumn, zWeight);
      }
      else
      {
         frameMatrix.setToZero(weightFrame);
         frameMatrix.setM00(xWeight);
         frameMatrix.setM11(yWeight);
         frameMatrix.setM22(zWeight);
         frameMatrix.changeFrame(destinationFrame);

         frameMatrix.get(startRow, startColumn, weightMatrixToPack);
      }
   }

   /**
    * Converts this into an actual 3-by-3 weight matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * In addition to what {@link #getFullWeightMatrixInFrame(ReferenceFrame, DenseMatrix64F)}
    * does, this method also removes the zero-rows of the given weight matrix.
    * </p>
    * <p>
    * Only the block (row=0, column=0) to (row=2, column=2) of {@code weightMatrixToPack} is
    * edited to insert the weight matrix. The given dense-matrix is not reshaped.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the weight matrix is to be used.
    * @param weightMatrixToPack the dense-matrix into which the 3-by-3 weight matrix is to be
    *           inserted.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getEfficientWeightMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F weightMatrixToPack)
   {
      getCompactWeightMatrixInFrame(destinationFrame, 0, 0, weightMatrixToPack);
   }

   /**
    * Converts this into an actual 3-by-3 weight matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * In addition to what
    * {@link #getFullWeightMatrixInFrame(ReferenceFrame, int, int, DenseMatrix64F)} does, this
    * method also removes the zero-rows of the given weight matrix.
    * </p>
    * <p>
    * Only the block (row=startRow, column=startColumn) to (row=startRow + 2, column=startColumn+2)
    * of {@code weightMatrixToPack} is edited to insert the weight matrix. The given
    * dense-matrix is not reshaped.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the weight matrix is to be used.
    * @param startRow the first row index to start writing in the dense-matrix.
    * @param startColumn the first column index to start writing in the dense-matrix.
    * @param weightMatrixToPack the dense-matrix into which the 3-by-3 weight matrix is to be
    *           inserted.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getCompactWeightMatrixInFrame(ReferenceFrame destinationFrame, int startRow, int startColumn, DenseMatrix64F weightMatrixToPack)
   {
      int numRows = weightMatrixToPack.getNumRows();
      int numCols = weightMatrixToPack.getNumCols();
      if (numRows < startRow + 3 || numCols < startColumn + 3)
         throw new MatrixDimensionException("The weight matrix has to be at least a " + (startRow + 3) + "-by-" + (startColumn + 3) + " but was instead a "
               + numRows + "-by-" + numCols + " matrix.");

      if (canIgnoreWeightFrame(destinationFrame))
      {
         for (int row = startRow; row < startRow + 3; row++)
         {
            for (int column = startColumn; column < startColumn + 3; column++)
            {
               weightMatrixToPack.set(row, column, 0.0);
            }
         }
         weightMatrixToPack.set(startRow, startColumn, xWeight);
         weightMatrixToPack.set(startRow + 1, startColumn + 1, yWeight);
         weightMatrixToPack.set(startRow + 2, startColumn + 2, zWeight);
         MatrixTools.removeZeroRows(weightMatrixToPack, startRow, startRow + 2, EPSILON);
      }
      else
      {
         frameMatrix.setToZero(weightFrame);
         frameMatrix.setM00(xWeight);
         frameMatrix.setM11(yWeight);
         frameMatrix.setM22(zWeight);
         frameMatrix.changeFrame(destinationFrame);

         frameMatrix.get(startRow, startColumn, weightMatrixToPack);
         MatrixTools.removeZeroRows(weightMatrixToPack, startRow, startRow + 2, EPSILON);
      }
   }

   /**
    * Internal method to determine if this weight frame is frame independent or not.
    * 
    * @param destinationFrame the frame into which the 3-by-3 weight matrix is about to be
    *           converted.
    * @return {@code true} if this is frame independent and thus there is no need to consider
    *         changing the frame to {@code destinationFrame}, {@code false} if the change of frame
    *         has to be performed.
    */
   private boolean canIgnoreWeightFrame(ReferenceFrame destinationFrame)
   {
      if (weightFrame == null)
         return true;
      if (weightFrame == destinationFrame)
         return true;
      if (Math.abs(xWeight) < 1.0e-6 && Math.abs(yWeight) < 1.0e-6 && Math.abs(zWeight) < EPSILON)
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
    * The reference frame to which the weight matrix is referring to.
    * <p>
    * This weight frame can be {@code null}.
    * </p>
    * 
    * @return the current weight frame.
    */
   public ReferenceFrame getWeightFrame()
   {
      return weightFrame;
   }

   @Override
   public String toString()
   {
      return "(" + xWeight + ", " + yWeight + ", " + zWeight + ") " + weightFrame;
   }

   /**
    * Returns true if any weight equals {@code SolverWeightLevels.HARD_CONSTRAINT}.
    * 
    * @return
    */
   public boolean containsHardConstraint()
   {
      if(getXAxisWeight() == SolverWeightLevels.HARD_CONSTRAINT || getYAxisWeight() == SolverWeightLevels.HARD_CONSTRAINT || getZAxisWeight() == SolverWeightLevels.HARD_CONSTRAINT)
      {
         return true;
      }

      return false;
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((frameMatrix == null) ? 0 : frameMatrix.hashCode());
      result = prime * result + ((weightFrame == null) ? 0 : weightFrame.hashCode());
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
   public boolean equals(Object object)
   {
      if (object instanceof WeightMatrix3D)
         return equals((WeightMatrix3D) object);
      else
         return false;
   }

   public boolean equals(WeightMatrix3D other)
   {
      if (other == null)
      {
         return false;
      }
      else if (other == this)
      {
         return true;
      }
      else
      {
         if (weightFrame == null)
         {
            if (other.weightFrame != null)
               return false;
         }
         else if (!weightFrame.equals(other.weightFrame))
         {
            return false;
         }

         if (Double.compare(xWeight, other.xWeight) != 0)
            return false;
         if (Double.compare(yWeight, other.yWeight) != 0)
            return false;
         if (Double.compare(zWeight, other.zWeight) != 0)
            return false;

         return true;
      }
   }

   @Override
   public double getX()
   {
      return xWeight;
   }

   @Override
   public double getY()
   {
      return yWeight;
   }

   @Override
   public double getZ()
   {
      return zWeight;
   }

   public void set(Tuple3DReadOnly weightVector)
   {
      setXAxisWeight(weightVector.getX());
      setYAxisWeight(weightVector.getY());
      setZAxisWeight(weightVector.getZ());
   }
}
