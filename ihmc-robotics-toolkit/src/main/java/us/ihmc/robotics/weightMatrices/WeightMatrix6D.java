package us.ihmc.robotics.weightMatrices;

import org.ejml.data.DenseMatrix64F;
import org.ejml.ops.MatrixDimensionException;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * The {@code WeightMatrix3D} provides a simple way to define weights in a particular frame, which typically make up the main diagonal of a matrix. 
 * <p>
 * Given the set of weights about particular axis of interest and a reference frame to which these axes refer to, the
 * {@code WeightMatrix3D} is then able compute the corresponding 6-by-6 weight matrix.
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
public class WeightMatrix6D
{
   /**
    * The 3-by-3 weight matrix A<sub>3x3</sub> for the angular part of this weight matrix.
    */
   private final WeightMatrix3D angularWeights = new WeightMatrix3D();
   /**
    * The 3-by-3 weight matrix L<sub>3x3</sub> for the linear part of this weight matrix.
    */
   private final WeightMatrix3D linearWeights = new WeightMatrix3D();

   /**
    * Creates a new weight matrix. This weight matrix is initialized with all the weights set to NAN. Until the weights are changed, this weight matrix is independent from its
    * weight frame.
    */
   public WeightMatrix6D()
   {
   }

   /**
    * Copy constructor.
    * 
    * @param other the weight matrix to copy. Not modified.
    */
   public WeightMatrix6D(WeightMatrix6D other)
   {
      set(other);
   }

   /**
    * Sets the weight frame of both the angular and linear parts to {@code null}.
    * <p>
    * When the weight frame is {@code null}, the conversion into a 6-by-6 weight matrix will
    * be done regardless of the destination frame.
    * </p>
    */
   public void clearWeightFrame()
   {
      setWeightFrame(null);
   }

   /**
    * Sets the weight frame of the angular part to {@code null}.
    * <p>
    * When the weight frame is {@code null}, the conversion into a weight matrix will be done
    * regardless of the destination frame.
    * </p>
    * <p>
    * The linear part of this weight matrix remains unchanged.
    * </p>
    */
   public void clearAngularWeightFrame()
   {
      angularWeights.clearWeightFrame();
   }

   /**
    * Sets the weight frame of the linear part to {@code null}.
    * <p>
    * When the weight frame is {@code null}, the conversion into a weight matrix will be done
    * regardless of the destination frame.
    * </p>
    * <p>
    * The angular part of this weight matrix remains unchanged.
    * </p>
    */
   public void clearLinearWeightFrame()
   {
      linearWeights.clearWeightFrame();
   }

   /**
    * Set the frame the linear and angular weights are expressed in.
    * 
    * @param weightFrame the new frame to which the weights are expressed in
    */
   public void setWeightFrame(ReferenceFrame weightFrame)
   {
      setWeightFrames(weightFrame, weightFrame);
   }

   /**
    * Sets the weight frame for the angular and linear parts separately.
    * 
    * @param angularWeightFrame the new frame to which the angular weights are referring to.
    * @param linearWeightFrame the new frame to which the linear weights are referring to.
    */
   public void setWeightFrames(ReferenceFrame angularWeightFrame, ReferenceFrame linearWeightFrame)
   {
      angularWeights.setWeightFrame(angularWeightFrame);
      linearWeights.setWeightFrame(linearWeightFrame);
   }

   /**
    * sets the linear and angular weights to NAN and sets the weight frame to null
    * <p>
    * Until the weights are changed, this weight matrix is independent from its weight frame.
    * </p>
    */
   public void clear()
   {
      angularWeights.clear();
      linearWeights.clear();
   }

   /**
    * Sets all the angular weights to NAN and clears the angular weight frame.
    * <p>
    * The linear part remains unmodified.
    * </p>
    */
   public void clearAngularWeights()
   {
      angularWeights.clear();
   }

   /**
    * Sets all the linear weights to NAN and clears the angular weight frame.
    * <p>
    * The angular part remains unmodified.
    * </p>
    */
   public void clearLinearWeights()
   {
      linearWeights.clear();
   }

   /**
    * Sets this weight matrix to {@code other}.
    * 
    * @param other the other weight matrix. Not modified.
    */
   public void set(WeightMatrix6D other)
   {
      angularWeights.set(other.angularWeights);
      linearWeights.set(other.linearWeights);
   }

   /**
    * Sets the angular part of this weight matrix to {@code angularPart}.
    * 
    * @param angularPart the new value for the angular part of this weight matrix. Not modified.
    */
   public void setAngularPart(WeightMatrix3D angularPart)
   {
      this.angularWeights.set(angularPart);
   }

   /**
    * Sets the linear part of this weight matrix to {@code linearPart}.
    * 
    * @param linearPart the new value for the linear part of this weight matrix. Not modified.
    */
   public void setLinearPart(WeightMatrix3D linearPart)
   {
      this.linearWeights.set(linearPart);
   }

   /**
    * Sets the angular weights.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param angularXWeight angular weight of the x axis
    * @param angularYWeight angular weight of the y axis
    * @param angularZWeight angular weight of the z axis
    */
   public void setAngularWeights(double angularXWeight, double angularYWeight, double angularZWeight)
   {
      setAngularXAxisWeight(angularXWeight);
      setAngularYAxisWeight(angularYWeight);
      setAngularZAxisWeight(angularZWeight);
   }

   /**
    * Sets the angular x axis weight.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param weight the x axis weight.
    */
   public void setAngularXAxisWeight(double weight)
   {
      angularWeights.setXAxisWeight(weight);
   }

   /**
    * Sets the angular y axis weight.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param weight the y axis weight.
    */
   public void setAngularYAxisWeight(double weight)
   {
      angularWeights.setYAxisWeight(weight);
   }

   /**
    * Sets the angular z axis weight.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param weight the z axis weight.
    */
   public void setAngularZAxisWeight(double weight)
   {
      angularWeights.setZAxisWeight(weight);
   }

   /**
    * Set the angular weights.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param angularWeights the linear weights
    */
   public void setAngularWeights(Vector3DReadOnly angularWeights)
   {
      this.angularWeights.setWeights(angularWeights.getX(), angularWeights.getY(), angularWeights.getZ());
   }

   /**
    * Sets the linear weights.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param angularXWeight linear weight of the x axis
    * @param angularYWeight linear weight of the y axis
    * @param angularZWeight linear weight of the z axis
    */
   public void setLinearWeights(double linearXWeight, double linearYWeight, double linearZWeight)
   {
      setLinearXAxisWeight(linearXWeight);
      setLinearYAxisWeight(linearYWeight);
      setLinearZAxisWeight(linearZWeight);
   }

   /**
    * Sets the linear x axis weight.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param weight the x axis weight.
    */
   public void setLinearXAxisWeight(double weight)
   {
      linearWeights.setXAxisWeight(weight);
   }

   /**
    * Sets the linear y axis weight.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param weight the y axis weight.
    */
   public void setLinearYAxisWeight(double weight)
   {
      linearWeights.setYAxisWeight(weight);
   }

   /**
    * Sets the linear z axis weight.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param weight the z axis weight.
    */
   public void setLinearZAxisWeight(double weight)
   {
      linearWeights.setZAxisWeight(weight);
   }

   /**
    * Set the linear weights.
    * <p>
    * Note that it is preferable to also set weight frame to which this weight matrix is referring to.
    * </p>
    * 
    * @param linearWeights the linear weights
    */
   public void setLinearWeights(Vector3DReadOnly linearWeights)
   {
      this.linearWeights.setWeights(linearWeights.getX(), linearWeights.getY(), linearWeights.getZ());
   }

   /**
    * Converts this into an actual 6-by-6 weight matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * The given {@code weightMatrixToPack} is first reshaped to be a 6-by-6 matrix which will be
    * set to represent this.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the weight matrix is to be used.
    * @param weightMatrixToPack the dense-matrix is first reshaped to a 6-by-6 matrix and then
    *           set to represent this. Modified.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getFullWeightMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F weightMatrixToPack)
   {
      weightMatrixToPack.reshape(6, 6);
      weightMatrixToPack.zero();

      angularWeights.getFullWeightMatrixInFrame(destinationFrame, 0, 0, weightMatrixToPack);
      linearWeights.getFullWeightMatrixInFrame(destinationFrame, 3, 3, weightMatrixToPack);
   }

   /**
    * Converts this into an actual 6-by-6 weight matrix that is to be used with data expressed in
    * the {@code destinationFrame}.
    * <p>
    * In addition to what {@link #getFullWeightMatrixInFrame(ReferenceFrame, DenseMatrix64F)}
    * does, this method also removes the zero-rows of the given weight matrix. This will help to
    * improve performance especially when the resulting weight matrix is to be multiplied to a
    * large matrix.
    * </p>
    * <p>
    * The given {@code weightMatrixToPack} is first reshaped to be a 6-by-6 matrix which will be
    * set to represent this.
    * </p>
    * 
    * @param destinationFrame the reference frame in which the weight matrix is to be used.
    * @param weightMatrixToPack the dense-matrix is first reshaped to a 6-by-6 matrix and then
    *           set to represent this. The zero-rows of the resulting matrix are removed. Modified.
    * @throws MatrixDimensionException if the given matrix is too small.
    */
   public void getCompactWeightMatrixInFrame(ReferenceFrame destinationFrame, DenseMatrix64F weightMatrixToPack)
   {
      weightMatrixToPack.reshape(6, 6);
      weightMatrixToPack.zero();

      // Need to do the linear part first as rows might be removed.
      linearWeights.getCompactWeightMatrixInFrame(destinationFrame, 3, 3, weightMatrixToPack);
      angularWeights.getCompactWeightMatrixInFrame(destinationFrame, 0, 0, weightMatrixToPack);
   }

   /**
    * Get the X axis weight for the angular part.
    * 
    * @return the X axis weight for the angular part.
    */
   public double getAngularXAxisWeight()
   {
      return angularWeights.getXAxisWeight();
   }

   /**
    * Get the Y axis weight for the angular part.
    * 
    * @return the Y axis weight for the angular part.
    */
   public double getAngularYAxisWeight()
   {
      return angularWeights.getYAxisWeight();
   }

   /**
    * Get the Z axis weight for the angular part.
    * 
    * @return the Z axis weight for the angular part.
    */
   public double getAngularZAxisWeight()
   {
      return angularWeights.getZAxisWeight();
   }

   /**
    * The reference frame to which the angular weights are referring.
    * <p>
    * This weight frame can be {@code null}.
    * </p>
    * 
    * @return the current weight frame for the angular part of this weight matrix.
    */
   public ReferenceFrame getAngularWeightFrame()
   {
      return angularWeights.getWeightFrame();
   }

   /**
    * Get the X axis weight for the linear part.
    * 
    * @return the X axis weight for the linear part.
    */
   public double getLinearXAxisWeight()
   {
      return linearWeights.getXAxisWeight();
   }

   /**
    * Get the Y axis weight for the linear part.
    * 
    * @return the Y axis weight for the linear part.
    */
   public double getLinearYAxisWeight()
   {
      return linearWeights.getYAxisWeight();
   }

   /**
    * Get the Z axis weight for the linear part.
    * 
    * @return the Z axis weight for the linear part.
    */
   public double getLinearZAxisWeight()
   {
      return linearWeights.getZAxisWeight();
   }

   /**
    * The reference frame to which the linear weights are referring.
    * <p>
    * This weight frame can be {@code null}.
    * </p>
    * 
    * @return the current weight frame for the linear part of this weight matrix.
    */
   public ReferenceFrame getLinearWeightFrame()
   {
      return linearWeights.getWeightFrame();
   }

   /**
    * Returns the internal reference to the angular part of this weight matrix.
    * 
    * @return the angular part.
    */
   public WeightMatrix3D getAngularPart()
   {
      return angularWeights;
   }

   /**
    * Returns the internal reference to the linear part of this weight matrix.
    * 
    * @return the linear part.
    */
   public WeightMatrix3D getLinearPart()
   {
      return linearWeights;
   }

   @Override
   public String toString()
   {
      return "Angular: " + angularWeights + ", linear: " + linearWeights;
   }

   /**
    * Returns true if any weight equals {@code SolverWeightLevels.HARD_CONSTRAINT}.
    * 
    * @return
    */
   public boolean containsHardConstraint()
   {
      return angularWeights.containsHardConstraint() || linearWeights.containsHardConstraint();
   }

   @Override
   public int hashCode()
   {
      final int prime = 31;
      int result = 1;
      result = prime * result + ((angularWeights == null) ? 0 : angularWeights.hashCode());
      result = prime * result + ((linearWeights == null) ? 0 : linearWeights.hashCode());
      return result;
   }

   @Override
   public boolean equals(Object object)
   {
      if (this == object)
      {
         return true;
      }
      else if (object instanceof WeightMatrix6D)
      {
         WeightMatrix6D other = (WeightMatrix6D) object;

         if (angularWeights == null)
         {
            if (other.angularWeights != null)
               return false;
         }
         else if (!angularWeights.equals(other.angularWeights))
         {
            return false;
         }

         if (linearWeights == null)
         {
            if (other.angularWeights != null)
               return false;
         }
         else if (!linearWeights.equals(other.linearWeights))
         {
            return false;
         }

         return true;
      }
      else
      {
         return false;
      }
   }
}
