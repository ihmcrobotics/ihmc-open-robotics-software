package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public abstract class SpatialMotionVector implements Clearable
{
   public static final int SIZE = 6;
   protected ReferenceFrame bodyFrame;
   protected ReferenceFrame baseFrame;
   protected ReferenceFrame expressedInFrame;
   private final FrameVector3D linearPart = new FrameVector3D();
   private final FrameVector3D angularPart = new FrameVector3D();

   public SpatialMotionVector()
   {
      this(null, null, null);
   }

   protected SpatialMotionVector(SpatialMotionVector other)
   {
      setIncludingFrame(other);
   }

   /**
    * Initiates the angular velocity and linear velocity to zero
    * 
    * @param bodyFrame what we're specifying the motion of
    * @param baseFrame with respect to what we're specifying the motion
    * @param expressedInFrame in which reference frame the motion is expressed
    */
   public SpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;
   }

   /**
    * @param bodyFrame what we're specifying the motion of
    * @param baseFrame with respect to what we're specifying the motion
    * @param expressedInFrame in which reference frame the motion is expressed
    * @param angularPart angular part of the spatial motion vector
    * @param linearPart linear part of the spatial motion vector
    */
   public SpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart,
                              Vector3DReadOnly linearPart)
   {
      setIncludingFrame(bodyFrame, baseFrame, expressedInFrame, angularPart, linearPart);
   }

   /**
    * @param bodyFrame what we're specifying the motion of
    * @param baseFrame with respect to what we're specifying the motion
    * @param angularPart angular part of the spatial motion vector expressed in the
    *           {@code expressedInFrame} to use.
    * @param linearPart linear part of the spatial motion vector expressed in the
    *           {@code expressedInFrame} to use.
    * @throws ReferenceFrameMismatchException if the linear and angular parts are not expressed in
    *            the same reference frame.
    */
   public SpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      linearPart.checkReferenceFrameMatch(angularPart);

      setIncludingFrame(bodyFrame, baseFrame, linearPart.getReferenceFrame(), angularPart, linearPart);
   }

   /**
    * Construct using a Matrix ([angular; linear])
    */
   public SpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      MatrixTools.checkMatrixDimensions(matrix, SIZE, 1);
      setIncludingFrame(bodyFrame, baseFrame, expressedInFrame, 0, matrix);
   }

   /**
    * Construct using a double array ([angular; linear])
    */
   public SpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, double[] array)
   {
      MathTools.checkEquals(SIZE, array.length);
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;
      this.getAngularPart().set(0, array);
      this.getLinearPart().set(3, array);
   }

   /**
    * @return the frame *of which* this object expresses the motion
    */
   public ReferenceFrame getBodyFrame()
   {
      return bodyFrame;
   }

   /**
    * @return the frame *with respect to which* this spatial motion vector is defined
    */
   public ReferenceFrame getBaseFrame()
   {
      return baseFrame;
   }

   /**
    * @return the reference frame *in which this spatial motion vector is expressed
    */
   public ReferenceFrame getReferenceFrame()
   {
      return expressedInFrame;
   }

   protected void checkAndSet(SpatialMotionVector other)
   {
      this.bodyFrame.checkReferenceFrameMatch(other.bodyFrame);
      this.baseFrame.checkReferenceFrameMatch(other.baseFrame);
      this.expressedInFrame.checkReferenceFrameMatch(other.expressedInFrame);
      setIncludingFrame(other);
   }

   protected void setIncludingFrame(SpatialMotionVector other)
   {
      this.bodyFrame = other.bodyFrame;
      this.baseFrame = other.baseFrame;
      this.expressedInFrame = other.expressedInFrame;

      this.getLinearPart().set(other.getLinearPart());
      this.getAngularPart().set(other.getAngularPart());
   }

   /**
    * Sets the X coordinate of the angular velocity part of the spatial motion vector
    */
   public void setAngularPartX(double val)
   {
      getAngularPart().setX(val);
   }

   /**
    * Sets the Y coordinate of the angular velocity part of the spatial motion vector
    */
   public void setAngularPartY(double val)
   {
      getAngularPart().setY(val);
   }

   /**
    * Sets the Z coordinate of the angular velocity part of the spatial motion vector
    */
   public void setAngularPartZ(double val)
   {
      getAngularPart().setZ(val);
   }

   /**
    * Sets the X coordinate of the linear velocity part of the spatial motion vector
    */
   public void setLinearPartX(double val)
   {
      getLinearPart().setX(val);
   }

   /**
    * Sets the Y coordinate of the linear velocity part of the spatial motion vector
    */
   public void setLinearPartY(double val)
   {
      getLinearPart().setY(val);
   }

   /**
    * Sets the Z coordinate of the linear velocity part of the spatial motion vector
    */
   public void setLinearPartZ(double val)
   {
      getLinearPart().setZ(val);
   }

   /**
    * Sets both the angular and the linear part to zero
    */
   @Override
   public void setToZero()
   {
      getAngularPart().setToZero();
      getLinearPart().setToZero();
   }

   /**
    * Invalidates this spatial motion vector by setting all its components to {@link Double#NaN}.
    */
   @Override
   public void setToNaN()
   {
      getAngularPart().setToNaN();
      getLinearPart().setToNaN();
   }

   /**
    * Tests if at least on component of this spatial motion vector is {@link Double#NaN}.
    */
   @Override
   public boolean containsNaN()
   {
      return getAngularPart().containsNaN() || getLinearPart().containsNaN();
   }

   /**
    * Sets this spatial motion vector based on a matrix ([angular part; linear part]).
    * @param matrix
    */
   public void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, int rowStart, DenseMatrix64F matrix)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;

      getAngularPart().setX(matrix.get(rowStart + 0, 0));
      getAngularPart().setY(matrix.get(rowStart + 1, 0));
      getAngularPart().setZ(matrix.get(rowStart + 2, 0));

      getLinearPart().setX(matrix.get(rowStart + 3, 0));
      getLinearPart().setY(matrix.get(rowStart + 4, 0));
      getLinearPart().setZ(matrix.get(rowStart + 5, 0));
   }

   /**
    * Sets this spatial motion vector based
    */
   public void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart,
                   Vector3DReadOnly linearPart)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;

      this.getLinearPart().set(linearPart);
      this.getAngularPart().set(angularPart);
   }

   /**
    * Sets this spatial motion vector based
    */
   public void setIncludingFrame(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, FrameVector3DReadOnly angularPart, FrameVector3DReadOnly linearPart)
   {
      linearPart.checkReferenceFrameMatch(expressedInFrame);
      angularPart.checkReferenceFrameMatch(expressedInFrame);

      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;

      this.getLinearPart().set(linearPart);
      this.getAngularPart().set(angularPart);
   }

   public void set(DenseMatrix64F matrix)
   {
      setIncludingFrame(getBodyFrame(), getBaseFrame(), getReferenceFrame(), 0, matrix);
   }

   public void setToZero(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;

      this.getLinearPart().set(0.0, 0.0, 0.0);
      this.getAngularPart().set(0.0, 0.0, 0.0);
   }

   /**
    * @return the angular part. For efficiency.
    */
   public FixedFrameVector3DBasics getAngularPart()
   {
      return angularPart;
   }

   /**
    * @return the linear part. For efficiency.
    */
   public FixedFrameVector3DBasics getLinearPart()
   {
      return linearPart;
   }

   /**
    * Gets the X coordinate of the angular velocity part of the spatial motion vector
    */
   public double getAngularPartX()
   {
      return getAngularPart().getX();
   }

   /**
    * Gets the Y coordinate of the angular velocity part of the spatial motion vector
    */
   public double getAngularPartY()
   {
      return getAngularPart().getY();
   }

   /**
    * Gets the Z coordinate of the angular velocity part of the spatial motion vector
    */
   public double getAngularPartZ()
   {
      return getAngularPart().getZ();
   }

   /**
    * Gets the X coordinate of the linear velocity part of the spatial motion vector
    */
   public double getLinearPartX()
   {
      return getLinearPart().getX();
   }

   /**
    * Gets the Y coordinate of the linear velocity part of the spatial motion vector
    */
   public double getLinearPartY()
   {
      return getLinearPart().getY();
   }

   /**
    * Gets the Z coordinate of the linear velocity part of the spatial motion vector
    */
   public double getLinearPartZ()
   {
      return getLinearPart().getZ();
   }

   /**
    * Packs the components of this spatial motion vector {@code angularPart}, {@code linearPart} in
    * order in the first column of the given {@code matrix} starting at the
    * {@code startRow}<sup>th</sup> row.
    * @param rowStart the first row index to start writing in the dense-matrix.
    * @param matrixToPack the matrix in which this vector is packed. Modified.
    */

   public void get(int rowStart, DenseMatrix64F matrixToPack)
   {
      get(rowStart, 0, matrixToPack);
   }

   /**
    * Packs the components of this spatial motion vector {@code angularPart}, {@code linearPart} in
    * order in the {@code columnIndex}<sup>th</sup> column of the given {@code matrix} starting at
    * the {@code startRow}<sup>th</sup> row.
    * @param rowStart the first row index to start writing in the dense-matrix.
    * @param columnIndex the column index to write in the dense-matrix.
    * @param matrixToPack the matrix in which this vector is packed. Modified.
    */
   public void get(int rowStart, int columnIndex, DenseMatrix64F matrixToPack)
   {
      matrixToPack.set(rowStart + 0, columnIndex, getAngularPart().getX());
      matrixToPack.set(rowStart + 1, columnIndex, getAngularPart().getY());
      matrixToPack.set(rowStart + 2, columnIndex, getAngularPart().getZ());

      matrixToPack.set(rowStart + 3, columnIndex, getLinearPart().getX());
      matrixToPack.set(rowStart + 4, columnIndex, getLinearPart().getY());
      matrixToPack.set(rowStart + 5, columnIndex, getLinearPart().getZ());
   }

   public void get(int offset, double[] array)
   {
      array[offset + 0] = getAngularPart().getX();
      array[offset + 1] = getAngularPart().getY();
      array[offset + 2] = getAngularPart().getZ();
      array[offset + 3] = getLinearPart().getX();
      array[offset + 4] = getLinearPart().getY();
      array[offset + 5] = getLinearPart().getZ();
   }

   /**
    * Inverts the spatial motion vector, i.e.: Given the spatial motion of frame A with respect to
    * frame B, expressed in frame C, this method computes the spatial motion of frame B with respect
    * to frame A, expressed in frame C (or vice versa), by just taking the additive inverse.
    *
    * See Duindam, Port-Based Modeling and Control for Efficient Bipedal Walking Robots, page 25,
    * lemma 2.8 (d) (and (e), for generalizing to any expressedInFrame)
    * http://sites.google.com/site/vincentduindam/publications
    *
    * Duindam proves this fact for twists, but differentiating the statement results in the same
    * thing for derivatives of twists
    */
   public void invert()
   {
      getAngularPart().scale(-1.0);
      getLinearPart().scale(-1.0);
      ReferenceFrame oldBaseFrame = baseFrame;
      baseFrame = bodyFrame;
      bodyFrame = oldBaseFrame;
   }

   public void scale(double scalar)
   {
      this.getLinearPart().scale(scalar);
      this.getAngularPart().scale(scalar);
   }

   public void normalize()
   {
      double norm = Math.sqrt(getLinearPart().lengthSquared() + getAngularPart().lengthSquared());
      double scalar = 1.0 / norm;
      this.getLinearPart().scale(scalar);
      this.getAngularPart().scale(scalar);
   }

   /**
    * Returns true if this SpatialMotionVector and the input SpatialMotionVector have the same
    * HumanoidReferenceFrames and if the L-infinite distance between their linear parts and angular
    * parts are less than or equal to the epsilon parameter. If any of the frames are different,
    * throws a RuntimeException. Otherwise returns false. The L-infinite distance is equal to
    * MAX[abs(x1-x2), abs(y1-y2), abs(z1-z2)].
    * 
    * @param spatialMotionVector the SpatialAccelerationVector to be compared to this
    *           SpatialAccelerationVector
    * @param epsilon the threshold value
    * @return true or false
    */
   public boolean epsilonEquals(SpatialMotionVector spatialMotionVector, double epsilon)
   {
      checkReferenceFrameMatch(spatialMotionVector);

      if (!getLinearPart().epsilonEquals(spatialMotionVector.getLinearPart(), epsilon))
         return false;
      if (!getAngularPart().epsilonEquals(spatialMotionVector.getAngularPart(), epsilon))
         return false;

      return true;
   }

   public void checkReferenceFrameMatch(SpatialMotionVector spatialMotionVector)
   {
      checkReferenceFrameMatch(spatialMotionVector.bodyFrame, spatialMotionVector.baseFrame, spatialMotionVector.expressedInFrame);
   }

   public void checkReferenceFrameMatch(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      if (this.bodyFrame != bodyFrame)
         throw new ReferenceFrameMismatchException("bodyFrame mismatch: this.bodyFrame = " + this.bodyFrame + ", other bodyFrame = " + bodyFrame);
      if (this.baseFrame != baseFrame)
         throw new ReferenceFrameMismatchException("baseFrame mismatch: this.baseFrame = " + this.baseFrame + ", other baseFrame = " + baseFrame);
      if (this.expressedInFrame != expressedInFrame)
         throw new ReferenceFrameMismatchException("expressedInFrame mismatch: this.expressedInFrame = " + this.expressedInFrame + ", other expressedInFrame = "
               + expressedInFrame);
   }

   public boolean containsNaN(SpatialMotionVector spatialMotionVector)
   {
      if (Double.isNaN(getLinearPartX()))
         return true;
      if (Double.isNaN(getLinearPartY()))
         return true;
      if (Double.isNaN(getLinearPartZ()))
         return true;

      if (Double.isNaN(getAngularPartX()))
         return true;
      if (Double.isNaN(getAngularPartY()))
         return true;
      if (Double.isNaN(getAngularPartZ()))
         return true;

      return false;
   }

   ///CLOVER:OFF
   @Override
   public String toString()
   {
      String ret = new String("Spatial motion of " + bodyFrame + ", with respect to " + baseFrame + ", expressed in frame " + expressedInFrame + "\n"
            + "Angular part: " + getAngularPart() + "\n" + "Linear part: " + getLinearPart());

      return ret;
   }
   ///CLOVER:ON
}
