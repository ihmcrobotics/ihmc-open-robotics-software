package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.linearAlgebra.MatrixTools;

public abstract class SpatialMotionVector implements Clearable
{
   public static final int SIZE = 6;
   protected ReferenceFrame bodyFrame;
   protected ReferenceFrame baseFrame;
   protected ReferenceFrame expressedInFrame;
   protected final Vector3D linearPart = new Vector3D();
   protected final Vector3D angularPart = new Vector3D();

   public SpatialMotionVector()
   {
      this(null, null, null);
   }

   protected SpatialMotionVector(SpatialMotionVector other)
   {
      set(other);
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
    * @param linearPart linear part of the spatial motion vector
    * @param angularPart angular part of the spatial motion vector
    */
   public SpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly linearPart,
                              Vector3DReadOnly angularPart)
   {
      set(bodyFrame, baseFrame, expressedInFrame, linearPart, angularPart);
   }

   /**
    * @param bodyFrame what we're specifying the motion of
    * @param baseFrame with respect to what we're specifying the motion
    * @param linearPart linear part of the spatial motion vector expressed in the
    *           {@code expressedInFrame} to use.
    * @param angularPart angular part of the spatial motion vector expressed in the
    *           {@code expressedInFrame} to use.
    * @throws ReferenceFrameMismatchException if the linear and angular parts are not expressed in
    *            the same reference frame.
    */
   public SpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, FrameVector3DReadOnly linearPart, FrameVector3DReadOnly angularPart)
   {
      linearPart.checkReferenceFrameMatch(angularPart);

      set(bodyFrame, baseFrame, linearPart.getReferenceFrame(), linearPart, angularPart);
   }

   /**
    * Construct using a Matrix ([angular; linear])
    */
   public SpatialMotionVector(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      MatrixTools.checkMatrixDimensions(matrix, SIZE, 1);
      set(bodyFrame, baseFrame, expressedInFrame, matrix, 0);
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
      this.angularPart.set(0, array);
      this.linearPart.set(3, array);
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
   public ReferenceFrame getExpressedInFrame()
   {
      return expressedInFrame;
   }

   protected void checkAndSet(SpatialMotionVector other)
   {
      this.bodyFrame.checkReferenceFrameMatch(other.bodyFrame);
      this.baseFrame.checkReferenceFrameMatch(other.baseFrame);
      this.expressedInFrame.checkReferenceFrameMatch(other.expressedInFrame);
      set(other);
   }

   protected void set(SpatialMotionVector other)
   {
      this.bodyFrame = other.bodyFrame;
      this.baseFrame = other.baseFrame;
      this.expressedInFrame = other.expressedInFrame;

      this.linearPart.set(other.linearPart);
      this.angularPart.set(other.angularPart);
   }

   /**
    * Sets the angular velocity part of the spatial motion vector
    */
   public void setAngularPart(double x, double y, double z)
   {
      angularPart.set(x, y, z);
   }

   /**
    * Sets the angular velocity part of the spatial motion vector
    */
   public void setAngularPart(Vector3DReadOnly newAngularVelocity)
   {
      angularPart.set(newAngularVelocity);
   }

   /**
    * Sets the angular velocity part of the spatial motion vector
    */
   public void setAngularPart(FrameVector3DReadOnly newAngularVelocity)
   {
      expressedInFrame.checkReferenceFrameMatch(newAngularVelocity.getReferenceFrame());
      angularPart.set(newAngularVelocity);
   }

   /**
    * Sets the X coordinate of the angular velocity part of the spatial motion vector
    */
   public void setAngularPartX(double val)
   {
      angularPart.setX(val);
   }

   /**
    * Sets the Y coordinate of the angular velocity part of the spatial motion vector
    */
   public void setAngularPartY(double val)
   {
      angularPart.setY(val);
   }

   /**
    * Sets the Z coordinate of the angular velocity part of the spatial motion vector
    */
   public void setAngularPartZ(double val)
   {
      angularPart.setZ(val);
   }

   /**
    * Sets the X coordinate of the linear velocity part of the spatial motion vector
    */
   public void setLinearPartX(double val)
   {
      linearPart.setX(val);
   }

   /**
    * Sets the Y coordinate of the linear velocity part of the spatial motion vector
    */
   public void setLinearPartY(double val)
   {
      linearPart.setY(val);
   }

   /**
    * Sets the Z coordinate of the linear velocity part of the spatial motion vector
    */
   public void setLinearPartZ(double val)
   {
      linearPart.setZ(val);
   }

   /**
    * Sets the linear velocity part of the spatial motion vector
    */
   public void setLinearPart(double x, double y, double z)
   {
      linearPart.set(x, y, z);
   }

   /**
    * Sets the linear velocity part of the spatial motion vector
    */
   public void setLinearPart(Vector3DReadOnly newLinearVelocity)
   {
      linearPart.set(newLinearVelocity);
   }

   /**
    * Sets the linear velocity part of the spatial motion vector
    */
   public void setLinearPart(FrameVector3DReadOnly newLinearVelocity)
   {
      expressedInFrame.checkReferenceFrameMatch(newLinearVelocity.getReferenceFrame());
      linearPart.set(newLinearVelocity);
   }

   /**
    * Sets both the angular and the linear part to zero
    */
   @Override
   public void setToZero()
   {
      angularPart.setToZero();
      linearPart.setToZero();
   }

   /**
    * Invalidates this spatial motion vector by setting all its components to {@link Double#NaN}.
    */
   @Override
   public void setToNaN()
   {
      angularPart.setToNaN();
      linearPart.setToNaN();
   }

   /**
    * Tests if at least on component of this spatial motion vector is {@link Double#NaN}.
    */
   @Override
   public boolean containsNaN()
   {
      return angularPart.containsNaN() || linearPart.containsNaN();
   }

   /**
    * Sets this spatial motion vector based on a matrix ([angular part; linear part]).
    * 
    * @param matrix
    */
   public void set(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, DenseMatrix64F matrix, int rowStart)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;

      angularPart.setX(matrix.get(rowStart + 0, 0));
      angularPart.setY(matrix.get(rowStart + 1, 0));
      angularPart.setZ(matrix.get(rowStart + 2, 0));

      linearPart.setX(matrix.get(rowStart + 3, 0));
      linearPart.setY(matrix.get(rowStart + 4, 0));
      linearPart.setZ(matrix.get(rowStart + 5, 0));
   }

   /**
    * Sets this spatial motion vector based
    */
   public void set(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, Vector3DReadOnly linearPart,
                   Vector3DReadOnly angularPart)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;

      this.linearPart.set(linearPart);
      this.angularPart.set(angularPart);
   }

   /**
    * Sets this spatial motion vector based
    */
   public void set(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame, FrameVector3DReadOnly linearPart, FrameVector3DReadOnly angularPart)
   {
      linearPart.checkReferenceFrameMatch(expressedInFrame);
      angularPart.checkReferenceFrameMatch(expressedInFrame);

      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;

      this.linearPart.set(linearPart);
      this.angularPart.set(angularPart);
   }

   public void set(DenseMatrix64F matrix)
   {
      set(getBodyFrame(), getBaseFrame(), getExpressedInFrame(), matrix, 0);
   }

   public void setToZero(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
   {
      this.bodyFrame = bodyFrame;
      this.baseFrame = baseFrame;
      this.expressedInFrame = expressedInFrame;

      this.linearPart.set(0.0, 0.0, 0.0);
      this.angularPart.set(0.0, 0.0, 0.0);
   }

   /**
    * @return the angular part. For efficiency.
    */
   protected Vector3DReadOnly getAngularPart()
   {
      return angularPart;
   }

   /**
    * @return the linear part. For efficiency.
    */
   protected Vector3DReadOnly getLinearPart()
   {
      return linearPart;
   }

   /**
    * Gets the X coordinate of the angular velocity part of the spatial motion vector
    */
   public double getAngularPartX()
   {
      return angularPart.getX();
   }

   /**
    * Gets the Y coordinate of the angular velocity part of the spatial motion vector
    */
   public double getAngularPartY()
   {
      return angularPart.getY();
   }

   /**
    * Gets the Z coordinate of the angular velocity part of the spatial motion vector
    */
   public double getAngularPartZ()
   {
      return angularPart.getZ();
   }

   /**
    * Gets the X coordinate of the linear velocity part of the spatial motion vector
    */
   public double getLinearPartX()
   {
      return linearPart.getX();
   }

   /**
    * Gets the Y coordinate of the linear velocity part of the spatial motion vector
    */
   public double getLinearPartY()
   {
      return linearPart.getY();
   }

   /**
    * Gets the Z coordinate of the linear velocity part of the spatial motion vector
    */
   public double getLinearPartZ()
   {
      return linearPart.getZ();
   }

   /**
    * @return a copy of the angular part
    */
   public Vector3D getAngularPartCopy()
   {
      return new Vector3D(angularPart);
   }

   /**
    * @return a copy of the linear part
    */
   public Vector3D getLinearPartCopy()
   {
      return new Vector3D(linearPart);
   }

   /**
    * @return a matrix representation of this spatial motion vector ([angular part; linear part]).
    */
   public DenseMatrix64F toMatrix()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(SIZE, 1);
      getMatrix(matrix, 0);
      return matrix;
   }

   /**
    * Packs the components of this spatial motion vector {@code angularPart}, {@code linearPart} in
    * order in the first column of the given {@code matrix} starting at the
    * {@code startRow}<sup>th</sup> row.
    *
    * @param matrixToPack the matrix in which this vector is packed. Modified.
    * @param rowStart the first row index to start writing in the dense-matrix.
    */

   public void getMatrix(DenseMatrix64F matrixToPack, int rowStart)
   {
      getMatrix(matrixToPack, rowStart, 0);
   }

   /**
    * Packs the components of this spatial motion vector {@code angularPart}, {@code linearPart} in
    * order in the {@code columnIndex}<sup>th</sup> column of the given {@code matrix} starting at
    * the {@code startRow}<sup>th</sup> row.
    *
    * @param matrixToPack the matrix in which this vector is packed. Modified.
    * @param rowStart the first row index to start writing in the dense-matrix.
    * @param columnIndex the column index to write in the dense-matrix.
    */
   public void getMatrix(DenseMatrix64F matrixToPack, int rowStart, int columnIndex)
   {
      matrixToPack.set(rowStart + 0, columnIndex, angularPart.getX());
      matrixToPack.set(rowStart + 1, columnIndex, angularPart.getY());
      matrixToPack.set(rowStart + 2, columnIndex, angularPart.getZ());

      matrixToPack.set(rowStart + 3, columnIndex, linearPart.getX());
      matrixToPack.set(rowStart + 4, columnIndex, linearPart.getY());
      matrixToPack.set(rowStart + 5, columnIndex, linearPart.getZ());
   }

   public void getArray(double[] array, int offset)
   {
      array[offset + 0] = angularPart.getX();
      array[offset + 1] = angularPart.getY();
      array[offset + 2] = angularPart.getZ();
      array[offset + 3] = linearPart.getX();
      array[offset + 4] = linearPart.getY();
      array[offset + 5] = linearPart.getZ();
   }

   /**
    * Packs an existing Vector3d with the angular velocity part
    */
   public void getAngularPart(Vector3DBasics vectorToPack)
   {
      vectorToPack.set(this.angularPart);
   }

   /**
    * Packs an existing Vector3d with the linear velocity part
    */
   public void getLinearPart(Vector3DBasics vectorToPack)
   {
      vectorToPack.set(this.linearPart);
   }

   public void limitLinearPartMagnitude(double maximumMagnitude)
   {
      if (maximumMagnitude < 1e-7)
         linearPart.set(0.0, 0.0, 0.0);

      if (linearPart.lengthSquared() > maximumMagnitude * maximumMagnitude)
      {
         linearPart.scale(maximumMagnitude / linearPart.length());
      }
   }

   public void limitAngularPartMagnitude(double maximumMagnitude)
   {
      if (maximumMagnitude < 1e-7)
         angularPart.set(0.0, 0.0, 0.0);

      if (angularPart.lengthSquared() > maximumMagnitude * maximumMagnitude)
      {
         angularPart.scale(maximumMagnitude / angularPart.length());
      }
   }

   public double getAngularPartMagnitude()
   {
      return angularPart.length();
   }

   public double getLinearPartMagnitude()
   {
      return linearPart.length();
   }

   /**
    * Packs an existing FrameVector with the angular velocity part
    */
   public void getAngularPart(FrameVector3DBasics vectorToPack)
   {
      vectorToPack.setIncludingFrame(expressedInFrame, this.angularPart);
   }

   /**
    * Packs an existing FrameVector with the linear velocity part
    */
   public void getLinearPart(FrameVector3DBasics vectorToPack)
   {
      vectorToPack.setIncludingFrame(expressedInFrame, this.linearPart);
   }

   /**
    * Multiplies this spatial motion vector by a scalar
    * 
    * @param scalar the scaling factor
    */
   public void times(double scalar)
   {
      this.angularPart.scale(scalar);
      this.linearPart.scale(scalar);
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
      angularPart.scale(-1.0);
      linearPart.scale(-1.0);
      ReferenceFrame oldBaseFrame = baseFrame;
      baseFrame = bodyFrame;
      bodyFrame = oldBaseFrame;
   }

   public void scale(double scalar)
   {
      scaleLinearPart(scalar);
      scaleAngularPart(scalar);
   }

   public void scaleLinearPart(double scalar)
   {
      this.linearPart.scale(scalar);
   }

   public void scaleAngularPart(double scalar)
   {
      this.angularPart.scale(scalar);
   }

   public void normalize()
   {
      double norm = Math.sqrt(linearPart.lengthSquared() + angularPart.lengthSquared());
      double scalar = 1.0 / norm;
      scaleLinearPart(scalar);
      scaleAngularPart(scalar);
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
      checkReferenceFramesMatch(spatialMotionVector);

      if (!linearPart.epsilonEquals(spatialMotionVector.linearPart, epsilon))
         return false;
      if (!angularPart.epsilonEquals(spatialMotionVector.angularPart, epsilon))
         return false;

      return true;
   }

   public void checkReferenceFramesMatch(SpatialMotionVector spatialMotionVector)
   {
      checkReferenceFramesMatch(spatialMotionVector.bodyFrame, spatialMotionVector.baseFrame, spatialMotionVector.expressedInFrame);
   }

   public void checkReferenceFramesMatch(ReferenceFrame bodyFrame, ReferenceFrame baseFrame, ReferenceFrame expressedInFrame)
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
            + "Angular part: " + angularPart + "\n" + "Linear part: " + linearPart);

      return ret;
   }
   ///CLOVER:ON
}
