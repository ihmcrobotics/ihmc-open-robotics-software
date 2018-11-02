package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class SpatialForceVector
{
   public static final int SIZE = 6;
   public static final String[] AXIS_NAMES = new String[] {"xAngular", "yAngular", "zAngular", "xLinear", "yLinear", "zLinear"};
   protected ReferenceFrame expressedInFrame;
   private final FrameVector3D linearPart;
   private final FrameVector3D angularPart;
   protected final Vector3D tempVector = new Vector3D();
   private RigidBodyTransform temporaryTransformHToDesiredFrame = new RigidBodyTransform();

   /**
    * Default constructor
    */
   public SpatialForceVector()
   {
      expressedInFrame = null;
      linearPart = new FrameVector3D();
      angularPart = new FrameVector3D();
   }

   /**
    * Initializes the components of the spatial acceleration vector to zero
    * 
    * @param expressedInFrame the frame in which the spatial acceleration vector is expressed
    */
   public SpatialForceVector(ReferenceFrame expressedInFrame)
   {
      this.expressedInFrame = expressedInFrame;
      linearPart = new FrameVector3D();
      angularPart = new FrameVector3D();
   }

   /**
    * @param expressedInFrame the frame in which the spatial acceleration vector is expressed
    * @param angularPart angular part of the spatial acceleration vector
    * @param linearPart linear part part of the spatial acceleration vector
    */
   public SpatialForceVector(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      this.expressedInFrame = expressedInFrame;
      this.linearPart = new FrameVector3D (expressedInFrame,  linearPart);
      this.angularPart = new FrameVector3D(expressedInFrame,  angularPart);
   }

   /**
    * Construct using a Matrix ([angularPart; linearPart])
    */
   public SpatialForceVector(ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      angularPart = new FrameVector3D();
      linearPart = new FrameVector3D();
      setIncludingFrame(expressedInFrame, matrix);
   }

   /**
    * Construct using a double array ([angularPart; linearPart])
    */
   public SpatialForceVector(ReferenceFrame expressedInFrame, double[] matrix)
   {
      MathTools.checkIntervalContains(matrix.length, SIZE, SIZE);
      this.expressedInFrame = expressedInFrame;
      angularPart = new FrameVector3D(expressedInFrame, matrix[0], matrix[1], matrix[2]);
      linearPart = new FrameVector3D (expressedInFrame, matrix[3], matrix[4], matrix[5]);
   }

   /**
    * Copy constructor
    */
   public SpatialForceVector(SpatialForceVector other)
   {
      expressedInFrame = other.expressedInFrame;
      linearPart = new FrameVector3D(other.getLinearPart());
      angularPart = new FrameVector3D(other.getAngularPart());
   }

   public void setIncludingFrame(FrameVector3DReadOnly moment, FrameVector3DReadOnly force, FramePoint3DReadOnly pointOfApplication)
   {
      force.checkReferenceFrameMatch(pointOfApplication);
      expressedInFrame = force.getReferenceFrame();
      getLinearPart().set(force);
      getAngularPart().cross(pointOfApplication, getLinearPart());
      getAngularPart().add(moment);
   }

   /**
    * @return the frame *in which this spatial force vector is expressed
    */
   public ReferenceFrame getReferenceFrame()
   {
      return expressedInFrame;
   }

   /**
    * Sets the X coordinate of the angular part of the spatial force vector
    */
   public void setAngularPartX(double val)
   {
      getAngularPart().setX(val);
   }

   /**
    * Sets the Y coordinate of the angular part of the spatial force vector
    */
   public void setAngularPartY(double val)
   {
      getAngularPart().setY(val);
   }

   /**
    * Sets the Z coordinate of the angular part of the spatial force vector
    */
   public void setAngularPartZ(double val)
   {
      getAngularPart().setZ(val);
   }

   /**
    * Sets the X coordinate of the linear part of the spatial force vector
    */
   public void setLinearPartX(double val)
   {
      getLinearPart().setX(val);
   }

   /**
    * Sets the Y coordinate of the linear part of the spatial force vector
    */
   public void setLinearPartY(double val)
   {
      getLinearPart().setY(val);
   }

   /**
    * Sets the Z coordinate of the linear part of the spatial force vector
    */
   public void setLinearPartZ(double val)
   {
      getLinearPart().setZ(val);
   }

   public void set(SpatialForceVector other)
   {
      expressedInFrame.checkReferenceFrameMatch(other.expressedInFrame);
      setIncludingFrame(other);
   }

   public void setIncludingFrame(SpatialForceVector other)
   {
      setIncludingFrame(other.expressedInFrame, other.getAngularPart(), other.getLinearPart());
   }

   /**
    * Adds another spatial force vector to this one, after performing some reference frame checks.
    */
   public void add(SpatialForceVector other)
   {
      expressedInFrame.checkReferenceFrameMatch(other.expressedInFrame);

      getLinearPart().add(other.getLinearPart());
      getAngularPart().add(other.getAngularPart());
   }

   /**
    * Subtracts another spatial force vector from this one, after performing some reference frame
    * checks.
    */
   public void sub(SpatialForceVector other)
   {
      expressedInFrame.checkReferenceFrameMatch(other.expressedInFrame);

      getLinearPart().sub(other.getLinearPart());
      getAngularPart().sub(other.getAngularPart());
   }

   public void set(FrameVector3D angularPart, FrameVector3D linearPart)
   {
      expressedInFrame.checkReferenceFrameMatch(linearPart);
      expressedInFrame.checkReferenceFrameMatch(angularPart);
      this.getLinearPart().set(linearPart);
      this.getAngularPart().set(angularPart);
   }

   public void setIncludingFrame(ReferenceFrame expressedInFrame, Vector3DReadOnly angularPart, Vector3DReadOnly linearPart)
   {
      this.expressedInFrame = expressedInFrame;
      this.getLinearPart().set(linearPart);
      this.getAngularPart().set(angularPart);
   }

   public void setIncludingFrame(ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      setIncludingFrame(expressedInFrame, 0, matrix);
   }

   public void setIncludingFrame(ReferenceFrame expressedInFrame, int rowStart, DenseMatrix64F matrix)
   {
      MathTools.checkEquals(matrix.getNumRows(), SIZE);
      MathTools.checkEquals(matrix.getNumCols(), 1);

      this.expressedInFrame = expressedInFrame;
      getAngularPart().set(matrix.get(0, 0), matrix.get(1 + rowStart, 0), matrix.get(2 + rowStart, 0));
      getLinearPart().set(matrix.get(3 + rowStart, 0), matrix.get(4 + rowStart, 0), matrix.get(5 + rowStart, 0));
   }

   public void setIncludingFrame(ReferenceFrame expressedInFrame, double[] doubleArray)
   {
      MathTools.checkEquals(doubleArray.length, SIZE);

      this.expressedInFrame = expressedInFrame;
      getAngularPart().set(doubleArray[0], doubleArray[1], doubleArray[2]);
      getLinearPart().set(doubleArray[3], doubleArray[4], doubleArray[5]);
   }

   /**
    * @return the angular part
    */
   public FixedFrameVector3DBasics getAngularPart()
   {
      return angularPart;
   }

   /**
    * @return the linear part
    */
   public FixedFrameVector3DBasics getLinearPart()
   {
      return linearPart;
   }

   /**
    * Packs a matrix
    * 
    * @param matrix
    */
   public void get(DenseMatrix64F matrix)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), SIZE, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(0, 0, getAngularPart().getX());
      matrix.set(1, 0, getAngularPart().getY());
      matrix.set(2, 0, getAngularPart().getZ());
      matrix.set(3, 0, getLinearPart().getX());
      matrix.set(4, 0, getLinearPart().getY());
      matrix.set(5, 0, getLinearPart().getZ());
   }

   public void get(int startRow, int column, DenseMatrix64F matrix)
   {
      MathTools.checkIntervalContains(matrix.getNumRows(), SIZE, Integer.MAX_VALUE);
      MathTools.checkIntervalContains(matrix.getNumCols(), column + 1, Integer.MAX_VALUE);
      matrix.set(0, column, getAngularPart().getX());
      matrix.set(1, column, getAngularPart().getY());
      matrix.set(2, column, getAngularPart().getZ());
      matrix.set(3, column, getLinearPart().getX());
      matrix.set(4, column, getLinearPart().getY());
      matrix.set(5, column, getLinearPart().getZ());
   }

   /**
    * Changes the reference frame in which this spatial force vector is expressed See Duindam,
    * Port-Based Modeling and Control for Efficient Bipedal Walking Robots, page 36, eq. 2.47
    * http://sites.google.com/site/vincentduindam/publications
    */
   public void changeFrame(ReferenceFrame newReferenceFrame)
   {
      // trivial case:
      if (expressedInFrame == newReferenceFrame)
      {
         return;
      }

      // non-trivial case
      // essentially using the transpose of the Adjoint operator, Ad_H = [R, 0; tilde(p) * R, R] (Matlab notation), but without creating a 6x6 matrix
      // compute the relevant rotations and translations
      expressedInFrame.getTransformToDesiredFrame(temporaryTransformHToDesiredFrame, newReferenceFrame);

      // transform the torques and forces so that they are expressed in newReferenceFrame
      if (temporaryTransformHToDesiredFrame.hasRotation())
      {
         temporaryTransformHToDesiredFrame.transform(getLinearPart());
         temporaryTransformHToDesiredFrame.transform(getAngularPart());
      }

      if (temporaryTransformHToDesiredFrame.hasTranslation())
      {
         tempVector.cross(temporaryTransformHToDesiredFrame.getTranslationVector(), getLinearPart()); // p x R * f
         getAngularPart().add(tempVector);
      }

      // change this spatial force vector's expressedInFrame to newReferenceFrame
      expressedInFrame = newReferenceFrame;
   }

   public void scale(double scalar)
   {
      getLinearPart().scale(scalar);
      getAngularPart().scale(scalar);
   }

   public void negate()
   {
      getLinearPart().negate();
      getAngularPart().negate();
   }

   @Override
   public String toString()
   {
      String ret = new String("SpatialForceVector expressed in frame " + expressedInFrame + "\n" + "Angular part: " + getAngularPart() + "\n" + "Linear part: "
            + getLinearPart());

      return ret;
   }

   public void setToZero()
   {
      getAngularPart().set(0.0, 0.0, 0.0);
      getLinearPart().set(0.0, 0.0, 0.0);
   }

   public void setToZero(ReferenceFrame expressedInFrame)
   {
      this.expressedInFrame = expressedInFrame;
      setToZero();
   }

   public void get(double[] matrix)
   {
      matrix[0] = getAngularPart().getX();
      matrix[1] = getAngularPart().getY();
      matrix[2] = getAngularPart().getZ();
      matrix[3] = getLinearPart().getX();
      matrix[4] = getLinearPart().getY();
      matrix[5] = getLinearPart().getZ();
   }

   public double getLinearPartX()
   {
      return getLinearPart().getX();
   }

   public double getLinearPartY()
   {
      return getLinearPart().getY();
   }

   public double getLinearPartZ()
   {
      return getLinearPart().getZ();
   }

   public double getAngularPartX()
   {
      return getAngularPart().getX();
   }

   public double getAngularPartY()
   {
      return getAngularPart().getY();
   }

   public double getAngularPartZ()
   {
      return getAngularPart().getZ();
   }
}
