package us.ihmc.robotics.screwTheory;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public class SpatialForceVector
{
   public static final int SIZE = 6;
   public static final String[] AXIS_NAMES = new String[]{"xAngular","yAngular","zAngular","xLinear","yLinear","zLinear"};
   protected ReferenceFrame expressedInFrame;
   protected final Vector3d linearPart;
   protected final Vector3d angularPart;
   protected final Vector3d tempVector = new Vector3d();
   private RigidBodyTransform temporaryTransformHToDesiredFrame = new RigidBodyTransform();

   /**
    * Default constructor
    */
   public SpatialForceVector()
   {
      this.expressedInFrame = null;
      this.linearPart = new Vector3d();
      this.angularPart = new Vector3d();
   }

   /**
    * Initializes the components of the spatial acceleration vector to zero
    * @param expressedInFrame the frame in which the spatial acceleration vector is expressed
    */
   public SpatialForceVector(ReferenceFrame expressedInFrame)
   {
      this.expressedInFrame = expressedInFrame;
      this.linearPart = new Vector3d();
      this.angularPart = new Vector3d();
   }

   /**
    * @param expressedInFrame the frame in which the spatial acceleration vector is expressed
    * @param linearPart linear part part of the spatial acceleration vector
    * @param angularPart angular part of the spatial acceleration vector
    */
   public SpatialForceVector(ReferenceFrame expressedInFrame, Vector3d linearPart, Vector3d angularPart)
   {
      this.expressedInFrame = expressedInFrame;
      this.linearPart = new Vector3d(linearPart);
      this.angularPart = new Vector3d(angularPart);
   }

   /**
    * Construct using a Matrix ([angularPart; linearPart])
    */
   public SpatialForceVector(ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      this.angularPart = new Vector3d();
      this.linearPart = new Vector3d();
      set(expressedInFrame, matrix);
   }

   /**
    * Construct using a double array ([angularPart; linearPart])
    */
   public SpatialForceVector(ReferenceFrame expressedInFrame, double[] matrix)
   {
      MathTools.checkIfInRange(matrix.length, SIZE, SIZE);
      this.expressedInFrame = expressedInFrame;
      this.angularPart = new Vector3d(matrix[0], matrix[1], matrix[2]);
      this.linearPart = new Vector3d(matrix[3], matrix[4], matrix[5]);
   }

   /**
    * Copy constructor
    */
   public SpatialForceVector(SpatialForceVector other)
   {
      this.expressedInFrame = other.expressedInFrame;
      this.linearPart = new Vector3d(other.linearPart);
      this.angularPart = new Vector3d(other.angularPart);
   }

   /**
    * Construct using linear part and arm
    */
   public static SpatialForceVector createUsingArm(ReferenceFrame expressedInFrame, Vector3d linearPart, Vector3d arm)
   {
      SpatialForceVector ret = new SpatialForceVector(expressedInFrame);
      ret.setLinearPart(linearPart);
      ret.getAngularPart().cross(arm, linearPart);
      return ret;
   }
   
   public void setUsingArm(ReferenceFrame expressedInFrame, Vector3d linearPart, Vector3d arm)
   {
      this.expressedInFrame = expressedInFrame;
      this.linearPart.set(linearPart);
      this.angularPart.cross(arm, linearPart);
   }

   public void setIncludingFrame(FrameVector force, FramePoint pointOfApplication)
   {
      force.checkReferenceFrameMatch(pointOfApplication);
      expressedInFrame = force.getReferenceFrame();
      force.get(linearPart);
      pointOfApplication.get(tempVector);
      angularPart.cross(tempVector, linearPart);
   }

   public void setIncludingFrame(FrameVector force, FrameVector moment, FramePoint pointOfApplication)
   {
      force.checkReferenceFrameMatch(pointOfApplication);
      expressedInFrame = force.getReferenceFrame();
      force.get(linearPart);
      pointOfApplication.get(tempVector);
      angularPart.cross(tempVector, linearPart);
      angularPart.add(moment.getVector());
   }

   /**
    * @return the frame *in which this spatial force vector is expressed
    */
   public ReferenceFrame getExpressedInFrame()
   {
      return expressedInFrame;
   }

   /**
    * Sets the angular part of the twist
    */
   public void setAngularPart(FrameVector angularPart)
   {
      expressedInFrame.checkReferenceFrameMatch(angularPart);
      angularPart.get(this.angularPart);
   }

   /**
    * Sets the angular part of the twist
    */
   public void setAngularPart(Vector3d angularPart)
   {
      this.angularPart.set(angularPart);
   }

   /**
    * Sets the linear part of the spatial force vector
    */
   public void setLinearPart(FrameVector linearPart)
   {
      expressedInFrame.checkReferenceFrameMatch(linearPart);
      linearPart.get(this.linearPart);
   }

   /**
    * Sets the linear part of the spatial force vector
    */
   public void setLinearPart(Vector3d linearPart)
   {
      this.linearPart.set(linearPart);
   }

   /**
    * Sets the X coordinate of the angular part of the spatial force vector
    */
   public void setAngularPartX(double val)
   {
      angularPart.setX(val);
   }

   /**
    * Sets the Y coordinate of the angular part of the spatial force vector
    */
   public void setAngularPartY(double val)
   {
      angularPart.setY(val);
   }

   /**
    * Sets the Z coordinate of the angular part of the spatial force vector
    */
   public void setAngularPartZ(double val)
   {
      angularPart.setZ(val);
   }

   /**
    * Sets the X coordinate of the linear part of the spatial force vector
    */
   public void setLinearPartX(double val)
   {
      linearPart.setX(val);
   }

   /**
    * Sets the Y coordinate of the linear part of the spatial force vector
    */
   public void setLinearPartY(double val)
   {
      linearPart.setY(val);
   }

   /**
    * Sets the Z coordinate of the linear part of the spatial force vector
    */
   public void setLinearPartZ(double val)
   {
      linearPart.setZ(val);
   }

   public void checkAndSet(SpatialForceVector other)
   {
      this.expressedInFrame.checkReferenceFrameMatch(other.expressedInFrame);
      set(other);
   }

   public void set(SpatialForceVector other)
   {
      set(other.expressedInFrame, other.linearPart, other.angularPart);
   }

   /**
    * Adds another spatial force vector to this one, after performing some reference frame checks.
    */
   public void add(SpatialForceVector other)
   {
      this.expressedInFrame.checkReferenceFrameMatch(other.expressedInFrame);

      this.linearPart.add(other.linearPart);
      this.angularPart.add(other.angularPart);
   }

   /**
    * Subtracts another spatial force vector from this one, after performing some reference frame checks.
    */
   public void sub(SpatialForceVector other)
   {
      this.expressedInFrame.checkReferenceFrameMatch(other.expressedInFrame);

      this.linearPart.sub(other.linearPart);
      this.angularPart.sub(other.angularPart);
   }

   public void set(FrameVector linearPart, FrameVector angularPart)
   {
      expressedInFrame.checkReferenceFrameMatch(linearPart);
      expressedInFrame.checkReferenceFrameMatch(angularPart);
      linearPart.get(this.linearPart);
      angularPart.get(this.angularPart);
   }

   public void set(ReferenceFrame expressedInFrame, Vector3d linearPart, Vector3d angularPart)
   {
      this.expressedInFrame = expressedInFrame;
      this.linearPart.set(linearPart);
      this.angularPart.set(angularPart);
   }

   public void set(ReferenceFrame expressedInFrame, DenseMatrix64F matrix)
   {
      set(expressedInFrame, matrix, 0);
   }

   public void set(ReferenceFrame expressedInFrame, DenseMatrix64F matrix, int rowStart)
   {
      MathTools.checkIfEqual(matrix.getNumRows(), SIZE);
      MathTools.checkIfEqual(matrix.getNumCols(), 1);

      this.expressedInFrame = expressedInFrame;
      this.angularPart.set(matrix.get(0, 0), matrix.get(1 + rowStart, 0), matrix.get(2 + rowStart, 0));
      this.linearPart.set(matrix.get(3 + rowStart, 0), matrix.get(4 + rowStart, 0), matrix.get(5 + rowStart, 0));
   }

   public void set(ReferenceFrame expressedInFrame, double[] doubleArray)
   {
      MathTools.checkIfEqual(doubleArray.length, SIZE);

      this.expressedInFrame = expressedInFrame;
      this.angularPart.set(doubleArray[0], doubleArray[1], doubleArray[2]);
      this.linearPart.set(doubleArray[3], doubleArray[4], doubleArray[5]);
   }
   
   /**
    * Adds to the angular part of the spatial force vector
    */
   public void addAngularPart(Vector3d angularPart)
   {
      this.angularPart.add(angularPart);
   }

   public void subAngularPart(Vector3d angularPart)
   {
      this.angularPart.sub(angularPart);
   }

   public void subAngularPart(FrameVector angularPart)
   {
      expressedInFrame.checkReferenceFrameMatch(angularPart);
      this.angularPart.sub(angularPart.getVector());
   }
   
   /**
    * Adds to the force part of the spatial force vector
    */
   public void addLinearPart(Vector3d linearPart)
   {
      this.linearPart.add(linearPart);
   }

   public void addLinearPart(FrameVector linearPart)
   {
      expressedInFrame.checkReferenceFrameMatch(linearPart);
      this.linearPart.add(linearPart.getVector());
   }

   public void subLinearPart(Vector3d linearPart)
   {
      this.linearPart.sub(linearPart);
   }

   public void subLinearPart(FrameVector linearPart)
   {
      expressedInFrame.checkReferenceFrameMatch(linearPart);
      this.linearPart.sub(linearPart.getVector());
   }

   /**
    *  @return copy of the angular part as a FrameVector
    */
   public FrameVector getAngularPartAsFrameVectorCopy()
   {
      return new FrameVector(expressedInFrame, angularPart);
   }
   
   /**
    *  @return copy of the angular part as a FrameVector
    */
   public FrameVector getLinearPartAsFrameVectorCopy()
   {
      return new FrameVector(expressedInFrame, linearPart);
   }
   
   /**
    * @return copy of the angular part
    */
   public Vector3d getAngularPartCopy()
   {
      return new Vector3d(angularPart);
   }

   /**
    * @return copy of the linear part
    */
   public Vector3d getLinearPartCopy()
   {
      return new Vector3d(linearPart);
   }

   /**
    * @return the angular part
    */
   public Vector3d getAngularPart()
   {
      return angularPart;
   }

   /**
    * @return the linear part
    */
   public Vector3d getLinearPart()
   {
      return linearPart;
   }

   /**
    * Packs a matrix
    * @param matrix
    */
   public void getMatrix(DenseMatrix64F matrix)
   {
      MathTools.checkIfInRange(matrix.getNumRows(), SIZE, Integer.MAX_VALUE);
      MathTools.checkIfInRange(matrix.getNumCols(), 1, Integer.MAX_VALUE);
      matrix.set(0, 0, angularPart.getX());
      matrix.set(1, 0, angularPart.getY());
      matrix.set(2, 0, angularPart.getZ());
      matrix.set(3, 0, linearPart.getX());
      matrix.set(4, 0, linearPart.getY());
      matrix.set(5, 0, linearPart.getZ());
   }
   
   public void getMatrixColumn(DenseMatrix64F matrix, int column)
   {
      MathTools.checkIfInRange(matrix.getNumRows(), SIZE, Integer.MAX_VALUE);
      MathTools.checkIfInRange(matrix.getNumCols(), column+1, Integer.MAX_VALUE);
      matrix.set(0, column, angularPart.getX());
      matrix.set(1, column, angularPart.getY());
      matrix.set(2, column, angularPart.getZ());
      matrix.set(3, column, linearPart.getX());
      matrix.set(4, column, linearPart.getY());
      matrix.set(5, column, linearPart.getZ());
   }


   /**
    * @return a matrix representation of this spatial force vector. [linearPart; angularPart]
    */
   public DenseMatrix64F toDenseMatrix()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(SIZE, 1);
      getMatrix(matrix);

      return matrix;
   }

   /**
    * Packs an existing Vector3d with the angular part
    */
   public void getAngularPart(Vector3d vectorToPack)
   {
      vectorToPack.set(this.angularPart);
   }

   /**
    * Packs an existing FrameVector with the angular part. 
    * The FrameVector must be in the same frame as the expressedInFrame
    */
   public void getAngularPart(FrameVector vectorToPack)
   {
      this.getExpressedInFrame().checkReferenceFrameMatch(vectorToPack.getReferenceFrame());
      vectorToPack.set(this.angularPart);
   }

   /**
    * Packs an existing FrameVector with the angular part. 
    */
   public void getAngularPartIncludingFrame(FrameVector vectorToPack)
   {
      vectorToPack.setIncludingFrame(getExpressedInFrame(), this.angularPart);
   }

   /**
    * Packs an existing Vector3d with the linear part
    */
   public void getLinearPart(Vector3d vectorToPack)
   {
      vectorToPack.set(this.linearPart);
   }

   /**
    * Packs an existing FrameVector with the linear part. 
    * The FrameVector must be in the same frame as the expressedInFrame
    */
   public void getLinearPart(FrameVector vectorToPack)
   {
      this.getExpressedInFrame().checkReferenceFrameMatch(vectorToPack.getReferenceFrame());
      vectorToPack.set(this.linearPart);
   }

   /**
    * Packs an existing FrameVector with the linear part. 
    */
   public void getLinearPartIncludingFrame(FrameVector vectorToPack)
   {
      vectorToPack.setIncludingFrame(getExpressedInFrame(), linearPart);
   }

   /**
    * Multiplies this spatial force vector by a scalar
    * @param scalar the scaling factor
    */
   public void times(double scalar)
   {
      this.angularPart.scale(scalar);
      this.linearPart.scale(scalar);
   }

   /**
    * Changes the reference frame in which this spatial force vector is expressed
    * See Duindam, Port-Based Modeling and Control for Efficient Bipedal Walking Robots, page 36, eq. 2.47
    * http://sites.google.com/site/vincentduindam/publications
    */
   public void changeFrame(ReferenceFrame newReferenceFrame)
   {
      // trivial case:
      if (this.expressedInFrame == newReferenceFrame)
      {
         return;
      }

      // non-trivial case
      // essentially using the transpose of the Adjoint operator, Ad_H = [R, 0; tilde(p) * R, R] (Matlab notation), but without creating a 6x6 matrix
      // compute the relevant rotations and translations
      expressedInFrame.getTransformToDesiredFrame(temporaryTransformHToDesiredFrame, newReferenceFrame);
      temporaryTransformHToDesiredFrame.getTranslation(tempVector); // p

      // transform the torques and forces so that they are expressed in newReferenceFrame
      temporaryTransformHToDesiredFrame.transform(linearPart);
      temporaryTransformHToDesiredFrame.transform(angularPart);
      tempVector.cross(tempVector, linearPart); // p x R * f
      angularPart.add(tempVector);

      // change this spatial force vector's expressedInFrame to newReferenceFrame
      this.expressedInFrame = newReferenceFrame;
   }

   public void scaleLinearPart(double scalar)
   {
      this.linearPart.scale(scalar);
   }

   public void scaleAngularPart(double scalar)
   {
      this.angularPart.scale(scalar);
   }

   public void scale(double scalar)
   {
      scaleLinearPart(scalar);
      scaleAngularPart(scalar);
   }
   
   public void negate()
   {
      this.linearPart.negate();
      this.angularPart.negate();
   }
   
   @Override
   public String toString()
   {
      String ret = new String("SpatialForceVector expressed in frame " + expressedInFrame + "\n" + "Angular part: " + angularPart + "\n"
                              + "Linear part: " + linearPart);

      return ret;
   }

   public void setToZero()
   {
      this.angularPart.set(0.0, 0.0, 0.0);
      this.linearPart.set(0.0, 0.0, 0.0);
   }
   
   public void setToZero(ReferenceFrame expressedInFrame)
   {
      this.expressedInFrame = expressedInFrame;
      setToZero();
   }

   public void getMatrix(double[] matrix)
   {
      matrix[0] = angularPart.getX();
      matrix[1] = angularPart.getY();
      matrix[2] = angularPart.getZ();
      matrix[3] = linearPart.getX();
      matrix[4] = linearPart.getY();
      matrix[5] = linearPart.getZ();
   }

   public double getLinearPartX()
   {
      return linearPart.getX();
   }
   
   public double getLinearPartY()
   {
      return linearPart.getY();
   }
   
   public double getLinearPartZ()
   {
      return linearPart.getZ();
   }

   public double getAngularPartX()
   {
      return angularPart.getX();
   }
   
   public double getAngularPartY()
   {
      return angularPart.getY();
   }
   
   public double getAngularPartZ()
   {
      return angularPart.getZ();
   }
}
