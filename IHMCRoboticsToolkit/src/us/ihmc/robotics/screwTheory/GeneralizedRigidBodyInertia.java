package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.linearAlgebra.MatrixTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

public abstract class GeneralizedRigidBodyInertia
{
   protected ReferenceFrame expressedInframe;
   protected final Matrix3d massMomentOfInertiaPart;
   protected final Vector3d crossPart;
   protected double mass;
   protected boolean crossPartZero;
   protected Vector3d tempVector = new Vector3d();
   private final Matrix3d tempForCrossPartTilde = new Matrix3d();

   /**
    * Default constructor; null reference frame, everything set to zero
    */
   public GeneralizedRigidBodyInertia()
   {
      this.expressedInframe = null;
      this.massMomentOfInertiaPart = new Matrix3d();
      this.crossPart = new Vector3d();
      this.mass = 0.0;
      this.crossPartZero = true;
   }

   /**
    * Construct using the reference frame in which the GeneralizedRigidBodyInertia is expressed, the mass moment of inertia matrix in that frame and the mass
    * For the case that the origin of the frame in which the GeneralizedRigidBodyInertia is expressed coincides with the center of mass.
    *
    * @param frame the reference frame in which the GeneralizedRigidBodyInertia is expressed, and also the frame of which this
    * @param massMomentOfInertia the mass moment of inertia matrix in ReferenceFrame frame
    * @param mass the mass of the rigid body to which this GeneralizedRigidBodyInertia corresponds
    */
   public GeneralizedRigidBodyInertia(ReferenceFrame frame, Matrix3d massMomentOfInertia, double mass)
   {
      this.expressedInframe = frame;
      this.massMomentOfInertiaPart = new Matrix3d(massMomentOfInertia);
      this.crossPart = new Vector3d();
      this.mass = mass;
      this.crossPartZero = true;
   }

   public GeneralizedRigidBodyInertia(ReferenceFrame frame, double Ixx, double Iyy, double Izz, double mass)
   {
      this.expressedInframe = frame;
      this.massMomentOfInertiaPart = new Matrix3d();
      setMomentOfInertia(Ixx, Iyy, Izz);
      this.crossPart = new Vector3d();
      this.mass = mass;
      this.crossPartZero = true;
   }

   public GeneralizedRigidBodyInertia(ReferenceFrame frame, Matrix3d massMomentOfInertia, double mass, Vector3d crossPart)
   {
      this.expressedInframe = frame;
      this.massMomentOfInertiaPart = new Matrix3d(massMomentOfInertia);
      this.crossPart = new Vector3d(crossPart);
      this.mass = mass;
      determineIfCrossPartIsZero();
   }

   /**
    * Copy constructor
    *
    * @param other the GeneralizedRigidBodyInertia to be copied
    */
   public GeneralizedRigidBodyInertia(GeneralizedRigidBodyInertia other)
   {
      this.expressedInframe = other.expressedInframe;
      this.massMomentOfInertiaPart = new Matrix3d(other.massMomentOfInertiaPart);
      this.crossPart = new Vector3d(other.crossPart);
      this.mass = other.mass;
      this.crossPartZero = other.crossPartZero;
   }

   /**
    * @return the frame in which this inertia is expressed
    */
   public ReferenceFrame getExpressedInFrame()
   {
      return expressedInframe;
   }

   public boolean isCrossPartZero()
   {
      return crossPartZero;
   }

   public Matrix3d getMassMomentOfInertiaPartCopy()
   {
      return new Matrix3d(massMomentOfInertiaPart);
   }

   public double getMass()
   {
      return mass;
   }

   public FramePoint getCenterOfMassOffset()
   {
      Vector3d comOffsetVector = new Vector3d(crossPart);
      comOffsetVector.negate(); // comOffset = -c
      comOffsetVector.scale(1.0 / mass); // comOffset = -1/m * c
      FramePoint centerOfMassOffset = new FramePoint(expressedInframe, comOffsetVector);
      return centerOfMassOffset;
   }

   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      massMomentOfInertiaPart.setIdentity();
      massMomentOfInertiaPart.m00 = Ixx;
      massMomentOfInertiaPart.m11 = Iyy;
      massMomentOfInertiaPart.m22 = Izz;
   }

   public void setMass(double mass)
   {
      this.mass = mass;
   }

   public void set(GeneralizedRigidBodyInertia other)
   {
      this.expressedInframe = other.expressedInframe;
      this.massMomentOfInertiaPart.set(other.massMomentOfInertiaPart);
      this.crossPart.set(other.crossPart);
      this.mass = other.mass;
      this.crossPartZero = other.crossPartZero;
   }

   /**
    * @return a 6 x 6 matrix representing this generalized inertia in matrix form [massMomentOfInertia, crossTranspose; cross, mI]
    */
   public void packMatrix(DenseMatrix64F matrixToPack)
   {

      // upper left block
      MatrixTools.setDenseMatrixFromMatrix3d(0, 0, massMomentOfInertiaPart, matrixToPack);

      // lower left
      MatrixTools.toTildeForm(tempForCrossPartTilde, crossPart);
      MatrixTools.setDenseMatrixFromMatrix3d(3, 0, tempForCrossPartTilde, matrixToPack);

      // upper right block
      tempForCrossPartTilde.transpose();
      MatrixTools.setDenseMatrixFromMatrix3d(0, 3, tempForCrossPartTilde, matrixToPack);

      // lower right block
      for (int i = 3; i < 6; i++)
      {
         matrixToPack.set(i, i, mass);
      }
   }
   
   /**
    * @return a 6 x 6 matrix representing this generalized inertia in matrix form [massMomentOfInertia, crossTranspose; cross, mI] 
    */
   public DenseMatrix64F toMatrix()
   {
      DenseMatrix64F matrix = new DenseMatrix64F(6, 6);
      packMatrix(matrix);
      return matrix;
   }

   /**
    * Changes the frame in which this GeneralizedRigidBodyInertia is expressed.
    *
    * See Duindam, Port-Based Modeling and Control for Efficient Bipedal Walking Robots, page 40, eq. (2.57)
    * http://sites.google.com/site/vincentduindam/publications
    *
    * @param newFrame the frame in which this inertia should be expressed
    */
   public void changeFrame(ReferenceFrame newFrame)
   {
      RigidBodyTransform transform = newFrame.getTransformToDesiredFrame(expressedInframe);

      Matrix3d rotation = new Matrix3d();
      transform.getRotation(rotation);

      transform.get(tempVector); // p

      // mass moment of inertia part:

      subTildeTimesTildePlusTildeTimesTildeTransposeFromSymmetricMatrix(massMomentOfInertiaPart, crossPart, tempVector); // J - (tilde(c) * tilde(p))^T - tilde(c) * tilde(p)
      subScalarTimesTildeTimesTildeFromSymmetricMatrix(massMomentOfInertiaPart, tempVector, mass); // J - (tilde(c) * tilde(p))^T - tilde(c) * tilde(p) - m * tilde(p) * tilde(p)

      massMomentOfInertiaPart.mulTransposeLeft(rotation, massMomentOfInertiaPart); // RTranspose * (J - (tilde(c) * tilde(p))^T - tilde(c) * tilde(p) - m * tilde(p) * tilde(p))
      massMomentOfInertiaPart.mul(rotation); // RTranspose * (J - (tilde(c) * tilde(p))^T - tilde(c) * tilde(p) - m * tilde(p) * tilde(p)) * R: done.

      // cross part:
      tempVector.scale(mass); // m * p
      crossPart.add(tempVector); // c + m * p
      rotation.transpose(); // RTranspose
      rotation.transform(crossPart); // RTranspose * (c + m * p)

      // mass part doesn't change

      // finally, change frame and check if cross term zero
      this.expressedInframe = newFrame;
      determineIfCrossPartIsZero();
   }

   /**
    * packs M with M - tilde(a) * tilde(b) - (tilde(a) * tilde(b))^T
    *
    * @param M a symmetric matrix - NOTE: no symmetry checks are being performed
    * @param a a vector
    * @param b another vector
    */
   private static void subTildeTimesTildePlusTildeTimesTildeTransposeFromSymmetricMatrix(Matrix3d M, Vector3d a, Vector3d b)
   {
      double axbx = a.x * b.x;
      double ayby = a.y * b.y;
      double azbz = a.z * b.z;

      M.m00 = M.m00 + 2.0 * (azbz + ayby);
      M.m01 = M.m01 - a.y * b.x - a.x * b.y;
      M.m02 = M.m02 - a.z * b.x - a.x * b.z;

      M.m10 = M.m01;
      M.m11 = M.m11 + 2.0 * (axbx + azbz);
      M.m12 = M.m12 - a.z * b.y - a.y * b.z;

      M.m20 = M.m02;
      M.m21 = M.m12;
      M.m22 = M.m22 + 2.0 * (axbx + ayby);
   }

   /**
    * packs M with M - scalar * (tilde(a) * tilde(a))
    *
    * @param M a symmetric matrix - NOTE: no symmetry checks are being performed
    * @param a a vector
    * @param scalar a scalar
    */
   private static void subScalarTimesTildeTimesTildeFromSymmetricMatrix(Matrix3d M, Vector3d a, double scalar)
   {
      double xSquared = scalar * a.x * a.x;
      double ySquared = scalar * a.y * a.y;
      double zSquared = scalar * a.z * a.z;

      double xy = scalar * a.x * a.y;
      double xz = scalar * a.x * a.z;
      double yz = scalar * a.y * a.z;

      M.m00 = M.m00 + ySquared + zSquared;
      M.m01 = M.m01 - xy;
      M.m02 = M.m02 - xz;

      M.m10 = M.m01;
      M.m11 = M.m11 + xSquared + zSquared;
      M.m12 = M.m12 - yz;

      M.m20 = M.m02;
      M.m21 = M.m12;
      M.m22 = M.m22 + xSquared + ySquared;
   }

   protected void determineIfCrossPartIsZero()
   {
      double epsilon = 1e-11;
      crossPartZero = (crossPart.lengthSquared() < epsilon);
   }

   protected void checkIsCrossPartZero()
   {
      if (!crossPartZero)
      {
         throw new RuntimeException("Cross part should be zero in order to perform this operation efficiently.");
      }
   }

   @Override
   public String toString()
   {
      DenseMatrix64F mat = new DenseMatrix64F(6, 6);
      packMatrix(mat);

      return mat.toString();
   }

}