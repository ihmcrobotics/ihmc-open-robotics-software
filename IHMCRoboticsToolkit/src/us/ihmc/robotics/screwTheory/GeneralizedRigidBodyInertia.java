package us.ihmc.robotics.screwTheory;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public abstract class GeneralizedRigidBodyInertia
{
   protected ReferenceFrame expressedInframe;
   protected final Matrix3D massMomentOfInertiaPart;
   protected final Vector3D crossPart;
   protected double mass;
   protected boolean crossPartZero;
   protected Vector3D tempVector = new Vector3D();
   private final Matrix3D tempForCrossPartTilde = new Matrix3D();

   /**
    * Default constructor; null reference frame, everything set to zero
    */
   public GeneralizedRigidBodyInertia()
   {
      this.expressedInframe = null;
      this.massMomentOfInertiaPart = new Matrix3D();
      this.crossPart = new Vector3D();
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
   public GeneralizedRigidBodyInertia(ReferenceFrame frame, Matrix3D massMomentOfInertia, double mass)
   {
      this.expressedInframe = frame;
      this.massMomentOfInertiaPart = new Matrix3D(massMomentOfInertia);
      this.crossPart = new Vector3D();
      this.mass = mass;
      this.crossPartZero = true;
   }

   public GeneralizedRigidBodyInertia(ReferenceFrame frame, double Ixx, double Iyy, double Izz, double mass)
   {
      this.expressedInframe = frame;
      this.massMomentOfInertiaPart = new Matrix3D();
      setMomentOfInertia(Ixx, Iyy, Izz);
      this.crossPart = new Vector3D();
      this.mass = mass;
      this.crossPartZero = true;
   }

   public GeneralizedRigidBodyInertia(ReferenceFrame frame, Matrix3D massMomentOfInertia, double mass, Vector3D crossPart)
   {
      this.expressedInframe = frame;
      this.massMomentOfInertiaPart = new Matrix3D(massMomentOfInertia);
      this.crossPart = new Vector3D(crossPart);
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
      this.massMomentOfInertiaPart = new Matrix3D(other.massMomentOfInertiaPart);
      this.crossPart = new Vector3D(other.crossPart);
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

   public Matrix3D getMassMomentOfInertiaPartCopy()
   {
      return new Matrix3D(massMomentOfInertiaPart);
   }

   public double getMass()
   {
      return mass;
   }

   public FramePoint getCenterOfMassOffset()
   {
      FramePoint centerOfMassOffset = new FramePoint();
      getCenterOfMassOffset(centerOfMassOffset);
      return centerOfMassOffset;
   }

   public void getCenterOfMassOffset(FramePoint centerOfMassOffsetToPack)
   {
      centerOfMassOffsetToPack.setIncludingFrame(expressedInframe, crossPart);
      centerOfMassOffsetToPack.scale(-1.0 / mass); // comOffset = -1/m * c
   }

   public void setCenterOfMassOffset(FramePoint centerOfMassOffset)
   {
      expressedInframe.checkReferenceFrameMatch(centerOfMassOffset);
      centerOfMassOffset.get(crossPart);
      crossPart.scale(-mass); // c = - m * comOffset
   }

   public void setMomentOfInertia(double Ixx, double Iyy, double Izz)
   {
      massMomentOfInertiaPart.setIdentity();
      massMomentOfInertiaPart.setM00(Ixx);
      massMomentOfInertiaPart.setM11(Iyy);
      massMomentOfInertiaPart.setM22(Izz);
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
   public void getMatrix(DenseMatrix64F matrixToPack)
   {

      // upper left block
      massMomentOfInertiaPart.get(0, 0, matrixToPack);

      // lower left
      tempForCrossPartTilde.setToTildeForm(crossPart);
      tempForCrossPartTilde.get(3, 0, matrixToPack);

      // upper right block
      tempForCrossPartTilde.transpose();
      tempForCrossPartTilde.get(0, 3, matrixToPack);

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
      getMatrix(matrix);
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

      RotationMatrix rotation = new RotationMatrix();
      transform.getRotation(rotation);

      transform.getTranslation(tempVector); // p

      // mass moment of inertia part:

      subTildeTimesTildePlusTildeTimesTildeTransposeFromSymmetricMatrix(massMomentOfInertiaPart, crossPart, tempVector); // J - (tilde(c) * tilde(p))^T - tilde(c) * tilde(p)
      subScalarTimesTildeTimesTildeFromSymmetricMatrix(massMomentOfInertiaPart, tempVector, mass); // J - (tilde(c) * tilde(p))^T - tilde(c) * tilde(p) - m * tilde(p) * tilde(p)

      massMomentOfInertiaPart.set(rotation);
      massMomentOfInertiaPart.multiplyTransposeThis(massMomentOfInertiaPart); // RTranspose * (J - (tilde(c) * tilde(p))^T - tilde(c) * tilde(p) - m * tilde(p) * tilde(p))
      massMomentOfInertiaPart.multiply(rotation); // RTranspose * (J - (tilde(c) * tilde(p))^T - tilde(c) * tilde(p) - m * tilde(p) * tilde(p)) * R: done.

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
   private static void subTildeTimesTildePlusTildeTimesTildeTransposeFromSymmetricMatrix(Matrix3D M, Vector3D a, Vector3D b)
   {
      double axbx = a.getX() * b.getX();
      double ayby = a.getY() * b.getY();
      double azbz = a.getZ() * b.getZ();

      M.setM00(M.getM00() + 2.0 * (azbz + ayby));
      M.setM01(M.getM01() - a.getY() * b.getX() - a.getX() * b.getY());
      M.setM02(M.getM02() - a.getZ() * b.getX() - a.getX() * b.getZ());

      M.setM10(M.getM01());
      M.setM11(M.getM11() + 2.0 * (axbx + azbz));
      M.setM12(M.getM12() - a.getZ() * b.getY() - a.getY() * b.getZ());

      M.setM20(M.getM02());
      M.setM21(M.getM12());
      M.setM22(M.getM22() + 2.0 * (axbx + ayby));
   }

   /**
    * packs M with M - scalar * (tilde(a) * tilde(a))
    *
    * @param M a symmetric matrix - NOTE: no symmetry checks are being performed
    * @param a a vector
    * @param scalar a scalar
    */
   private static void subScalarTimesTildeTimesTildeFromSymmetricMatrix(Matrix3D M, Vector3D a, double scalar)
   {
      double xSquared = scalar * a.getX() * a.getX();
      double ySquared = scalar * a.getY() * a.getY();
      double zSquared = scalar * a.getZ() * a.getZ();

      double xy = scalar * a.getX() * a.getY();
      double xz = scalar * a.getX() * a.getZ();
      double yz = scalar * a.getY() * a.getZ();

      M.setM00(M.getM00() + ySquared + zSquared);
      M.setM01(M.getM01() - xy);
      M.setM02(M.getM02() - xz);

      M.setM10(M.getM01());
      M.setM11(M.getM11() + xSquared + zSquared);
      M.setM12(M.getM12() - yz);

      M.setM20(M.getM02());
      M.setM21(M.getM12());
      M.setM22(M.getM22() + xSquared + ySquared);
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
      getMatrix(mat);

      return mat.toString();
   }

}