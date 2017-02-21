package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;

import Jama.Matrix;
import Jama.SingularValueDecomposition;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;

public class InertiaTools
{
   /**
    * Returns the radii of an ellipsoid given the inertia parameters, assuming a uniform mass distribution.
    * @param principalMomentsOfInertia principal moments of inertia {Ixx, Iyy, Izz}
    * @param mass mass of the link
    * @return the three radii of the inertia ellipsoid
    */
   public static Vector3D getInertiaEllipsoidRadii(Vector3D principalMomentsOfInertia, double mass)
   {
      double Ixx = principalMomentsOfInertia.getX();
      double Iyy = principalMomentsOfInertia.getY();
      double Izz = principalMomentsOfInertia.getZ();

//    http://en.wikipedia.org/wiki/Ellipsoid#Mass_properties
      Vector3D ret = new Vector3D();
      ret.setX(Math.sqrt(5.0 / 2.0 * (Iyy + Izz - Ixx) / mass));
      ret.setY(Math.sqrt(5.0 / 2.0 * (Izz + Ixx - Iyy) / mass));
      ret.setZ(Math.sqrt(5.0 / 2.0 * (Ixx + Iyy - Izz) / mass));

      return ret;
   }

   public static Matrix3D rotate(RigidBodyTransform inertialFrameRotation, Matrix3D inertia)
   {
      RotationMatrix temp = new RotationMatrix();
      inertialFrameRotation.getRotation(temp);

      return rotate(temp, inertia);
   }

   public static Matrix3D rotate(RotationMatrix inertialFrameRotation, Matrix3D inertia)
   {
      Matrix3D result = new Matrix3D();
      result.set(inertia);
      inertialFrameRotation.transform(result);

      return result;

//
//    inertialFrameRotation.transpose();
//    inertia.mul(inertialFrameRotation);
//    inertialFrameRotation.transpose();
//    inertialFrameRotation.mul(inertia);

   }


   public static void computePrincipalMomentsOfInertia(DenseMatrix64F Inertia, RotationMatrix principalAxesRotationToPack, Vector3D principalMomentsOfInertiaToPack)
   {
      double[][] moiArray = new double[][]{
         {Inertia.get(0, 0), Inertia.get(0, 1), Inertia.get(0, 2)},
         {Inertia.get(1, 0), Inertia.get(1, 1), Inertia.get(1, 2)},
         {Inertia.get(2, 0), Inertia.get(2, 1), Inertia.get(2, 2)}
      };

      Matrix inertiaForSVD = new Matrix(moiArray);
      computePrincipalMomentsOfInertia(inertiaForSVD, principalAxesRotationToPack, principalMomentsOfInertiaToPack);
   }

   public static void computePrincipalMomentsOfInertia(Matrix3D Inertia, RotationMatrix principalAxesRotationToPack, Vector3D principalMomentsOfInertiaToPack)
   {
      double[][] moiArray = new double[][]{
         {Inertia.getM00(), Inertia.getM01(), Inertia.getM02()},
         {Inertia.getM10(), Inertia.getM11(), Inertia.getM12()},
         {Inertia.getM20(), Inertia.getM21(), Inertia.getM22()}
      };

      Matrix inertiaForSVD = new Matrix(moiArray);
      computePrincipalMomentsOfInertia(inertiaForSVD, principalAxesRotationToPack, principalMomentsOfInertiaToPack);
   }

   public static void computePrincipalMomentsOfInertia(Matrix inertiaForSVD, RotationMatrix principalAxesRotationToPack, Vector3D principalMomentsOfInertiaToPack)
   {
      // Decompose Inertia Matrix:  I = U*sigma*V

      SingularValueDecomposition inertiaSVD = new SingularValueDecomposition(inertiaForSVD);

      Matrix sigma = inertiaSVD.getS();

      //    Matrix U = inertiaSVD.getU();
      Matrix V = inertiaSVD.getV();

      // If determinant is -1.0, then multiply V by -1. Since I = U*sigma*U_Transpose, then I = (-U) * sigma * (-U_Transpose)
      double determinant = V.det();
      if (determinant < 0.0)
      {
         V = V.times(-1.0);
         determinant = -determinant;
      }

      if (Math.abs(determinant - 1.0) > 1e-5)
      {
         throw new RuntimeException("Problem in Link.addEllipsoidFromMassProperties(). Determinant should be 1.0");
      }

      principalMomentsOfInertiaToPack.set(sigma.get(0, 0), sigma.get(1, 1), sigma.get(2, 2));
      principalAxesRotationToPack.set(V.getRowPackedCopy());
   }

}
