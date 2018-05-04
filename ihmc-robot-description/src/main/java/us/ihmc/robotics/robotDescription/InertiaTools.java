package us.ihmc.robotics.robotDescription;

import org.ejml.alg.dense.decomposition.svd.SvdImplicitQrDecompose_D64;
import org.ejml.data.DenseMatrix64F;
import org.ejml.interfaces.decomposition.SingularValueDecomposition;
import org.ejml.ops.CommonOps;

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

   public static void computePrincipalMomentsOfInertia(Matrix3D Inertia, RotationMatrix principalAxesRotationToPack, Vector3D principalMomentsOfInertiaToPack)
   {
      DenseMatrix64F inertiaForSVD = new DenseMatrix64F(3, 3);
      Inertia.get(inertiaForSVD);
      computePrincipalMomentsOfInertia(inertiaForSVD, principalAxesRotationToPack, principalMomentsOfInertiaToPack);
   }

   public static void computePrincipalMomentsOfInertia(DenseMatrix64F inertiaForSVD, RotationMatrix principalAxesRotationToPack, Vector3D principalMomentsOfInertiaToPack)
   {
      // Decompose Inertia Matrix:  I = U * W * V
      SingularValueDecomposition<DenseMatrix64F> svd = new SvdImplicitQrDecompose_D64(true, false, true, false);
      svd.decompose(inertiaForSVD);

      DenseMatrix64F W = svd.getW(null);
      //    DenseMatrix64F U = svd.getU(null);
      DenseMatrix64F V = svd.getV(null, false);

      // If determinant is -1.0, then multiply V by -1. Since I = U*sigma*U_Transpose, then I = (-U) * sigma * (-U_Transpose)
      double determinant = CommonOps.det(V);
      if (determinant < 0.0)
      {
         CommonOps.scale(-1.0, V);
         determinant = -determinant;
      }

      if (Math.abs(determinant - 1.0) > 1e-5)
      {
         throw new RuntimeException("Problem in Link.addEllipsoidFromMassProperties(). Determinant should be 1.0");
      }

      principalMomentsOfInertiaToPack.set(W.get(0, 0), W.get(1, 1), W.get(2, 2));
      principalAxesRotationToPack.set(V);
   }

}
