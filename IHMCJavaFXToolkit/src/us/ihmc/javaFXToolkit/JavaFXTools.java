package us.ihmc.javaFXToolkit;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;

import javafx.geometry.Point3D;
import javafx.scene.transform.Affine;
import javafx.scene.transform.NonInvertibleTransformException;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public abstract class JavaFXTools
{
   public static void convertRotationMatrixToAffine(Matrix3d rotation, Affine affineToModify)
   {
      affineToModify.setMxx(rotation.getM00());
      affineToModify.setMxy(rotation.getM01());
      affineToModify.setMxz(rotation.getM02());
      affineToModify.setMyx(rotation.getM10());
      affineToModify.setMyy(rotation.getM11());
      affineToModify.setMyz(rotation.getM12());
      affineToModify.setMzx(rotation.getM20());
      affineToModify.setMzy(rotation.getM21());
      affineToModify.setMzz(rotation.getM22());
   }

   public static void convertTransformToRotationMatrix(Transform transform, Matrix3d rotationToPack)
   {
      rotationToPack.setM00(transform.getMxx());
      rotationToPack.setM01(transform.getMxy());
      rotationToPack.setM02(transform.getMxz());
      rotationToPack.setM10(transform.getMyx());
      rotationToPack.setM11(transform.getMyy());
      rotationToPack.setM12(transform.getMyz());
      rotationToPack.setM20(transform.getMzx());
      rotationToPack.setM21(transform.getMzy());
      rotationToPack.setM22(transform.getMzz());
   }

   public static void convertAxisAngleToAffine(AxisAngle4d axisAngle, Affine affineToPack)
   {
      Matrix3d intermediateMatrix = new Matrix3d();
      intermediateMatrix.set(axisAngle);
      convertRotationMatrixToAffine(intermediateMatrix, affineToPack);
   }

   public static Affine convertRigidBodyTransformToAffine(RigidBodyTransform rigidBodyTransform)
   {
      Affine ret = new Affine();
      convertRigidBodyTransformToAffine(rigidBodyTransform, ret);
      return ret;
   }

   public static void convertRigidBodyTransformToAffine(RigidBodyTransform rigidBodyTransform, Affine affineToPack)
   {
      affineToPack.setMxx(rigidBodyTransform.getM00());
      affineToPack.setMxy(rigidBodyTransform.getM01());
      affineToPack.setMxz(rigidBodyTransform.getM02());
      affineToPack.setMyx(rigidBodyTransform.getM10());
      affineToPack.setMyy(rigidBodyTransform.getM11());
      affineToPack.setMyz(rigidBodyTransform.getM12());
      affineToPack.setMzx(rigidBodyTransform.getM20());
      affineToPack.setMzy(rigidBodyTransform.getM21());
      affineToPack.setMzz(rigidBodyTransform.getM22());

      affineToPack.setTx(rigidBodyTransform.getM03());
      affineToPack.setTy(rigidBodyTransform.getM13());
      affineToPack.setTz(rigidBodyTransform.getM23());
   }

   public static Affine createAffineFromQuaternionAndTuple(Quat4d quaternion, Tuple3d translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(quaternion);
      transform.setTranslation(translation.getX(), translation.getY(), translation.getZ());
      return convertRigidBodyTransformToAffine(transform);
   }

   public static Affine createAffineFromQuaternionAndTuple(Quat4f quaternion, Tuple3f translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(quaternion);
      transform.setTranslation(translation.getX(), translation.getY(), translation.getZ());
      return convertRigidBodyTransformToAffine(transform);
   }

   public static Affine createAffineFromAxisAngle(AxisAngle4d axisAngle)
   {
      Affine affine = new Affine();
      convertAxisAngleToAffine(axisAngle, affine);
      return affine;
   }

   public static void applyTranform(Transform transform, Vector3d vectorToTransform)
   {
      Point3D temporaryVector = transform.deltaTransform(vectorToTransform.getX(), vectorToTransform.getY(), vectorToTransform.getZ());
      vectorToTransform.set(temporaryVector.getX(), temporaryVector.getY(), temporaryVector.getZ());
   }

   public static void applyTranform(Transform transform, Point3d pointToTransform)
   {
      Point3D temporaryVector = transform.transform(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ());
      pointToTransform.set(temporaryVector.getX(), temporaryVector.getY(), temporaryVector.getZ());
   }

   public static void applyInvertTranform(Transform transform, Vector3d vectorToTransform)
   {
      Point3D temporaryVector = new Point3D(vectorToTransform.getX(), vectorToTransform.getY(), vectorToTransform.getZ());
      try
      {
         transform.inverseDeltaTransform(temporaryVector);
      }
      catch (NonInvertibleTransformException e)
      {
         e.printStackTrace();
      }
      vectorToTransform.set(temporaryVector.getX(), temporaryVector.getY(), temporaryVector.getZ());
   }

   public static void addEquals(Translate translateToModify, Tuple3d offset)
   {
      translateToModify.setX(translateToModify.getX() + offset.getX());
      translateToModify.setY(translateToModify.getY() + offset.getY());
      translateToModify.setZ(translateToModify.getZ() + offset.getZ());
   }
   
   public static void subEquals(Translate translateToModify, Tuple3d offset)
   {
      translateToModify.setX(translateToModify.getX() - offset.getX());
      translateToModify.setY(translateToModify.getY() - offset.getY());
      translateToModify.setZ(translateToModify.getZ() - offset.getZ());
   }
}
