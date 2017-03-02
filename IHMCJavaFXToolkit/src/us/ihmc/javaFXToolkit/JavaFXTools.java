package us.ihmc.javaFXToolkit;

import javafx.scene.transform.Affine;
import javafx.scene.transform.NonInvertibleTransformException;
import javafx.scene.transform.Transform;
import javafx.scene.transform.Translate;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.Quaternion32;

public abstract class JavaFXTools
{
   public static void convertRotationMatrixToAffine(RotationMatrix rotation, Affine affineToModify)
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

   public static void convertTransformToRotationMatrix(Transform transform, RotationMatrix rotationToPack)
   {
      rotationToPack.set(transform.getMxx(), transform.getMxy(), transform.getMxz(), transform.getMyx(), transform.getMyy(), transform.getMyz(), transform.getMzx(), transform.getMzy(), transform.getMzz());
   }

   public static void convertAxisAngleToAffine(AxisAngle axisAngle, Affine affineToPack)
   {
      RotationMatrix intermediateMatrix = new RotationMatrix();
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

   public static Affine createAffineFromQuaternionAndTuple(Quaternion quaternion, Tuple3DBasics translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(quaternion);
      transform.setTranslation(translation.getX(), translation.getY(), translation.getZ());
      return convertRigidBodyTransformToAffine(transform);
   }

   public static Affine createAffineFromQuaternionAndTuple(Quaternion32 quaternion, Tuple3DBasics translation)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      transform.setRotation(quaternion);
      transform.setTranslation(translation.getX(), translation.getY(), translation.getZ());
      return convertRigidBodyTransformToAffine(transform);
   }

   public static Affine createAffineFromAxisAngle(AxisAngle axisAngle)
   {
      Affine affine = new Affine();
      convertAxisAngleToAffine(axisAngle, affine);
      return affine;
   }

   public static void applyTranform(Transform transform, Vector3D vectorToTransform)
   {
      javafx.geometry.Point3D temporaryVector = transform.deltaTransform(vectorToTransform.getX(), vectorToTransform.getY(), vectorToTransform.getZ());
      vectorToTransform.set(temporaryVector.getX(), temporaryVector.getY(), temporaryVector.getZ());
   }

   public static void applyTranform(Transform transform, Point3D pointToTransform)
   {
      javafx.geometry.Point3D temporaryVector = transform.transform(pointToTransform.getX(), pointToTransform.getY(), pointToTransform.getZ());
      pointToTransform.set(temporaryVector.getX(), temporaryVector.getY(), temporaryVector.getZ());
   }

   public static void applyInvertTranform(Transform transform, Vector3D vectorToTransform)
   {
      javafx.geometry.Point3D temporaryVector = new javafx.geometry.Point3D(vectorToTransform.getX(), vectorToTransform.getY(), vectorToTransform.getZ());
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

   public static void addEquals(Translate translateToModify, Tuple3DBasics offset)
   {
      translateToModify.setX(translateToModify.getX() + offset.getX());
      translateToModify.setY(translateToModify.getY() + offset.getY());
      translateToModify.setZ(translateToModify.getZ() + offset.getZ());
   }
   
   public static void subEquals(Translate translateToModify, Tuple3DBasics offset)
   {
      translateToModify.setX(translateToModify.getX() - offset.getX());
      translateToModify.setY(translateToModify.getY() - offset.getY());
      translateToModify.setZ(translateToModify.getZ() - offset.getZ());
   }
}
