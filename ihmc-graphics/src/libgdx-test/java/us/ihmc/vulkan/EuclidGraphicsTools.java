package us.ihmc.vulkan;

import org.ejml.data.DMatrix4x4;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.nio.ByteBuffer;

public class EuclidGraphicsTools
{
   public static void setM11(RigidBodyTransform rigidBodyTransform, double m11)
   {
      rigidBodyTransform.getRotation().setRotationMatrix(rigidBodyTransform.getM00(),
                                                         rigidBodyTransform.getM01(),
                                                         rigidBodyTransform.getM02(),
                                                         rigidBodyTransform.getM10(),
                                                         m11,
                                                         rigidBodyTransform.getM12(),
                                                         rigidBodyTransform.getM20(),
                                                         rigidBodyTransform.getM21(),
                                                         rigidBodyTransform.getM22());
   }

   /**
    * Taken from https://github.dev/JOML-CI/JOML/blob/main/src/main/java/org/joml/Matrix4f.java
    */
   public static void lookAt(RigidBodyTransform rigidBodyTransformToPack,
                             Point3DReadOnly eyePosition,
                             Point3DReadOnly pointToLookAt,
                             Vector3DReadOnly upNormal)
   {
      // Compute direction from position to lookAt
      double dirX, dirY, dirZ;
      dirX = eyePosition.getX() - pointToLookAt.getX();
      dirY = eyePosition.getY() - pointToLookAt.getY();
      dirZ = eyePosition.getZ() - pointToLookAt.getZ();
      // Normalize direction
      double invDirLength = 1.0 / EuclidCoreTools.squareRoot(dirX * dirX + dirY * dirY + dirZ * dirZ);
      dirX *= invDirLength;
      dirY *= invDirLength;
      dirZ *= invDirLength;
      // left = up x direction
      double leftX, leftY, leftZ;
      leftX = upNormal.getY() * dirZ - upNormal.getZ() * dirY;
      leftY = upNormal.getZ() * dirX - upNormal.getX() * dirZ;
      leftZ = upNormal.getX() * dirY - upNormal.getY() * dirX;
      // normalize left
      double invLeftLength = 1.0 / EuclidCoreTools.squareRoot(leftX * leftX + leftY * leftY + leftZ * leftZ);
      leftX *= invLeftLength;
      leftY *= invLeftLength;
      leftZ *= invLeftLength;
      // up = direction x left
      double upnX = dirY * leftZ - dirZ * leftY;
      double upnY = dirZ * leftX - dirX * leftZ;
      double upnZ = dirX * leftY - dirY * leftX;

      // calculate right matrix elements
      double rm30 = -(leftX * eyePosition.getX() + leftY * eyePosition.getY() + leftZ * eyePosition.getZ());
      double rm31 = -(upnX *  eyePosition.getX() + upnY *  eyePosition.getY() + upnZ *  eyePosition.getZ());
      double rm32 = -(dirX *  eyePosition.getX() + dirY *  eyePosition.getY() + dirZ *  eyePosition.getZ());
      // introduce temporaries for dependent results
      double nm00 = rigidBodyTransformToPack.getM00() * leftX + rigidBodyTransformToPack.getM10() * upnX + rigidBodyTransformToPack.getM20() * dirX;
      double nm01 = rigidBodyTransformToPack.getM01() * leftX + rigidBodyTransformToPack.getM11() * upnX + rigidBodyTransformToPack.getM21() * dirX;
      double nm02 = rigidBodyTransformToPack.getM02() * leftX + rigidBodyTransformToPack.getM12() * upnX + rigidBodyTransformToPack.getM22() * dirX;
      double nm03 = rigidBodyTransformToPack.getM03() * leftX + rigidBodyTransformToPack.getM13() * upnX + rigidBodyTransformToPack.getM23() * dirX;
      double nm10 = rigidBodyTransformToPack.getM00() * leftY + rigidBodyTransformToPack.getM10() * upnY + rigidBodyTransformToPack.getM20() * dirY;
      double nm11 = rigidBodyTransformToPack.getM01() * leftY + rigidBodyTransformToPack.getM11() * upnY + rigidBodyTransformToPack.getM21() * dirY;
      double nm12 = rigidBodyTransformToPack.getM02() * leftY + rigidBodyTransformToPack.getM12() * upnY + rigidBodyTransformToPack.getM22() * dirY;
      double nm13 = rigidBodyTransformToPack.getM03() * leftY + rigidBodyTransformToPack.getM13() * upnY + rigidBodyTransformToPack.getM23() * dirY;

      // perform optimized matrix multiplication
      // compute last column first, because others do not depend on it
      double m30 = rigidBodyTransformToPack.getM00() * rm30 +  rigidBodyTransformToPack.getM10() * rm31 + rigidBodyTransformToPack.getM20() * rm32 + rigidBodyTransformToPack.getM30();
      double m31 = rigidBodyTransformToPack.getM01() * rm30 +  rigidBodyTransformToPack.getM11() * rm31 + rigidBodyTransformToPack.getM21() * rm32 + rigidBodyTransformToPack.getM31();
      double m32 = rigidBodyTransformToPack.getM02() * rm30 +  rigidBodyTransformToPack.getM12() * rm31 + rigidBodyTransformToPack.getM22() * rm32 + rigidBodyTransformToPack.getM32();
      double m33 = rigidBodyTransformToPack.getM03() * rm30 +  rigidBodyTransformToPack.getM13() * rm31 + rigidBodyTransformToPack.getM23() * rm32 + rigidBodyTransformToPack.getM33();
      double m20 = rigidBodyTransformToPack.getM00() * leftZ + rigidBodyTransformToPack.getM10() * upnZ + rigidBodyTransformToPack.getM20() * dirZ;
      double m21 = rigidBodyTransformToPack.getM01() * leftZ + rigidBodyTransformToPack.getM11() * upnZ + rigidBodyTransformToPack.getM21() * dirZ;
      double m22 = rigidBodyTransformToPack.getM02() * leftZ + rigidBodyTransformToPack.getM12() * upnZ + rigidBodyTransformToPack.getM22() * dirZ;
      double m23 = rigidBodyTransformToPack.getM03() * leftZ + rigidBodyTransformToPack.getM13() * upnZ + rigidBodyTransformToPack.getM23() * dirZ;
      double m00 = nm00;
      double m01 = nm01;
      double m02 = nm02;
      double m03 = nm03;
      double m10 = nm10;
      double m11 = nm11;
      double m12 = nm12;
      double m13 = nm13;

      rigidBodyTransformToPack.set(m00, m01, m02, m03, m10, m11, m12, m13, m20, m21, m22, m23);
   }

   public static void perspective(DMatrix4x4 projectionMatrixToPack, double fieldOfViewY, double aspectRatio, double zNear, double zFar)
   {
      projectionMatrixToPack.zero();

      double h = Math.tan(fieldOfViewY * 0.5f);
      projectionMatrixToPack.a11 = 1.0 / h * aspectRatio;
      projectionMatrixToPack.a22 = 1.0 / h;

      boolean farIsInfinity = zFar > 0.0 && Double.isInfinite(zFar);
      boolean nearIsInfinity = zNear > 0.0 && Double.isInfinite(zNear);

      if (farIsInfinity)
      {
         // See: "Infinite Projection Matrix" (http://www.terathon.com/gdc07_lengyel.pdf)
         double e = 1e-6f;
         projectionMatrixToPack.a33 = e - 1.0;
         projectionMatrixToPack.a43 = e - 2.0 * zNear;
      }
      else if (nearIsInfinity)
      {
         float e = 1e-6f;
         projectionMatrixToPack.a33 = 1.0;
         projectionMatrixToPack.a43 = (2.0 - e) * zFar;
      }
      else
      {
         projectionMatrixToPack.a33 = (zFar + zNear) / (zNear - zFar);
         projectionMatrixToPack.a43 = (zFar + zFar) * zNear / (zNear - zFar);
      }
      projectionMatrixToPack.a34 = -1.0;
   }

   public static void packValuesIntoBuffer(ByteBuffer byteBuffer, RigidBodyTransform rigidBodyTransform)
   {
      byteBuffer.putFloat((float) rigidBodyTransform.getM00());
      byteBuffer.putFloat((float) rigidBodyTransform.getM01());
      byteBuffer.putFloat((float) rigidBodyTransform.getM02());
      byteBuffer.putFloat((float) rigidBodyTransform.getM03());
      byteBuffer.putFloat((float) rigidBodyTransform.getM10());
      byteBuffer.putFloat((float) rigidBodyTransform.getM11());
      byteBuffer.putFloat((float) rigidBodyTransform.getM12());
      byteBuffer.putFloat((float) rigidBodyTransform.getM13());
      byteBuffer.putFloat((float) rigidBodyTransform.getM20());
      byteBuffer.putFloat((float) rigidBodyTransform.getM21());
      byteBuffer.putFloat((float) rigidBodyTransform.getM22());
      byteBuffer.putFloat((float) rigidBodyTransform.getM23());
      byteBuffer.putFloat((float) rigidBodyTransform.getM30());
      byteBuffer.putFloat((float) rigidBodyTransform.getM31());
      byteBuffer.putFloat((float) rigidBodyTransform.getM32());
      byteBuffer.putFloat((float) rigidBodyTransform.getM33());
   }

   public static void packValuesIntoBuffer(ByteBuffer byteBuffer, DMatrix4x4 dMatrix4x4)
   {
      byteBuffer.putFloat((float) dMatrix4x4.a11);
      byteBuffer.putFloat((float) dMatrix4x4.a12);
      byteBuffer.putFloat((float) dMatrix4x4.a13);
      byteBuffer.putFloat((float) dMatrix4x4.a14);
      byteBuffer.putFloat((float) dMatrix4x4.a21);
      byteBuffer.putFloat((float) dMatrix4x4.a22);
      byteBuffer.putFloat((float) dMatrix4x4.a23);
      byteBuffer.putFloat((float) dMatrix4x4.a24);
      byteBuffer.putFloat((float) dMatrix4x4.a31);
      byteBuffer.putFloat((float) dMatrix4x4.a32);
      byteBuffer.putFloat((float) dMatrix4x4.a33);
      byteBuffer.putFloat((float) dMatrix4x4.a34);
      byteBuffer.putFloat((float) dMatrix4x4.a41);
      byteBuffer.putFloat((float) dMatrix4x4.a42);
      byteBuffer.putFloat((float) dMatrix4x4.a43);
      byteBuffer.putFloat((float) dMatrix4x4.a44);
   }
}
