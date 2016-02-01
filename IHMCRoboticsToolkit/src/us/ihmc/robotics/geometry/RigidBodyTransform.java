package us.ihmc.robotics.geometry;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.random.RandomTools;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4d;
import javax.vecmath.Matrix4f;
import javax.vecmath.Point3d;
import javax.vecmath.Point3f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;
import javax.vecmath.Vector4d;
import javax.vecmath.Vector4f;
import java.io.ByteArrayOutputStream;
import java.io.PrintStream;
import java.io.Serializable;
import java.util.Random;

/**
 * 
 * This class creates a 4x4 affine, rigid body transformation matrix. The top
 * left 3x3 is an orthogonal rotation matrix, while the top right 3x1 is a vector
 * describing a translation. 
 * 
 * T = | xx yx zx px | 
 *     | xy yy zy py |  
 *     | xz yz zz pz |  
 *     | 0 0 0 1 |
 */

public class RigidBodyTransform implements Serializable
{
   private static final long serialVersionUID = 1915106568805908193L;
   
   public double mat00 = 1.0;
   public double mat01 = 0.0;
   public double mat02 = 0.0;
   public double mat03 = 0.0;
   public double mat10 = 0.0;
   public double mat11 = 1.0;
   public double mat13 = 0.0;
   public double mat12 = 0.0;
   public double mat20 = 0.0;
   public double mat21 = 0.0;
   public double mat22 = 1.0;
   public double mat23 = 0.0;

   /**
    * Set to identity
    */
   public RigidBodyTransform()
   {
      setIdentity();
   }

   /**
    * Set this transform equal to the RigidBodyTransformed sent as an argument.
    * @param transform
    */
   public RigidBodyTransform(RigidBodyTransform transform)
   {
      set(transform);
   }

   /**
    * Create transformation matrix from Matrix4d
    * 
    * @param mat4d
    */
   public RigidBodyTransform(Matrix4d matrix)
   {
      set(matrix);
   }

   /**
    * Create transformation matrix from Matrix4f
    * 
    * @param mat4d
    */
   public RigidBodyTransform(Matrix4f matrix)
   {
      set(matrix);
   }

   /**
    * Create transform from 1D array of doubles.
    * 
    * @param doubleArray
    */
   public RigidBodyTransform(double[] doubleArray)
   {
      set(doubleArray);
   }

   /**
    * Create transform from 1D array of floats.
    * 
    * @param floatArray
    */
   public RigidBodyTransform(float[] floatArray)
   {
      set(floatArray);
   }

   /**
    * Create transform from 4x4 DenseMatrix64F
    * 
    * @param matrix
    */
   public RigidBodyTransform(DenseMatrix64F matrix)
   {
      set(matrix);
   }

   /**
    * Create 4x4 RigidBodyTransform from rotation matrix of type DenseMatrix64F and
    * translational vector of type Vector3d
    * 
    * @param matrix
    * @param vector
    */
   public RigidBodyTransform(DenseMatrix64F matrix, Vector3d vector)
   {
      if (matrix.numRows != 3 && matrix.numCols != 3)
      {
         throw new RuntimeException("Incorrectly sized matrix. Must be 3x3");
      }
      set(matrix, vector);
   }

   /**
    * Create transformation matrix from rotation matrix and vector translation
    * 
    * @param matrix
    * @param vector
    */
   public RigidBodyTransform(Matrix3d matrix, Vector3d vector)
   {
      set(matrix, vector);
   }

   /**
    * Create transformation matrix from rotation matrix and vector translation
    * 
    * @param matrix
    * @param vector
    */
   public RigidBodyTransform(Matrix3f matrix, Vector3f vector)
   {
      set(matrix, vector);
   }

   /**
    * Create RigidBodyTransform from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    */
   public RigidBodyTransform(Quat4d quat, Vector3d vector)
   {
      set(quat, vector);
   }

   /**
    * Create RigidBodyTransform from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    */
   public RigidBodyTransform(Quat4f quat, Vector3f vector)
   {
      set(quat, vector);
   }

   /**
    * Create RigidBodyTransform from AxisAngle4d and Vector3d
    * 
    * @param axisAngle
    * @param vector
    */
   public RigidBodyTransform(AxisAngle4d axisAngle, Vector3d vector)
   {
      set(axisAngle, vector);
   }

   /**
    * Create RigidBodyTransform from AxisAngle4d and Vector3d
    * 
    * @param axisAngle
    * @param vector
    */
   public RigidBodyTransform(AxisAngle4f axisAngle, Vector3f vector)
   {
      set(axisAngle, vector);
   }

   public static RigidBodyTransform generateRandomTransform(Random random)
   {
      RigidBodyTransform ret = new RigidBodyTransform();
      ret.setRotationAndZeroTranslation(RandomTools.generateRandomRotation(random));
      ret.setTranslation(RandomTools.generateRandomVector(random));

      return ret;
   }

   /**
    * Convert AxisAngle representation to rotation matrix and store as 
    * rotational component of this transform.
    * 
    * @param axisAngle
    */
   public void setRotation(AxisAngle4d axisAngle)
   {
      setRotationWithAxisAngle(axisAngle.x, axisAngle.y, axisAngle.z, axisAngle.angle);
   }

   /**
    * Convert AxisAngle representation to rotation matrix and store as 
    * rotational component of this transform.
    * 
    * @param axisAngle
    */
   public void setRotation(AxisAngle4f axisAngle)
   {
      setRotationWithAxisAngle(axisAngle.x, axisAngle.y, axisAngle.z, axisAngle.angle);
   }

   private void setRotationWithAxisAngle(double axisAngleX, double axisAngleY, double axisAngleZ, double axisAngleTheta)
   {
      double mag = Math.sqrt(axisAngleX * axisAngleX + axisAngleY * axisAngleY + axisAngleZ * axisAngleZ);

      if (almostZero(mag))
      {
         setIdentity();
      }
      else
      {
         mag = 1.0 / mag;
         double ax = axisAngleX * mag;
         double ay = axisAngleY * mag;
         double az = axisAngleZ * mag;

         double sinTheta = Math.sin(axisAngleTheta);
         double cosTheta = Math.cos(axisAngleTheta);
         double t = 1.0 - cosTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = (t * ax * ax + cosTheta);
         mat01 = (t * xy - sinTheta * az);
         mat02 = (t * xz + sinTheta * ay);

         mat10 = (t * xy + sinTheta * az);
         mat11 = (t * ay * ay + cosTheta);
         mat12 = (t * yz - sinTheta * ax);

         mat20 = (t * xz - sinTheta * ay);
         mat21 = (t * yz + sinTheta * ax);
         mat22 = (t * az * az + cosTheta);
      }
   }

   /**
    * Convert quaternion to rotation matrix and store as rotational 
    * component of this transform.
    * 
    * @param quat
    */
   public void setRotation(Quat4d quat)
   {
      setRotationWithQuaternion(quat.x, quat.y, quat.z, quat.w);
   }

   /**
    * Convert quaternion to rotation matrix and store as rotational 
    * component of this transform.
    * 
    * @param quat
    */
   public void setRotation(Quat4f quat)
   {
      setRotationWithQuaternion(quat.x, quat.y, quat.z, quat.w);
   }
   
   public void setRotationWithQuaternion(double qx, double qy, double qz, double qw)
   {
      double yy2 = 2.0 * qy * qy;
      double zz2 = 2.0 * qz * qz;
      double xx2 = 2.0 * qx * qx;
      double xy2 = 2.0 * qx * qy;
      double wz2 = 2.0 * qw * qz;
      double xz2 = 2.0 * qx * qz;
      double wy2 = 2.0 * qw * qy;
      double yz2 = 2.0 * qy * qz;
      double wx2 = 2.0 * qw * qx;

      mat00 = (1.0 - yy2 - zz2);
      mat01 = (xy2 - wz2);
      mat02 = (xz2 + wy2);
      mat10 = (xy2 + wz2);
      mat11 = (1.0 - xx2 - zz2);
      mat12 = (yz2 - wx2);
      mat20 = (xz2 - wy2);
      mat21 = (yz2 + wx2);
      mat22 = (1.0 - xx2 - yy2);
   }

   /**
    * Set the 3x3 rotation matrix equal to mat3d.
    * 
    * @param matrix
    */
   public void setRotation(Matrix3d matrix)
   {
      mat00 = matrix.m00;
      mat01 = matrix.m01;
      mat02 = matrix.m02;
      mat10 = matrix.m10;
      mat11 = matrix.m11;
      mat12 = matrix.m12;
      mat20 = matrix.m20;
      mat21 = matrix.m21;
      mat22 = matrix.m22;
   }

   /**
    * Set the 3x3 rotation matrix equal to mat3f.
    * 
    * @param mat3d
    */
   public void setRotation(Matrix3f matrix)
   {
      mat00 = matrix.m00;
      mat01 = matrix.m01;
      mat02 = matrix.m02;
      mat10 = matrix.m10;
      mat11 = matrix.m11;
      mat12 = matrix.m12;
      mat20 = matrix.m20;
      mat21 = matrix.m21;
      mat22 = matrix.m22;
   }

   /**
    * Sets rotation portion equal to the rotation matrix described in the
    * parameter matrix.
    * 
    * @param matrix
    */
   public void setRotation(DenseMatrix64F matrix)
   {
      if (matrix.numRows != 3 && matrix.numCols != 3)
      {
         throw new RuntimeException("Improperly sized matrix. Must be 3x3.");
      }
      mat00 = matrix.get(0, 0);
      mat01 = matrix.get(0, 1);
      mat02 = matrix.get(0, 2);
      mat10 = matrix.get(1, 0);
      mat11 = matrix.get(1, 1);
      mat12 = matrix.get(1, 2);
      mat20 = matrix.get(2, 0);
      mat21 = matrix.get(2, 1);
      mat22 = matrix.get(2, 2);
   }

   /**
    * Set translational portion of the transformation matrix
    * 
    * @param vector
    */
   public final void setTranslation(Vector3d vector)
   {
      setTranslation(vector.x, vector.y, vector.z);
   }

   public void setTranslation(double x, double y, double z)
   {
      mat03 = x;
      mat13 = y;
      mat23 = z;
   }
   
   /**
   *  Add a translation to the current transform. It is equivalent to:
   *  
   *      transform.setTranslationAndIdentityRotation(translation);
   *      this = this*transform
   */
   public final void applyTranslation(Vector3d translation)
   {
      Point3d temp = new Point3d(translation);
      transform(temp);
      mat03 = temp.x;
      mat13 = temp.y;
      mat23 = temp.z;
   }
   
   /**
   *  Add a rotation to the current transform. 
   */
   public final void applyRotationX(double angle)
   {
      RigidBodyTransform temp = new RigidBodyTransform();
      temp.rotX(angle);    
      multiply(temp);
   }
   
   /**
   *  Add a rotation to the current transform. 
   */
   public final void applyRotationY(double angle)
   {
      RigidBodyTransform temp = new RigidBodyTransform();
      temp.rotY(angle);   
      multiply(temp);
   }
   
   /**
   *  Add a rotation to the current transform. 
   */
   public final void applyRotationZ(double angle)
   {
      RigidBodyTransform temp = new RigidBodyTransform();
      temp.rotZ(angle);    
      multiply(temp);
   }
   
   /**
    * Set translational portion of the transformation matrix
    * 
    * @param vec3d
    */
   public final void setTranslation(Vector3f vector)
   {
      setTranslation(vector.x, vector.y, vector.z);
   }

   /**
    * Set elements of this transform equal to the elements of transform.
    * 
    * @param transform
    */
   public final void set(RigidBodyTransform transform)
   {
      this.mat00 = transform.mat00;
      this.mat01 = transform.mat01;
      this.mat02 = transform.mat02;
      this.mat03 = transform.mat03;
      this.mat10 = transform.mat10;
      this.mat11 = transform.mat11;
      this.mat12 = transform.mat12;
      this.mat13 = transform.mat13;
      this.mat20 = transform.mat20;
      this.mat21 = transform.mat21;
      this.mat22 = transform.mat22;
      this.mat23 = transform.mat23;
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3d matrix.
    * 
    * @param matrix
    */
   public final void setRotationAndZeroTranslation(Matrix3d matrix)
   {
      setRotation(matrix);
      setTranslation(0, 0, 0);
   }

   /**
    * Set this transform to have translation described in vector 
    * and a rotation equal to the Matrix3d matrix.
    * 
    * @param matrix
    */
   public final void set(Matrix3d matrix, Vector3d vector)
   {
      setRotation(matrix);
      setTranslation(vector.x, vector.y, vector.z);
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Quat4d quat.
    * 
    * @param quat
    */
   public final void setRotationAndZeroTranslation(Quat4d quat)
   {
      setRotation(quat);
      setTranslation(0, 0, 0);
   }

   /**
    * Set this transform to have translation described in vector and a rotation
    * equal to the Quat4d quat.
    * 
    * @param quat
    */
   public final void set(Quat4d quat, Vector3d vector)
   {
      setRotation(quat);
      setTranslation(vector);
   }

   /**
    * Sets this transform to have rotation described by axisAngle and zero
    * translation.
    * 
    * @param axisAngle
    */
   public final void setRotationAndZeroTranslation(AxisAngle4d axisAngle)
   {
      setRotation(axisAngle);
      setTranslation(0, 0, 0);
   }

   /**
    * Sets this transform to have rotation described by axisAngle and 
    * translation described in the Vector3d argument vector.
    * 
    * @param axisAngle
    */
   public final void set(AxisAngle4d axisAngle, Vector3d vector)
   {
      setRotation(axisAngle);
      setTranslation(vector.x, vector.y, vector.z);
   }

   /**
    * Sets this transform to have rotation described by axisAngle and zero
    * translation.
    * 
    * @param axisAngle
    */
   public final void setRotationAndZeroTranslation(AxisAngle4f axisAngle)
   {
      setRotation(axisAngle);
      setTranslation(0, 0, 0);
   }

   /**
    * Sets this transform to have rotation described by axisAngle and 
    * translation described by the Vector3f vector.
    * 
    * @param axisAngle
    */
   public final void set(AxisAngle4f axisAngle, Vector3f vector)
   {
      setRotation(axisAngle);
      setTranslation(vector.x, vector.y, vector.z);
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Quat4f quat.
    * 
    * @param quat
    */
   public final void setRotationAndZeroTranslation(Quat4f quat)
   {
      setRotation(quat);
      setTranslation(0, 0, 0);
   }

   /**
    * Set this transform to have zero translation
    */
   public final void zeroTranslation()
   {
      setTranslation(0, 0, 0);
   }

   /**
    * Set this transform to have translation described in vector and a rotation
    * equal to the Quat4f quat.
    * 
    * @param quat
    */
   public final void set(Quat4f quat, Vector3f vector)
   {
      setRotation(quat);
      setTranslation(vector);
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3f matrix.
    * 
    * @param matrix
    */
   public final void setRotationAndZeroTranslation(Matrix3f matrix)
   {
      setRotation(matrix);
      setTranslation(0, 0, 0);
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3f matrix.
    * 
    * @param matrix
    */
   public final void set(Matrix3f matrix, Vector3f vector)
   {
      setRotation(matrix);
      setTranslation(vector);
   }

   /**
    * Set this transform to have an identity rotation and a translation given
    * by the Vector3d vector.
    * 
    * @param vector
    */
   public final void setTranslationAndIdentityRotation(Vector3d vector)
   {
      setTranslation(vector.x, vector.y, vector.z);
      mat00 = 1.0;
      mat01 = 0.0;
      mat02 = 0.0;
      mat10 = 0.0;
      mat11 = 1.0;
      mat12 = 0.0;
      mat20 = 0.0;
      mat21 = 0.0;
      mat22 = 1.0;
   }

   /**
    * Sets rotation to the identity, does not effect the translational component of the Transform
    * 
    * @param vector
    */
   public final void setRotationToIdentity()
   {
      mat00 = 1.0;
      mat01 = 0.0;
      mat02 = 0.0;
      mat10 = 0.0;
      mat11 = 1.0;
      mat12 = 0.0;
      mat20 = 0.0;
      mat21 = 0.0;
      mat22 = 1.0;
   }

   /**
    * Set this transform to have an identity rotation and a translation given
    * by the Vector3d vector.
    * 
    * @param vector
    */
   public final void setTranslationAndIdentityRotation(Vector3f vector)
   {
      setTranslation(vector.x, vector.y, vector.z);
      mat00 = 1.0;
      mat01 = 0.0;
      mat02 = 0.0;
      mat10 = 0.0;
      mat11 = 1.0;
      mat12 = 0.0;
      mat20 = 0.0;
      mat21 = 0.0;
      mat22 = 1.0;
   }

   /**
    * Set elements of this transform equal to the elements of matrix.
    * 
    * @param matrix
    */
   public final void set(DenseMatrix64F matrix)
   {
      if (matrix.numCols != 4 && matrix.numRows != 4)
      {
         throw new RuntimeException("Incorrectly sized matrix. Matrix must be 4x4");
      }

      mat00 = matrix.get(0, 0);
      mat01 = matrix.get(0, 1);
      mat02 = matrix.get(0, 2);
      mat03 = matrix.get(0, 3);
      mat10 = matrix.get(1, 0);
      mat11 = matrix.get(1, 1);
      mat12 = matrix.get(1, 2);
      mat13 = matrix.get(1, 3);
      mat20 = matrix.get(2, 0);
      mat21 = matrix.get(2, 1);
      mat22 = matrix.get(2, 2);
      mat23 = matrix.get(2, 3);
   }

   /**
    * Set elements of this transform equal to the elements of matrix.
    * 
    * @param matrix
    */
   public final void set(DenseMatrix64F matrix, Vector3d vector)
   {
      if (matrix.numCols != 3 && matrix.numRows != 3)
      {
         throw new RuntimeException("Incorrectly sized matrix. Matrix must be 4x4");
      }

      setRotation(matrix);
      setTranslation(vector.x, vector.y, vector.z);
   }

   /**
    * Sets the elements of this transform to the elements of the transform in
    * doubleArray.
    * 
    * @param doubleArray
    */
   public final void set(double[] doubleArray)
   {
      mat00 = doubleArray[0];
      mat01 = doubleArray[1];
      mat02 = doubleArray[2];
      mat03 = doubleArray[3];
      mat10 = doubleArray[4];
      mat11 = doubleArray[5];
      mat12 = doubleArray[6];
      mat13 = doubleArray[7];
      mat20 = doubleArray[8];
      mat21 = doubleArray[9];
      mat22 = doubleArray[10];
      mat23 = doubleArray[11];
   }

   /**
    * Sets the elements of this transform to the elements of the transform in
    * floatArray.
    * 
    * @param floatArray
    */
   public final void set(float[] floatArray)
   {
      mat00 = floatArray[0];
      mat01 = floatArray[1];
      mat02 = floatArray[2];
      mat03 = floatArray[3];
      mat10 = floatArray[4];
      mat11 = floatArray[5];
      mat12 = floatArray[6];
      mat13 = floatArray[7];
      mat20 = floatArray[8];
      mat21 = floatArray[9];
      mat22 = floatArray[10];
      mat23 = floatArray[11];
   }

   /**
    * Sets the elements of this tranform equal to that of the 
    * transpose of floatArray. This is useful for setting a 
    * transform from a column-major floatArray describing a 
    * transform.
    * 
    * @param floatArray
    */
   public final void setAsTranspose(float[] floatArray)
   {
      float tmp10 = floatArray[4];
      float tmp20 = floatArray[8];
      float tmp21 = floatArray[9];
      float tmp30 = floatArray[12];
      float tmp31 = floatArray[13];
      float tmp32 = floatArray[14];

      mat00 = floatArray[0];
      mat11 = floatArray[5];
      mat22 = floatArray[10];
      mat10 = floatArray[1];
      mat20 = floatArray[2];
      mat21 = floatArray[6];
      mat01 = tmp10;
      mat02 = tmp20;
      mat12 = tmp21;
      mat03 = tmp30;
      mat13 = tmp31;
      mat23 = tmp32;

   }

   /**
    * Set elements of transform equal to elements of the Matrix4d.
    * 
    * @param matrix
    */
   public final void set(Matrix4d matrix)
   {
      mat00 = matrix.m00;
      mat01 = matrix.m01;
      mat02 = matrix.m02;
      mat03 = matrix.m03;
      mat10 = matrix.m10;
      mat11 = matrix.m11;
      mat12 = matrix.m12;
      mat13 = matrix.m13;
      mat20 = matrix.m20;
      mat21 = matrix.m21;
      mat22 = matrix.m22;
      mat23 = matrix.m23;
   }

   /**
    * This method is for when the Matrix4d matrix is column major and needs to
    * be transposed.
    * 
    * @param matrix
    */
   public void setAsTranspose(Matrix4d matrix)
   {
      double tmp10 = matrix.m10;
      double tmp20 = matrix.m20;
      double tmp21 = matrix.m21;
      double tmp30 = matrix.m30;
      double tmp31 = matrix.m31;
      double tmp32 = matrix.m32;

      mat00 = matrix.m00;
      mat11 = matrix.m11;
      mat22 = matrix.m22;
      mat10 = matrix.m01;
      mat20 = matrix.m02;
      mat21 = matrix.m12;
      mat01 = tmp10;
      mat03 = tmp30;
      mat13 = tmp31;
      mat23 = tmp32;
      mat02 = tmp20;
      mat12 = tmp21;
   }

   /**
    * This method is for when the Matrix4d matrix is column major and needs to
    * be transposed.
    * 
    * @param matrix
    */
   public void setAsTranspose(Matrix4f matrix)
   {
      double tmp10 = matrix.m10;
      double tmp20 = matrix.m20;
      double tmp21 = matrix.m21;
      double tmp30 = matrix.m30;
      double tmp31 = matrix.m31;
      double tmp32 = matrix.m32;

      mat00 = matrix.m00;
      mat11 = matrix.m11;
      mat22 = matrix.m22;
      mat10 = matrix.m01;
      mat20 = matrix.m02;
      mat21 = matrix.m12;
      mat01 = tmp10;
      mat03 = tmp30;
      mat13 = tmp31;
      mat23 = tmp32;
      mat02 = tmp20;
      mat12 = tmp21;
   }

   /**
    * Set elements of transform equal to elements of the Matrix4d matrix.
    * 
    * @param mat4d
    */
   public final void set(Matrix4f matrix)
   {
      mat00 = matrix.m00;
      mat01 = matrix.m01;
      mat02 = matrix.m02;
      mat03 = matrix.m03;
      mat10 = matrix.m10;
      mat11 = matrix.m11;
      mat12 = matrix.m12;
      mat13 = matrix.m13;
      mat20 = matrix.m20;
      mat21 = matrix.m21;
      mat22 = matrix.m22;
      mat23 = matrix.m23;
   }

   /**
    * Set transformation matrix to Identity, meaning no rotation or
    * translation.
    */
   public final void setIdentity()
   {
      mat00 = 1.0;
      mat01 = 0.0;
      mat02 = 0.0;
      mat03 = 0.0;
      mat10 = 0.0;
      mat11 = 1.0;
      mat12 = 0.0;
      mat13 = 0.0;
      mat20 = 0.0;
      mat21 = 0.0;
      mat22 = 1.0;
      mat23 = 0.0;
   }

   /**
    * Set the rotational component of the transform to the rotation matrix
    * created given an X-Y-Z rotation described by the angles in vector which
    * describe angles of rotation about the X, Y, and Z axis, respectively. The
    * orientation of each rotation is not effected by any of the other
    * rotations. This method sets the translational component of this
    * transform3d to zeros.
    * 
    * @param vector
    */
   public final void setEuler(Vector3d vector)
   {
      setEuler(vector.x, vector.y, vector.z);
   }

   /**
    * Set the rotational component of the transform to the rotation matrix
    * created given an X-Y-Z rotation described by the angles in vector which
    * describe angles of rotation about the X, Y, and Z axis, respectively. The
    * orientation of each rotation is not effected by any of the other
    * rotations. This method sets the translational component of this
    * transform3d to zeros.
    * 
    * @param rotX
    * @param rotY
    * @param rotZ
    */
   public final void setEuler(double rotX, double rotY, double rotZ)
   {
      double sina = Math.sin(rotX);
      double sinb = Math.sin(rotY);
      double sinc = Math.sin(rotZ);
      double cosa = Math.cos(rotX);
      double cosb = Math.cos(rotY);
      double cosc = Math.cos(rotZ);

      mat00 = cosb * cosc;
      mat01 = -(cosa * sinc) + (sina * sinb * cosc);
      mat02 = (sina * sinc) + (cosa * sinb * cosc);
      mat10 = cosb * sinc;
      mat11 = (cosa * cosc) + (sina * sinb * sinc);
      mat12 = -(sina * cosc) + (cosa * sinb * sinc);
      mat20 = -sinb;
      mat21 = sina * cosb;
      mat22 = cosa * cosb;
      mat03 = 0.0;
      mat13 = 0.0;
      mat23 = 0.0;
   }

   /**
    * Computes the RPY angles from the rotation matrix for rotations about the
    * X, Y, and Z axes respectively. Note that this method is here for the
    * purpose of unit testing the method setEuler. This particular solution is
    * only valid for -pi/2 < vector.y < pi/2 and for vector.y != 0.
    * 
    * @param vector
    */
   public void getEulerXYZ(Vector3d vector)
   {
      vector.x = Math.atan2(mat21, mat22);
      vector.y = Math.atan2(-mat20, Math.sqrt(mat21 * mat21 + mat22 * mat22));
      vector.z = Math.atan2(mat10, mat00);
   }

   /**
    * Return rotation matrix of type Matrix3d
    * 
    * @param matrix
    */
   public void getRotation(Matrix3d matrix)
   {
      matrix.m00 = mat00;
      matrix.m01 = mat01;
      matrix.m02 = mat02;
      matrix.m10 = mat10;
      matrix.m11 = mat11;
      matrix.m12 = mat12;
      matrix.m20 = mat20;
      matrix.m21 = mat21;
      matrix.m22 = mat22;
   }

   /**
    * Return rotation matrix of type Matrix3f
    * 
    * @param matrix
    */
   public void getRotation(Matrix3f matrix)
   {
      matrix.m00 = (float) mat00;
      matrix.m01 = (float) mat01;
      matrix.m02 = (float) mat02;
      matrix.m10 = (float) mat10;
      matrix.m11 = (float) mat11;
      matrix.m12 = (float) mat12;
      matrix.m20 = (float) mat20;
      matrix.m21 = (float) mat21;
      matrix.m22 = (float) mat22;
   }

   /**
    * Return rotation matrix of type DenseMatrix64F
    * 
    * @param matrix
    */
   public void getRotation(DenseMatrix64F matrix)
   {
      matrix.set(0, 0, mat00);
      matrix.set(0, 1, mat01);
      matrix.set(0, 2, mat02);
      matrix.set(1, 0, mat10);
      matrix.set(1, 1, mat11);
      matrix.set(1, 2, mat12);
      matrix.set(2, 0, mat20);
      matrix.set(2, 1, mat21);
      matrix.set(2, 2, mat22);
   }

   /**
    * Return rotation in quaternion form.
    * 
    * @param quat
    */
   public void getRotation(Quat4d quat)
   {
      double trace = mat00 + mat11 + mat22;
      double val;

      if (trace > 0.0)
      {
         val = Math.sqrt(trace + 1.0) * 2.0;
         quat.x = (mat21 - mat12) / val;
         quat.y = (mat02 - mat20) / val;
         quat.z = (mat10 - mat01) / val;
         quat.w = 0.25 * val;
      }
      else if (mat11 > mat22)
      {
         double temp = Math.max(0.0, 1.0 + mat11 - mat00 - mat22);
         val = Math.sqrt(temp) * 2.0;
         quat.x = (mat01 + mat10) / val;
         quat.y = 0.25 * val;
         quat.z = (mat12 + mat21) / val;
         quat.w = (mat02 - mat20) / val;
      }
      else if ((mat00 > mat11) && (mat00 > mat22))
      {
         val = Math.sqrt(1.0 + mat00 - mat11 - mat22) * 2.0;
         quat.x = 0.25 * val;
         quat.y = (mat01 + mat10) / val;
         quat.z = (mat02 + mat20) / val;
         quat.w = (mat21 - mat12) / val;
      }
      else
      {
         val = Math.sqrt(1.0 + mat22 - mat00 - mat11) * 2.0;
         quat.x = (mat02 + mat20) / val;
         quat.y = (mat12 + mat21) / val;
         quat.z = 0.25 * val;
         quat.w = (mat10 - mat01) / val;
      }
      /* Other implementation. Already tested.
       * 
      double q0,q1,q2,q3;
      q0 = ( mat00 + mat11 + mat22 + 1.0) / 4.0;
      q1 = ( mat00 - mat11 - mat22 + 1.0) / 4.0;
      q2 = (-mat00 + mat11 - mat22 + 1.0) / 4.0;
      q3 = (-mat00 - mat11 + mat22 + 1.0) / 4.0;
      
      if(q0 < 0.0) q0 = 0.0;
      if(q1 < 0.0) q1 = 0.0;
      if(q2 < 0.0) q2 = 0.0;
      if(q3 < 0.0) q3 = 0.0;
      q0 = Math.sqrt(q0);
      q1 = Math.sqrt(q1);
      q2 = Math.sqrt(q2);
      q3 = Math.sqrt(q3);
      if(q0 >= q1 && q0 >= q2 && q0 >= q3) {
          q0 *= +1.0;
          q1 *= Math.signum(mat21 - mat12);
          q2 *= Math.signum(mat02 - mat20);
          q3 *= Math.signum(mat10 - mat01);
      } else if(q1 >= q0 && q1 >= q2 && q1 >= q3) {
          q0 *= Math.signum(mat21 - mat12);
          q1 *= +1.0;
          q2 *= Math.signum(mat10 + mat01);
          q3 *= Math.signum(mat02 + mat20);
      } else if(q2 >= q0 && q2 >= q1 && q2 >= q3) {
          q0 *= Math.signum(mat02 - mat20);
          q1 *= Math.signum(mat10 + mat01);
          q2 *= +1.0;
          q3 *= Math.signum(mat21 + mat12);
      } else if(q3 >= q0 && q3 >= q1 && q3 >= q2) {
          q0 *= Math.signum(mat10 - mat01);
          q1 *= Math.signum(mat20 + mat02);
          q2 *= Math.signum(mat21 + mat12);
          q3 *= +1.0;
      } else {
          System.out.println("coding error\n");
      }
      quat.w = q0;
      quat.x = q1;
      quat.y = q2;
      quat.z = q3;
      
      quat.normalize();*/
   }

   /**
    * Return rotation in quaternion form.
    * 
    * @param quat
    */
   public void getRotation(Quat4f quat)
   {
      double trace = mat00 + mat11 + mat22;
      double val;
      if (trace > 0.0)
      {
         val = Math.sqrt(trace + 1.0) * 2.0;
         quat.x = (float) ((mat21 - mat12) / val);
         quat.y = (float) ((mat02 - mat20) / val);
         quat.z = (float) ((mat10 - mat01) / val);
         quat.w = (float) (0.25 * val);
      }
      else if (mat11 > mat22)
      {
         val = Math.sqrt(1.0 + mat11 - mat00 - mat22) * 2.0;
         quat.x = (float) ((mat01 + mat10) / val);
         quat.y = (float) (0.25 * val);
         quat.z = (float) ((mat12 + mat21) / val);
         quat.w = (float) ((mat02 - mat20) / val);
      }
      else if ((mat00 > mat11) && (mat00 > mat22))
      {
         val = Math.sqrt(1.0 + mat00 - mat11 - mat22) * 2.0;
         quat.x = (float) (0.25 * val);
         quat.y = (float) ((mat01 + mat10) / val);
         quat.z = (float) ((mat02 + mat20) / val);
         quat.w = (float) ((mat21 - mat12) / val);
      }
      else
      {
         val = Math.sqrt(1.0 + mat22 - mat00 - mat11) * 2.0;
         quat.x = (float) ((mat02 + mat20) / val);
         quat.y = (float) ((mat12 + mat21) / val);
         quat.z = (float) (0.25 * val);
         quat.w = (float) ((mat10 - mat01) / val);
      }
   }

   /**
    * Return rotation in AxisAngle form.
    * 
    * @param axisAngle
    */
   public void getRotation(AxisAngle4d axisAngle)
   {
      getRotation(axisAngle, 1.0e-12);
   }

   public void getRotation(AxisAngle4d axisAngle, double epsilon)
   {
      axisAngle.x = mat21 - mat12;
      axisAngle.y = mat02 - mat20;
      axisAngle.z = mat10 - mat01;
      double mag = axisAngle.x * axisAngle.x + axisAngle.y * axisAngle.y + axisAngle.z * axisAngle.z;

      if (mag > epsilon)
      {
         mag = Math.sqrt(mag);
         double sin = 0.5 * mag;
         double cos = 0.5 * (mat00 + mat11 + mat22 - 1.0);

         axisAngle.angle = Math.atan2(sin, cos);

         double invMag = 1.0 / mag;
         axisAngle.x = axisAngle.x * invMag;
         axisAngle.y = axisAngle.y * invMag;
         axisAngle.z = axisAngle.z * invMag;
      }
      else
      {
         if (isRotationMatrixEpsilonIdentity(10.0 * epsilon))
         {
            axisAngle.set(0.0, 1.0, 0.0, 0.0);
            return;
         }
         else
         {
            axisAngle.angle = Math.PI;

            double xx = (mat00 + 1.0) / 2.0;
            double yy = (mat11 + 1.0) / 2.0;
            double zz = (mat22 + 1.0) / 2.0;
            double xy = (mat01 + mat10) / 4.0;
            double xz = (mat02 + mat20) / 4.0;
            double yz = (mat12 + mat21) / 4.0;
            double cos45 = Math.cos(Math.PI / 4.0);

            if ((xx > yy) && (xx > zz))
            { // mat00 is the largest diagonal term
               if (xx < epsilon)
               {
                  axisAngle.x = 0.0;
                  axisAngle.y = cos45;
                  axisAngle.z = cos45;
               }
               else
               {
                  axisAngle.x = Math.sqrt(xx);
                  axisAngle.y = xy / axisAngle.x;
                  axisAngle.z = xz / axisAngle.x;
               }
            }
            else if (yy > zz)
            { // mat11 is the largest diagonal term
               if (yy < epsilon)
               {
                  axisAngle.x = cos45;
                  axisAngle.y = 0.0;
                  axisAngle.z = cos45;
               }
               else
               {
                  axisAngle.y = Math.sqrt(yy);
                  axisAngle.x = xy / axisAngle.y;
                  axisAngle.z = yz / axisAngle.y;
               }
            }
            else
            { // mat22 is the largest diagonal term
               if (zz < epsilon)
               {
                  axisAngle.x = cos45;
                  axisAngle.y = cos45;
                  axisAngle.z = 0.0;
               }
               else
               {
                  axisAngle.z = Math.sqrt(zz);
                  axisAngle.x = xz / axisAngle.z;
                  axisAngle.y = yz / axisAngle.z;
               }
            }
         }
      }
   }
   
   public boolean isRotationMatrixSingular(double epsilon)
   {
      return (Math.abs(mat01 - mat10) < epsilon) && (Math.abs(mat02 - mat20) < epsilon) && (Math.abs(mat12 - mat21) < epsilon);
   }
   
   public boolean isRotationMatrixEpsilonIdentity(double epsilon)
   {     
      return (Math.abs(mat01 + mat10) < epsilon) && (Math.abs(mat02 + mat20) < epsilon) && (Math.abs(mat12 + mat21) < epsilon)
            && (Math.abs(mat00 + mat11 + mat22 - 3) < epsilon);
   }

   /**
    * Return rotation in AxisAngle form.
    * 
    * @param axisAngle
    */
   public void getRotation(AxisAngle4f axisAngle)
   {
      axisAngle.x = (float) (mat21 - mat12);
      axisAngle.y = (float) (mat02 - mat20);
      axisAngle.z = (float) (mat10 - mat01);
      double mag = axisAngle.x * axisAngle.x + axisAngle.y * axisAngle.y + axisAngle.z * axisAngle.z;

      if (mag > 1.0e-12)
      {
         mag = Math.sqrt(mag);
         double sin = 0.5 * mag;
         double cos = 0.5 * (mat00 + mat11 + mat22 - 1.0);

         axisAngle.angle = (float) Math.atan2(sin, cos);

         double invMag = 1.0 / mag;
         axisAngle.x = (float) (axisAngle.x * invMag);
         axisAngle.y = (float) (axisAngle.y * invMag);
         axisAngle.z = (float) (axisAngle.z * invMag);
      }
      else
      {
         axisAngle.x = (float) 0.0;
         axisAngle.y = (float) 1.0;
         axisAngle.z = (float) 0.0;
         axisAngle.angle = (float) 0.0;
      }
   }

   /**
    * Return translational part as Vector3d
    * 
    * @param vector
    */
   public final void getTranslation(Vector3d vector)
   {
      vector.x = mat03;
      vector.y = mat13;
      vector.z = mat23;
   }

   /**
    * Return translational part as Vector3f
    * 
    * @param vector
    */
   public final void getTranslation(Vector3f vector)
   {
      vector.x = (float) mat03;
      vector.y = (float) mat13;
      vector.z = (float) mat23;
   }
   
   /**
    * Return translational part as Point3d
    * 
    * @param point
    */
   public final void getTranslation(Point3d point)
   {
      point.x = mat03;
      point.y = mat13;
      point.z = mat23;
   }

   /**
    * Return RigidBodyTransform as array of doubles
    * 
    * @param ret
    */
   public final void get(double[] ret)
   {
      ret[0] = mat00;
      ret[1] = mat01;
      ret[2] = mat02;
      ret[3] = mat03;
      ret[4] = mat10;
      ret[5] = mat11;
      ret[6] = mat12;
      ret[7] = mat13;
      ret[8] = mat20;
      ret[9] = mat21;
      ret[10] = mat22;
      ret[11] = mat23;
      ret[12] = 0.0;
      ret[13] = 0.0;
      ret[14] = 0.0;
      ret[15] = 1.0;
   }

   /**
    * Return RigidBodyTransform as array of floats
    * 
    * @param ret
    */
   public final void get(float[] ret)
   {
      ret[0] = (float) mat00;
      ret[1] = (float) mat01;
      ret[2] = (float) mat02;
      ret[3] = (float) mat03;
      ret[4] = (float) mat10;
      ret[5] = (float) mat11;
      ret[6] = (float) mat12;
      ret[7] = (float) mat13;
      ret[8] = (float) mat20;
      ret[9] = (float) mat21;
      ret[10] = (float) mat22;
      ret[11] = (float) mat23;
      ret[12] = (float) 0.0;
      ret[13] = (float) 0.0;
      ret[14] = (float) 0.0;
      ret[15] = (float) 1.0;
   }

   /**
    * Return Transform as Matrix4d type.
    * 
    * @param ret
    */
   public final void get(Matrix4d ret)
   {
      ret.m00 = mat00;
      ret.m01 = mat01;
      ret.m02 = mat02;
      ret.m03 = mat03;
      ret.m10 = mat10;
      ret.m11 = mat11;
      ret.m12 = mat12;
      ret.m13 = mat13;
      ret.m20 = mat20;
      ret.m21 = mat21;
      ret.m22 = mat22;
      ret.m23 = mat23;
      ret.m30 = 0.0;
      ret.m31 = 0.0;
      ret.m32 = 0.0;
      ret.m33 = 1.0;
   }

   /**
    * Pack transform into Matrix4f
    * 
    * @param ret
    */
   public final void get(Matrix4f ret)
   {
      ret.m00 = (float) mat00;
      ret.m01 = (float) mat01;
      ret.m02 = (float) mat02;
      ret.m03 = (float) mat03;
      ret.m10 = (float) mat10;
      ret.m11 = (float) mat11;
      ret.m12 = (float) mat12;
      ret.m13 = (float) mat13;
      ret.m20 = (float) mat20;
      ret.m21 = (float) mat21;
      ret.m22 = (float) mat22;
      ret.m23 = (float) mat23;
      ret.m30 = 0.0f;
      ret.m31 = 0.0f;
      ret.m32 = 0.0f;
      ret.m33 = 1.0f;
   }

   /**
    * Pack transform into DenseMatrix64F
    * 
    * @param ret
    */
   public final void get(DenseMatrix64F ret)
   {
      if (ret.numCols != 4 && ret.numRows != 4)
      {
         throw new RuntimeException("Matrix has incorrect size. Must be 4x4.");
      }

      ret.set(0, 0, mat00);
      ret.set(0, 1, mat01);
      ret.set(0, 2, mat02);
      ret.set(0, 3, mat03);
      ret.set(1, 0, mat10);
      ret.set(1, 1, mat11);
      ret.set(1, 2, mat12);
      ret.set(1, 3, mat13);
      ret.set(2, 0, mat20);
      ret.set(2, 1, mat21);
      ret.set(2, 2, mat22);
      ret.set(2, 3, mat23);
      ret.set(3, 0, 0.0);
      ret.set(3, 1, 0.0);
      ret.set(3, 2, 0.0);
      ret.set(3, 3, 1.0);
   }

   /**
    * Pack rotation part into Matrix3d and translation part into Vector3d
    * 
    * @param matrix
    * @param vector
    */
   public void get(Matrix3d matrix, Vector3d vector)
   {
      getRotation(matrix);
      getTranslation(vector);
   }

   /**
    * Return rotation portion of this transform.
    * 
    * @param matrix
    */
   public void get(Matrix3d matrix)
   {
      getRotation(matrix);
   }

   /**
    * Return rotation portion of this transform.
    * 
    * @param matrix
    */
   public void get(Matrix3f matrix)
   {
      getRotation(matrix);
   }

   /**
    * Return translational portion of this transform.
    * 
    * @param vector
    */
   public final void get(Vector3d vector)
   {
      getTranslation(vector);
   }

   /**
    * Return translational portion of this transform.
    * 
    * @param vector
    */
   public final void get(Vector3f vector)
   {
      getTranslation(vector);
   }

   /**
    * Pack rotation part into Matrix3f and translation part into Vector3f
    * 
    * @param matrix
    * @param vector
    */
   public void get(Matrix3f matrix, Vector3f vector)
   {
      getRotation(matrix);
      getTranslation(vector);
   }

   /**
    * Convert and pack rotation part of transform into Quat4d and pack
    * translation into Vector3d.
    * 
    * @param quat
    * @param vector
    */
   public void get(Quat4d quat, Vector3d vector)
   {
      getRotation(quat);
      getTranslation(vector);
   }
   
   /**
    * Convert and pack rotation part of transform into Quat4d and pack
    * translation into Point3d.
    * 
    * @param quat
    * @param point
    */
   public void get(Quat4d quat, Point3d point)
   {
      getRotation(quat);
      getTranslation(point);
   }

   /**
    * Convert and pack rotation part of transform into Quat4d.
    * 
    * @param quat
    * @param vector
    */
   public void get(Quat4d quat)
   {
      getRotation(quat);
   }

   /**
    * Convert and pack rotation part of transform into Quat4f.
    * 
    * @param quat
    * @param vector
    */
   public void get(Quat4f quat)
   {
      getRotation(quat);
   }

   /**
    * Convert and pack rotation part of transform into Quat4f and pack
    * translation into Vector3f.
    * 
    * @param quat
    * @param vector
    */
   public void get(Quat4f quat, Vector3f vector)
   {
      getRotation(quat);
      getTranslation(vector);
   }

   /**
    * Multiplies this RigidBodyTransform by transform and stores the result in this,
    * i.e. this = this*transform
    * 
    * @param transform
    */
   public final void multiply(RigidBodyTransform transform)
   {
      multiply(this, transform);
   }

   /**
    * Multiplies transform1 and transform2 and puts result into this. this =
    * transform1*transform2
    * 
    * @param transform1
    * @param transform2
    */
   public final void multiply(RigidBodyTransform transform1, RigidBodyTransform transform2)
   {
      double tmp00 = transform1.mat00 * transform2.mat00 + transform1.mat01 * transform2.mat10 + transform1.mat02 * transform2.mat20;
      double tmp01 = transform1.mat00 * transform2.mat01 + transform1.mat01 * transform2.mat11 + transform1.mat02 * transform2.mat21;
      double tmp02 = transform1.mat00 * transform2.mat02 + transform1.mat01 * transform2.mat12 + transform1.mat02 * transform2.mat22;
      double tmp03 = transform1.mat00 * transform2.mat03 + transform1.mat01 * transform2.mat13 + transform1.mat02 * transform2.mat23 + transform1.mat03;

      double tmp10 = transform1.mat10 * transform2.mat00 + transform1.mat11 * transform2.mat10 + transform1.mat12 * transform2.mat20;
      double tmp11 = transform1.mat10 * transform2.mat01 + transform1.mat11 * transform2.mat11 + transform1.mat12 * transform2.mat21;
      double tmp12 = transform1.mat10 * transform2.mat02 + transform1.mat11 * transform2.mat12 + transform1.mat12 * transform2.mat22;
      double tmp13 = transform1.mat10 * transform2.mat03 + transform1.mat11 * transform2.mat13 + transform1.mat12 * transform2.mat23 + transform1.mat13;

      double tmp20 = transform1.mat20 * transform2.mat00 + transform1.mat21 * transform2.mat10 + transform1.mat22 * transform2.mat20;
      double tmp21 = transform1.mat20 * transform2.mat01 + transform1.mat21 * transform2.mat11 + transform1.mat22 * transform2.mat21;
      double tmp22 = transform1.mat20 * transform2.mat02 + transform1.mat21 * transform2.mat12 + transform1.mat22 * transform2.mat22;
      double tmp23 = transform1.mat20 * transform2.mat03 + transform1.mat21 * transform2.mat13 + transform1.mat22 * transform2.mat23 + transform1.mat23;

      mat00 = tmp00;
      mat01 = tmp01;
      mat02 = tmp02;
      mat03 = tmp03;
      mat10 = tmp10;
      mat11 = tmp11;
      mat12 = tmp12;
      mat13 = tmp13;
      mat20 = tmp20;
      mat21 = tmp21;
      mat22 = tmp22;
      mat23 = tmp23;
   }

   /**
    * Compute the inverse of the RigidBodyTransform passed in as an 
    * argument exploiting the orthogonality of the rotation matrix 
    * and store the result in this.
    * @param transform
    */
   public void invert(RigidBodyTransform transform)
   {
      if (transform != this)
      {
         set(transform);
      }
      invert();
   }

   public void invert()
   {
      invertOrthogonal();
   }

   /**
    * Invert this assuming an orthogonal rotation portion.
    */
   private final void invertOrthogonal()
   {
      double tmp01 = mat01;
      double tmp02 = mat02;
      double tmp12 = mat12;

      // For orthogonal matrix, R^{-1} = R^{T}
      mat01 = mat10;
      mat02 = mat20;
      mat12 = mat21;
      mat10 = tmp01;
      mat20 = tmp02;
      mat21 = tmp12;

      // New translation vector becomes -R^{T} * p
      double newTransX = -(mat23 * mat02 + mat00 * mat03 + mat01 * mat13);
      double newTransY = -(mat03 * mat10 + mat23 * mat12 + mat11 * mat13);
      mat23 = -(mat22 * mat23 + mat03 * mat20 + mat13 * mat21);
      mat03 = newTransX;
      mat13 = newTransY;
   }
   
   public final void invertRotationButKeepTranslation()
   {
      double tmp01 = mat01;
      double tmp02 = mat02;
      double tmp12 = mat12;

      // For orthogonal matrix, R^{-1} = R^{T}
      mat01 = mat10;
      mat02 = mat20;
      mat12 = mat21;
      mat10 = tmp01;
      mat20 = tmp02;
      mat21 = tmp12;
   }

   /**
    * Create RigidBodyTransform with zero translation and the rotation matrix being a
    * rotation about the x-axis by angle.
    * 
    * @param angle
    */
   public void rotX(double angle)
   {
      double cosAngle = Math.cos(angle);
      double sinAngle = Math.sin(angle);

      mat00 = 1.0;
      mat01 = 0.0;
      mat02 = 0.0;
      mat03 = 0.0;
      mat10 = 0.0;
      mat11 = cosAngle;
      mat12 = -sinAngle;
      mat13 = 0.0;
      mat20 = 0.0;
      mat21 = sinAngle;
      mat22 = cosAngle;
      mat23 = 0.0;
   }

   /**
    * Create RigidBodyTransform with zero translation and the rotation matrix being a
    * rotation about the y-axis by angle.
    * 
    * @param angle
    */
   public void rotY(double angle)
   {
      double cosAngle = Math.cos(angle);
      double sinAngle = Math.sin(angle);

      mat00 = cosAngle;
      mat01 = 0.0;
      mat02 = sinAngle;
      mat03 = 0.0;
      mat10 = 0.0;
      mat11 = 1.0;
      mat12 = 0.0;
      mat13 = 0.0;
      mat20 = -sinAngle;
      mat21 = 0.0;
      mat22 = cosAngle;
      mat23 = 0.0;
   }

   /**
    * Create RigidBodyTransform with zero translation and the rotation matrix being a
    * rotation about the z-axis by angle.
    * 
    * @param angle
    */
   public void rotZ(double angle)
   {
      double cosAngle = Math.cos(angle);
      double sinAngle = Math.sin(angle);

      mat00 = cosAngle;
      mat01 = -sinAngle;
      mat02 = 0.0;
      mat03 = 0.0;
      mat10 = sinAngle;
      mat11 = cosAngle;
      mat12 = 0.0;
      mat13 = 0.0;
      mat20 = 0.0;
      mat21 = 0.0;
      mat22 = 1.0;
      mat23 = 0.0;
   }

   /**
    * Check if the elements of this are within epsilon of the elements of
    * transform.
    * 
    * @param transform
    * @param epsilon
    * @return
    */
   public final boolean epsilonEquals(RigidBodyTransform transform, double epsilon)
   {
      if (!MathTools.epsilonEquals(mat00, transform.mat00, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat01, transform.mat01, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat02, transform.mat02, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat03, transform.mat03, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat10, transform.mat10, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat11, transform.mat11, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat12, transform.mat12, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat13, transform.mat13, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat20, transform.mat20, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat21, transform.mat21, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat22, transform.mat22, epsilon)) return false;
      if (!MathTools.epsilonEquals(mat23, transform.mat23, epsilon)) return false;
      return true;
   }

   /**
    * Returns true if each element of this is equal to each element of
    * transform within a default tolerance of 1e-10.
    * 
    * @param transform
    * @return
    */
   public final boolean equals(RigidBodyTransform transform)
   {
      return epsilonEquals(transform, 1e-10);
   }

   /**
    * Returns true if the Object o1 is of type Transform3D and all of the data
    * members of o1 are equal to the corresponding data members in this
    * Transform3D.
    * 
    * @param o1 the object with which the comparison is made.
    *            
    * @return true or false
    */
   @Override
   public boolean equals(Object object1)
   {
      return (object1 instanceof RigidBodyTransform) && equals((RigidBodyTransform) object1);
   }

   /**
    * Transform vector by multiplying it by this transform and put result back
    * into vector.
    * 
    * @param vector
    */
   public final void transform(Vector4d vector)
   {
      if (vector.w != 1.0)
      {
         throw new RuntimeException("Final element of vector must be 1.");
      }
      double x = mat00 * vector.x + mat01 * vector.y + mat02 * vector.z + mat03;
      double y = mat10 * vector.x + mat11 * vector.y + mat12 * vector.z + mat13;
      vector.z = mat20 * vector.x + mat21 * vector.y + mat22 * vector.z + mat23;
      vector.x = x;
      vector.y = y;
      vector.w = 1.0;
   }

   /**
    * Transform vector by multiplying it by this transform and put result back
    * into vector.
    * 
    * @param vector
    */
   public final void transform(Vector3d vector)
   {
      double x = mat00 * vector.x + mat01 * vector.y + mat02 * vector.z;
      double y = mat10 * vector.x + mat11 * vector.y + mat12 * vector.z;
      vector.z = mat20 * vector.x + mat21 * vector.y + mat22 * vector.z;

      vector.x = x;
      vector.y = y;
   }

   /**
    * Transform vector by multiplying it by this transform and put result back
    * into vector.
    * 
    * @param vector
    */
   public final void transform(Vector3d vectorIn, Vector3d vectorOut)
   {
      if (vectorIn != vectorOut)
      {
         vectorOut.x = mat00 * vectorIn.x + mat01 * vectorIn.y + mat02 * vectorIn.z;
         vectorOut.y = mat10 * vectorIn.x + mat11 * vectorIn.y + mat12 * vectorIn.z;
         vectorOut.z = mat20 * vectorIn.x + mat21 * vectorIn.y + mat22 * vectorIn.z;
      }
      else
      {
         transform(vectorIn);
      }
   }

   /**
    * Transform vector by multiplying it by this transform and put result back
    * into vector.
    * 
    * @param vector
    */
   public final void transform(Vector4f vector)
   {
      if (vector.w != 1.0)
      {
         throw new RuntimeException("Final element of vector must be 1.");
      }

      double x = mat00 * vector.x + mat01 * vector.y + mat02 * vector.z + mat03;
      double y = mat10 * vector.x + mat11 * vector.y + mat12 * vector.z + mat13;
      double z = mat20 * vector.x + mat21 * vector.y + mat22 * vector.z + mat23;

      vector.x = (float) x;
      vector.y = (float) y;
      vector.z = (float) z;
      vector.w = 1.0f;
   }

   /**
    * Transform vector by multiplying it by this transform and put result back
    * into vector.
    * 
    * @param vector
    */
   public final void transform(Vector3f vector)
   {
      double x = mat00 * vector.x + mat01 * vector.y + mat02 * vector.z;
      double y = mat10 * vector.x + mat11 * vector.y + mat12 * vector.z;
      vector.z = (float) (mat20 * vector.x + mat21 * vector.y + mat22 * vector.z);

      vector.x = (float) x;
      vector.y = (float) y;
   }

   /**
    * Transform vector by multiplying it by this transform and put result back
    * into vector.
    * 
    * @param vector
    */
   public final void transform(Vector3f vectorIn, Vector3f vectorOut)
   {
      if (vectorIn != vectorOut)
      {
         vectorOut.x = (float) (mat00 * vectorIn.x + mat01 * vectorIn.y + mat02 * vectorIn.z);
         vectorOut.y = (float) (mat10 * vectorIn.x + mat11 * vectorIn.y + mat12 * vectorIn.z);
         vectorOut.z = (float) (mat20 * vectorIn.x + mat21 * vectorIn.y + mat22 * vectorIn.z);
      }
      else
      {
         transform(vectorIn);
      }
   }

   /**
    * Transform vectorIn using this transform and store result in vectorOut.
    * 
    * @param vectorIn
    * @param vectorOut
    */
   public final void transform(Vector4d vectorIn, Vector4d vectorOut)
   {
      if (vectorIn != vectorOut)
      {
         vectorOut.x = mat00 * vectorIn.x + mat01 * vectorIn.y + mat02 * vectorIn.z + mat03;
         vectorOut.y = mat10 * vectorIn.x + mat11 * vectorIn.y + mat12 * vectorIn.z + mat13;
         vectorOut.z = mat20 * vectorIn.x + mat21 * vectorIn.y + mat22 * vectorIn.z + mat23;
         vectorOut.w = 1.0;
      }
      else
      {
         transform(vectorIn);
      }
   }

   /**
    * Transform vectorIn using this transform and store result in vectorOut.
    * 
    * @param vectorIn
    * @param vectorOut
    */
   public final void transform(Vector4f vectorIn, Vector4f vectorOut)
   {
      if (vectorIn != vectorOut)
      {
         vectorOut.x = (float) (mat00 * vectorIn.x + mat01 * vectorIn.y + mat02 * vectorIn.z + mat03);
         vectorOut.y = (float) (mat10 * vectorIn.x + mat11 * vectorIn.y + mat12 * vectorIn.z + mat13);
         vectorOut.z = (float) (mat20 * vectorIn.x + mat21 * vectorIn.y + mat22 * vectorIn.z + mat23);
         vectorOut.w = 1.0f;
      }
      else
      {
         transform(vectorIn);
      }
   }

   /**
    * Transform the Point3d point by this transform and place result back in
    * point.
    * 
    * @param point
    */
   public final void transform(Point3d point)
   {
      double x = mat00 * point.x + mat01 * point.y + mat02 * point.z + mat03;
      double y = mat10 * point.x + mat11 * point.y + mat12 * point.z + mat13;
      point.z = mat20 * point.x + mat21 * point.y + mat22 * point.z + mat23;

      point.x = x;
      point.y = y;
   }

   /**
    * Transform the Point3d pointIn by this transform and place result in
    * pointOut.
    * 
    * @param point
    */
   public final void transform(Point3d pointIn, Point3d pointOut)
   {
      if (pointIn != pointOut)
      {
         pointOut.x = mat00 * pointIn.x + mat01 * pointIn.y + mat02 * pointIn.z + mat03;
         pointOut.y = mat10 * pointIn.x + mat11 * pointIn.y + mat12 * pointIn.z + mat13;
         pointOut.z = mat20 * pointIn.x + mat21 * pointIn.y + mat22 * pointIn.z + mat23;
      }
      else
      {
         transform(pointIn);
      }
   }

   /**
    * Transform the Point3f point by this transform and place result back in
    * point.
    * 
    * @param point
    */
   public final void transform(Point3f point)
   {
      float x = (float) (mat00 * point.x + mat01 * point.y + mat02 * point.z + mat03);
      float y = (float) (mat10 * point.x + mat11 * point.y + mat12 * point.z + mat13);
      point.z = (float) (mat20 * point.x + mat21 * point.y + mat22 * point.z + mat23);

      point.x = x;
      point.y = y;
   }

   /**
    * Transform the Point3f pointIn by this transform and place result in
    * pointOut.
    * 
    * @param point
    */
   public final void transform(Point3f pointIn, Point3f pointOut)
   {
      if (pointIn != pointOut)
      {
         pointOut.x = (float) (mat00 * pointIn.x + mat01 * pointIn.y + mat02 * pointIn.z + mat03);
         pointOut.y = (float) (mat10 * pointIn.x + mat11 * pointIn.y + mat12 * pointIn.z + mat13);
         pointOut.z = (float) (mat20 * pointIn.x + mat21 * pointIn.y + mat22 * pointIn.z + mat23);
      }
      else
      {
         transform(pointIn);
      }
   }

   /**
    * Return the determinant of this transform.
    * 
    * @return
    */
   public final double determinant()
   {
      return (mat00 * (mat11 * mat22 - mat12 * mat21) - mat01 * (mat10 * mat22 - mat12 * mat20) + mat02 * (mat10 * mat21 - mat11 * mat20));
   }

   /**
    * Orthonormalization of the rotation matrix using Gram-Schmidt method.
    */
   public void normalize()
   {
      double xdoty = mat00 * mat01 + mat10 * mat11 + mat20 * mat21;
      double xdotx = mat00 * mat00 + mat10 * mat10 + mat20 * mat20;
      double tmp = xdoty / xdotx;

      mat01 -= tmp * mat00;
      mat11 -= tmp * mat10;
      mat21 -= tmp * mat20;

      double zdoty = mat02 * mat01 + mat12 * mat11 + mat22 * mat21;
      double zdotx = mat02 * mat00 + mat12 * mat10 + mat22 * mat20;
      double ydoty = mat01 * mat01 + mat11 * mat11 + mat21 * mat21;

      tmp = zdotx / xdotx;
      double tmp1 = zdoty / ydoty;

      mat02 = mat02 - (tmp * mat00 + tmp1 * mat01);
      mat12 = mat12 - (tmp * mat10 + tmp1 * mat11);
      mat22 = mat22 - (tmp * mat20 + tmp1 * mat21);

      // Compute orthogonalized vector magnitudes and normalize
      double magX = Math.sqrt(mat00 * mat00 + mat10 * mat10 + mat20 * mat20);
      double magY = Math.sqrt(mat01 * mat01 + mat11 * mat11 + mat21 * mat21);
      double magZ = Math.sqrt(mat02 * mat02 + mat12 * mat12 + mat22 * mat22);

      mat00 = mat00 / magX;
      mat10 = mat10 / magX;
      mat20 = mat20 / magX;
      mat01 = mat01 / magY;
      mat11 = mat11 / magY;
      mat21 = mat21 / magY;
      mat02 = mat02 / magZ;
      mat12 = mat12 / magZ;
      mat22 = mat22 / magZ;
   }

   private static final boolean almostZero(double a)
   {
      return ((a < 1.0e-5) && (a > -1.0e-5));
   }

   static final ByteArrayOutputStream stream = new ByteArrayOutputStream();
   static final PrintStream out = new PrintStream(stream);
   
   /**
    * Returns the matrix elements of this transform as a string.
    * 
    * @return the matrix elements of this transform
    */  
   @Override
   synchronized public String toString()
   {     
      return toString(8);
   }
   
   synchronized public String toString(int decimals)
   {     
      stream.reset();
      String F =" %" + (3+decimals) + "." + decimals + "f ";
      out.format( F + F + F + "|" + F + "\n", mat00, mat01, mat02, mat03);
      out.format( F + F + F + "|" + F + "\n", mat10, mat11, mat12, mat13);
      out.format( F + F + F + "|" + F + "\n", mat20, mat21, mat22, mat23);
      
      F =" %" + (3+decimals) + "." + 0 + "f ";
      out.format( F + F + F + "|" + F + "\n", 0f,0f,0f,1f );      
      return stream.toString();
   }
   
   
   /**
    * Simple linear interpolation between two transforms. Note that 0<= alpha <= 1
    * No bound limit is performed on alpha.
    */
   public void interpolate(RigidBodyTransform initialTransform, RigidBodyTransform finalTransform, double alpha)
   {     
      Vector3d initialPosition = new Vector3d(); 
      Vector3d finalPosition   = new Vector3d(); 
      
      Quat4d initialRotation = new Quat4d();
      Quat4d finalRotation   = new Quat4d();
      
      initialTransform.get(initialRotation, initialPosition);
      finalTransform.get(finalRotation, finalPosition);
      
      Quat4d computedRotation   = new Quat4d();
      computedRotation.interpolate(initialRotation, finalRotation, alpha);
      
      Vector3d computedPosition = new Vector3d( 
            initialPosition.x *(1.0 - alpha) + finalPosition.x*alpha,
            initialPosition.y *(1.0 - alpha) + finalPosition.y*alpha,
            initialPosition.z *(1.0 - alpha) + finalPosition.z*alpha );
      
      this.set( computedRotation, computedPosition);
   }
   
   static public Vector3d getRotationDifference( Quat4d Q1, Quat4d Q2 )
   { 
      Matrix3d R1  = new Matrix3d();
      Matrix3d R2  = new Matrix3d();
      R1.set(Q1);
      R2.set(Q2);
      return getRotationDifference( R1, R2 );
   }
   
   /**
    * Rotation difference between two rotation matrixes. Translation difference is neglected.
    * This difference is used often to compute the orientation error during an iterative inverse
    * kinematics routine.
    */
   static public Vector3d getRotationDifference( RigidBodyTransform T1, RigidBodyTransform T2 )
   {  
      Matrix3d R1  = new Matrix3d();
      Matrix3d R2  = new Matrix3d();
      T1.getRotation( R1 );
      T2.getRotation( R2 );
      return getRotationDifference( R1, R2 );
   }
   
   /**
    * Rotation difference between two rotation matrixes. Translation difference is neglected.
    * This difference is used often to compute the orientation error during an iterative inverse
    * kinematics routine.
    */
   static public Vector3d getRotationDifference( Matrix3d R1, Matrix3d R2 )
   {    
      Matrix3d Rtemp = new Matrix3d();
      Matrix3d R1t   = new Matrix3d();
      Vector3d axis  = new Vector3d();
      
      R1t.set(R1);
      R1t.transpose();
      Rtemp.mul( R1t, R2 );
      
      AxisAngle4d axis_angle = new AxisAngle4d();

      RotationTools.convertMatrixToAxisAngle(Rtemp, axis_angle);
      
      axis.set( axis_angle.getX(), axis_angle.getY(), axis_angle.getZ() );
      axis.scale( - axis_angle.getAngle());
      R1.transform(axis);
      return  axis ;
   }
   
   static public Vector3d getTranslationDifference( RigidBodyTransform T1, RigidBodyTransform T2 )
   {     
      Vector3d pos1  = new Vector3d();
      Vector3d pos2  = new Vector3d();
      T1.getTranslation( pos1 );
      T2.getTranslation( pos2 );
      pos2.sub(pos1);   
      return pos2;
   } 
   
   public Vector3d cloneTranslation(){
      return new Vector3d(mat03, mat13, mat23 );
   }
   
   public Quat4d cloneRotation(){
      Quat4d output= new Quat4d();
      getRotation(output);
      return output;
   }
      
   
}
