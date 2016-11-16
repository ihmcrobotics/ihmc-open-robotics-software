package us.ihmc.robotics.geometry;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.AxisAngle4f;
import javax.vecmath.Matrix3d;
import javax.vecmath.Matrix3f;
import javax.vecmath.Matrix4d;
import javax.vecmath.Matrix4f;
import javax.vecmath.Quat4d;
import javax.vecmath.Quat4f;
import javax.vecmath.Tuple3d;
import javax.vecmath.Tuple3f;
import javax.vecmath.Vector3d;
import javax.vecmath.Vector3f;

import org.ejml.data.DenseMatrix64F;

/**
 * 
 * This class creates a 4x4 affine, rigid body transformation matrix. The top
 * left 3x3 is a scaled rotation matrix, while the top right 3x1 is a vector
 * describing a translation. If the parameter scale is unity, then the top left
 * 3x3 is an orthogonal rotation matrix.
 * 
 * T = | xx yx zx px | 
 *     | xy yy zy py |  
 *     | xz yz zz pz |  
 *     | 0 0 0 1 |
 */

public class Transform3d extends RigidBodyTransform
{
   private static final long serialVersionUID = -738333942603723019L;
   
   private double rot00 = 1.0;
   private double rot01 = 0.0;
   private double rot02 = 0.0;
   private double rot10 = 0.0;
   private double rot11 = 1.0;
   private double rot12 = 0.0;
   private double rot20 = 0.0;
   private double rot21 = 0.0;
   private double rot22 = 1.0;

   double scale1 = 1.0;
   double scale2 = 1.0;
   double scale3 = 1.0;

   private double[] tmpMat = new double[12];
   private double[] tmp = new double[16];
   private double row_scale[] = new double[4];
   private int row_perm[] = new int[4];

   private double[] t1 = new double[9];
   private double[] t2 = new double[9];
   private double[] svdRot = new double[9];
   private double[] svdScales = new double[3];

   /**
    * Set to identity
    */
   public Transform3d()
   {
      setIdentity();
   }

   public Transform3d(Transform3d transform)
   {
      set(transform);
   }

   /**
    * Create transformation matrix from Matrix4d
    * 
    * @param mat4d
    */
   public Transform3d(Matrix4d matrix)
   {
      set(matrix);
   }

   /**
    * Create transformation matrix from Matrix4f
    * 
    * @param mat4d
    */
   public Transform3d(Matrix4f matrix)
   {
      set(matrix);
   }

   /**
    * Create transform from 1D array of doubles.
    * 
    * @param doubleArray
    */
   public Transform3d(double[] doubleArray)
   {
      set(doubleArray);
   }

   /**
    * Create transform from 1D array of floats.
    * 
    * @param floatArray
    */
   public Transform3d(float[] floatArray)
   {
      set(floatArray);
   }

   /**
    * Create transform from 4x4 DenseMatrix64F
    * 
    * @param matrix
    */
   public Transform3d(DenseMatrix64F matrix)
   {
      set(matrix);
   }

   /**
    * Create 4x4 Transform3d from rotation matrix of type DenseMatrix64F and
    * translational vector of type Vector3d
    * 
    * @param matrix
    * @param vector
    * @param scales
    */
   public Transform3d(DenseMatrix64F matrix, Vector3d vector, Vector3d scales)
   {
      if (matrix.numRows != 3 && matrix.numCols != 3)
      {
         throw new RuntimeException("Incorrectly sized matrix. Must be 3x3");
      }
      set(matrix, vector, scales);
   }

   /**
    * Create Transform3d from rotation matrix, translation vector and 3 scale factors.
    * @param matrix
    * @param vector
    * @param scalex
    * @param scaley
    * @param scalez
    */
   public Transform3d(DenseMatrix64F matrix, Vector3d vector, double scalex, double scaley, double scalez)
   {
      if (matrix.numRows != 3 && matrix.numCols != 3)
      {
         throw new RuntimeException("Incorrectly sized matrix. Must be 3x3");
      }
      set(matrix, vector, scalex, scaley, scalez);
   }

   /**
    * Create 4x4 Transform3d from rotation matrix of type DenseMatrix64F and
    * translational vector of type Vector3d
    * 
    * @param matrix
    * @param vector
    */
   public Transform3d(DenseMatrix64F matrix, Vector3d vector)
   {
      set(matrix, vector);
   }

   /**
    * Create 4x4 Transform3d from rotation matrix of type DenseMatrix64F and
    * translational vector of type Vector3d and set the scale factor equal to
    * the double scale.
    * 
    * @param matrix
    * @param vector
    * @param scale
    */
   public Transform3d(DenseMatrix64F matrix, Vector3d vector, double scale)
   {
      if (matrix.numRows != 3 && matrix.numCols != 3)
      {
         throw new RuntimeException("Incorrectly sized matrix. Must be 3x3");
      }
      set(matrix, vector, scale);
   }

   /**
    * Set this transform to have rotation matrix equal to matrix, translation 
    * equal to vector, and scale1 = scalex, scale2 = scaley, scale3 = scalez.
    * 
    * @param matrix
    * @param vector
    * @param scalex
    * @param scaley
    * @param scalez
    */
   public Transform3d(Matrix3d matrix, Vector3d vector, double scalex, double scaley, double scalez)
   {
      set(matrix, vector, scalex, scaley, scalez);
   }

   /**
    * Set this transform with rotation described by matrix, 
    * translation described by vector, and scales equal 
    * to the argument scale.
    * 
    * @param matrix
    * @param vector
    * @param scale
    */
   public Transform3d(Matrix3d matrix, Vector3d vector, double scale)
   {
      set(matrix, vector, scale);
   }

   /**
    * Create transformation matrix from rotation matrix and vector translation
    * 
    * @param matrix
    * @param vector
    */
   public Transform3d(Matrix3d matrix, Vector3d vector)
   {
      set(matrix, vector);
   }

   /**
    * Create transformation matrix from rotation matrix and vector translation
    * 
    * @param matrix
    * @param vector
    */
   public Transform3d(Matrix3f matrix, Vector3f vector)
   {
      set(matrix, vector);
   }

   /**
    * Create transformation matrix from rotation matrix and vector translation
    * 
    * @param matrix
    * @param vector
    * @param scale
    */
   public Transform3d(Matrix3f matrix, Vector3f vector, double scale)
   {
      set(matrix, vector, scale);
   }

   /**
    * Create transformation matrix from rotation matrix and vector translation
    * 
    * @param matrix
    * @param vector
    * @param scalex
    * @param scaley
    * @param scalez
    */
   public Transform3d(Matrix3f matrix, Vector3f vector, double scalex, double scaley, double scalez)
   {
      set(matrix, vector, scalex, scaley, scalez);
   }

   /**
    * Create Transform3d from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    */
   public Transform3d(Quat4d quat, Vector3d vector, Vector3d scales)
   {
      set(quat, vector, scales);
   }

   /**
    * Create Transform3d from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    */
   public Transform3d(Quat4d quat, Vector3d vector)
   {
      set(quat, vector);
   }

   /**
    * Create Transform3d from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    * @param scale
    */
   public Transform3d(Quat4d quat, Vector3d vector, double scale)
   {
      set(quat, vector, scale);
   }

   /**
    * Create Transform3d from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    * @param scalex
    * @param scaley
    * @param scalez
    */
   public Transform3d(Quat4d quat, Vector3d vector, double scalex, double scaley, double scalez)
   {
      set(quat, vector, scalex, scaley, scalez);
   }

   /**
    * Create Transform3d from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    */
   public Transform3d(Quat4f quat, Vector3f vector)
   {
      set(quat, vector);
   }

   /**
    * Create Transform3d from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    */
   public Transform3d(Quat4f quat, Vector3f vector, Vector3f scales)
   {
      set(quat, vector, scales);
   }

   /**
    * Create Transform3d from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    * @param scale
    */
   public Transform3d(Quat4f quat, Vector3f vector, double scale)
   {
      set(quat, vector, scale);
   }

   /**
    * Create Transform3d from quaternion describing a rotation and vector
    * describing a translation.
    * 
    * @param quat
    * @param vector
    * @param scalex
    * @param scaley
    * @param scalez
    */
   public Transform3d(Quat4f quat, Vector3f vector, double scalex, double scaley, double scalez)
   {
      set(quat, vector, scalex, scaley, scalez);
   }

   /**
    * Create Transform3d from AxisAngle4d and Vector3d
    * 
    * @param axisAngle
    * @param vector
    * @param scales
    */
   public Transform3d(AxisAngle4d axisAngle, Vector3d vector, Vector3d scales)
   {
      set(axisAngle, vector, scales);
   }

   /**
    * Create Transform3d from AxisAngle4d and Vector3d
    * 
    * @param axisAngle
    * @param vector
    */
   public Transform3d(AxisAngle4d axisAngle, Vector3d vector)
   {
      set(axisAngle, vector);
   }

   /**
    * Create Transform3d from AxisAngle4d and Vector3d
    * 
    * @param axisAngle
    * @param vector
    * @param scale
    */
   public Transform3d(AxisAngle4d axisAngle, Vector3d vector, double scale)
   {
      set(axisAngle, vector, scale);
   }

   /**
    * Create Transform3d from AxisAngle4d and Vector3d
    * 
    * @param axisAngle
    * @param vector
    * @param scalex
    * @param scaley
    * @param scalez
    */
   public Transform3d(AxisAngle4d axisAngle, Vector3d vector, double scalex, double scaley, double scalez)
   {
      set(axisAngle, vector, scalex, scaley, scalez);
   }

   /**
    * Create Transform3d from AxisAngle4f and Vector3f
    * 
    * @param axisAngle
    * @param vector
    */
   public Transform3d(AxisAngle4f axisAngle, Vector3f vector)
   {
      set(axisAngle, vector);
   }

   /**
    * Create Transform3d from AxisAngle4f and Vector3f
    * 
    * @param axisAngle
    * @param vector
    */
   public Transform3d(AxisAngle4f axisAngle, Vector3f vector, Vector3f scales)
   {
      set(axisAngle, vector, scales);
   }

   /**
    * Create Transform3d from AxisAngle4f and Vector3f
    * 
    * @param axisAngle
    * @param vector
    * @param scale
    */
   public Transform3d(AxisAngle4f axisAngle, Vector3f vector, double scale)
   {
      set(axisAngle, vector, scale);
   }

   /**
    * Create Transform3d from AxisAngle4f and Vector3f
    * 
    * @param axisAngle
    * @param vector
    * @param scalex
    * @param scaley
    * @param scalez
    */
   public Transform3d(AxisAngle4f axisAngle, Vector3f vector, double scalex, double scaley, double scalez)
   {
      set(axisAngle, vector, scalex, scaley, scalez);
   }

   /**
    * Convert AxisAngle representation to rotation matrix.
    * 
    * @param axisAngle
    */
   @Override
   public void setRotation(AxisAngle4d axisAngle)
   {
      computeScale();

      double mag = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());

      if (almostZero(mag))
      {
         mat00 = scale1;
         mat01 = 0.0;
         mat02 = 0.0;
         mat10 = 0.0;
         mat11 = scale2;
         mat12 = 0.0;
         mat20 = 0.0;
         mat21 = 0.0;
         mat22 = scale3;
      }
      else
      {
         mag = 1.0 / mag;
         double ax = axisAngle.getX() * mag;
         double ay = axisAngle.getY() * mag;
         double az = axisAngle.getZ() * mag;

         double sinTheta = Math.sin(axisAngle.getAngle());
         double cosTheta = Math.cos(axisAngle.getAngle());
         double t = 1.0 - cosTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = (t * ax * ax + cosTheta) * scale1;
         mat01 = (t * xy - sinTheta * az) * scale2;
         mat02 = (t * xz + sinTheta * ay) * scale3;

         mat10 = (t * xy + sinTheta * az) * scale1;
         mat11 = (t * ay * ay + cosTheta) * scale2;
         mat12 = (t * yz - sinTheta * ax) * scale3;

         mat20 = (t * xz - sinTheta * ay) * scale1;
         mat21 = (t * yz + sinTheta * ax) * scale2;
         mat22 = (t * az * az + cosTheta) * scale3;
      }
   }

   /**
    * Convert AxisAngle representation to rotation matrix.
    * 
    * @param axisAngle
    */
   @Override
   public void setRotation(AxisAngle4f axisAngle)
   {
      computeScale();

      double mag = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());

      if (almostZero(mag))
      {
         mat00 = scale1;
         mat01 = 0.0;
         mat02 = 0.0;
         mat10 = 0.0;
         mat11 = scale2;
         mat12 = 0.0;
         mat20 = 0.0;
         mat21 = 0.0;
         mat22 = scale3;
      }
      else
      {
         mag = 1.0 / mag;
         double ax = axisAngle.getX() * mag;
         double ay = axisAngle.getY() * mag;
         double az = axisAngle.getZ() * mag;

         double sinTheta = Math.sin(axisAngle.getAngle());
         double cosTheta = Math.cos(axisAngle.getAngle());
         double t = 1.0 - cosTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = (t * ax * ax + cosTheta) * scale1;
         mat01 = (t * xy - sinTheta * az) * scale2;
         mat02 = (t * xz + sinTheta * ay) * scale3;

         mat10 = (t * xy + sinTheta * az) * scale1;
         mat11 = (t * ay * ay + cosTheta) * scale2;
         mat12 = (t * yz - sinTheta * ax) * scale3;

         mat20 = (t * xz - sinTheta * ay) * scale1;
         mat21 = (t * yz + sinTheta * ax) * scale2;
         mat22 = (t * az * az + cosTheta) * scale3;
      }
   }

   /**
    * Convert quaternion to rotation matrix.
    * 
    * @param quat
    */
   @Override
   public final void setRotation(Quat4d quat)
   {
      computeScale();

      double yy2 = 2.0 * quat.getY() * quat.getY();
      double zz2 = 2.0 * quat.getZ() * quat.getZ();
      double xx2 = 2.0 * quat.getX() * quat.getX();
      double xy2 = 2.0 * quat.getX() * quat.getY();
      double wz2 = 2.0 * quat.getW() * quat.getZ();
      double xz2 = 2.0 * quat.getX() * quat.getZ();
      double wy2 = 2.0 * quat.getW() * quat.getY();
      double yz2 = 2.0 * quat.getY() * quat.getZ();
      double wx2 = 2.0 * quat.getW() * quat.getX();

      mat00 = (1.0 - yy2 - zz2) * scale1;
      mat01 = (xy2 - wz2) * scale2;
      mat02 = (xz2 + wy2) * scale3;
      mat10 = (xy2 + wz2) * scale1;
      mat11 = (1.0 - xx2 - zz2) * scale2;
      mat12 = (yz2 - wx2) * scale3;
      mat20 = (xz2 - wy2) * scale1;
      mat21 = (yz2 + wx2) * scale2;
      mat22 = (1.0 - xx2 - yy2) * scale3;
   }

   /**
    * Convert quaternion to rotation matrix.
    * 
    * @param quat
    */
   @Override
   public void setRotation(Quat4f quat)
   {
      computeScale();

      double yy2 = 2.0 * quat.getY() * quat.getY();
      double zz2 = 2.0 * quat.getZ() * quat.getZ();
      double xx2 = 2.0 * quat.getX() * quat.getX();
      double xy2 = 2.0 * quat.getX() * quat.getY();
      double wz2 = 2.0 * quat.getW() * quat.getZ();
      double xz2 = 2.0 * quat.getX() * quat.getZ();
      double wy2 = 2.0 * quat.getW() * quat.getY();
      double yz2 = 2.0 * quat.getY() * quat.getZ();
      double wx2 = 2.0 * quat.getW() * quat.getX();

      mat00 = (1.0 - yy2 - zz2) * scale1;
      mat01 = (xy2 - wz2) * scale2;
      mat02 = (xz2 + wy2) * scale3;
      mat10 = (xy2 + wz2) * scale1;
      mat11 = (1.0 - xx2 - zz2) * scale2;
      mat12 = (yz2 - wx2) * scale3;
      mat20 = (xz2 - wy2) * scale1;
      mat21 = (yz2 + wx2) * scale2;
      mat22 = (1.0 - xx2 - yy2) * scale3;
   }

   /**
    * Set the 3x3 rotation matrix equal to mat3d. Preserves scale.
    * 
    * @param matrix
    */
   @Override
   public void setRotation(Matrix3d matrix)
   {
      computeScale();

      mat00 = scale1 * matrix.getM00();
      mat01 = scale2 * matrix.getM01();
      mat02 = scale3 * matrix.getM02();
      mat10 = scale1 * matrix.getM10();
      mat11 = scale2 * matrix.getM11();
      mat12 = scale3 * matrix.getM12();
      mat20 = scale1 * matrix.getM20();
      mat21 = scale2 * matrix.getM21();
      mat22 = scale3 * matrix.getM22();
   }

   /**
    * Set the 3x3 rotation matrix equal to mat3f. Preserves scale.
    * 
    * @param mat3d
    */
   @Override
   public final void setRotation(Matrix3f matrix)
   {
      computeScale();

      mat00 = scale1 * matrix.getM00();
      mat01 = scale2 * matrix.getM01();
      mat02 = scale3 * matrix.getM02();
      mat10 = scale1 * matrix.getM10();
      mat11 = scale2 * matrix.getM11();
      mat12 = scale3 * matrix.getM12();
      mat20 = scale1 * matrix.getM20();
      mat21 = scale2 * matrix.getM21();
      mat22 = scale3 * matrix.getM22();
   }

   /**
    * Sets the top left 3x3 equal to matrix.
    * 
    * @param matrix
    */
   public void setRotationScale(Matrix3f matrix)
   {
      mat00 = matrix.getM00();
      mat01 = matrix.getM01();
      mat02 = matrix.getM02();
      mat10 = matrix.getM10();
      mat11 = matrix.getM11();
      mat12 = matrix.getM12();
      mat20 = matrix.getM20();
      mat21 = matrix.getM21();
      mat22 = matrix.getM22();
   }

   /**
    * Sets the top left 3x3 equal to matrix.
    * 
    * @param matrix
    */
   public void setRotationScale(Matrix3d matrix)
   {
      mat00 = matrix.getM00();
      mat01 = matrix.getM01();
      mat02 = matrix.getM02();
      mat10 = matrix.getM10();
      mat11 = matrix.getM11();
      mat12 = matrix.getM12();
      mat20 = matrix.getM20();
      mat21 = matrix.getM21();
      mat22 = matrix.getM22();
   }

   /**
    * Sets rotation portion equal to the rotation matrix described in the
    * parameter matrix. Preserves scale.
    * 
    * @param matrix
    */
   @Override
   public final void setRotation(DenseMatrix64F matrix)
   {
      if (matrix.numRows != 3 && matrix.numCols != 3)
      {
         throw new RuntimeException("Improperly sized matrix. Must be 3x3.");
      }
      computeScale();

      mat00 = scale1 * matrix.get(0, 0);
      mat01 = scale2 * matrix.get(0, 1);
      mat02 = scale3 * matrix.get(0, 2);
      mat10 = scale1 * matrix.get(1, 0);
      mat11 = scale2 * matrix.get(1, 1);
      mat12 = scale3 * matrix.get(1, 2);
      mat20 = scale1 * matrix.get(2, 0);
      mat21 = scale2 * matrix.get(2, 1);
      mat22 = scale3 * matrix.get(2, 2);
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3d matrix.
    * 
    * @param matrix
    */
   public final void set(Matrix3d matrix, Vector3d vector, double scale)
   {
      mat00 = scale * matrix.getM00();
      mat01 = scale * matrix.getM01();
      mat02 = scale * matrix.getM02();
      mat10 = scale * matrix.getM10();
      mat11 = scale * matrix.getM11();
      mat12 = scale * matrix.getM12();
      mat20 = scale * matrix.getM20();
      mat21 = scale * matrix.getM21();
      mat22 = scale * matrix.getM22();
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3d matrix.
    * 
    * @param matrix
    */
   public final void set(Matrix3d matrix, Vector3d vector, Vector3d scales)
   {
      mat00 = scales.getX() * matrix.getM00();
      mat01 = scales.getY() * matrix.getM01();
      mat02 = scales.getZ() * matrix.getM02();
      mat10 = scales.getX() * matrix.getM10();
      mat11 = scales.getY() * matrix.getM11();
      mat12 = scales.getZ() * matrix.getM12();
      mat20 = scales.getX() * matrix.getM20();
      mat21 = scales.getY() * matrix.getM21();
      mat22 = scales.getZ() * matrix.getM22();
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3d matrix.
    * 
    * @param matrix
    */
   public final void set(Matrix3d matrix, Vector3d vector, double scalex, double scaley, double scalez)
   {
      mat00 = scalex * matrix.getM00();
      mat01 = scaley * matrix.getM01();
      mat02 = scalez * matrix.getM02();
      mat10 = scalex * matrix.getM10();
      mat11 = scaley * matrix.getM11();
      mat12 = scalez * matrix.getM12();
      mat20 = scalex * matrix.getM20();
      mat21 = scaley * matrix.getM21();
      mat22 = scalez * matrix.getM22();
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Set this transform to have translation described in vector and a rotation
    * equal to the Quat4d quat.
    * 
    * @param quat
    */
   public final void set(Quat4d quat, Vector3d vector, double scale)
   {
      double yy2 = 2.0 * quat.getY() * quat.getY();
      double zz2 = 2.0 * quat.getZ() * quat.getZ();
      double xx2 = 2.0 * quat.getX() * quat.getX();
      double xy2 = 2.0 * quat.getX() * quat.getY();
      double wz2 = 2.0 * quat.getW() * quat.getZ();
      double xz2 = 2.0 * quat.getX() * quat.getZ();
      double wy2 = 2.0 * quat.getW() * quat.getY();
      double yz2 = 2.0 * quat.getY() * quat.getZ();
      double wx2 = 2.0 * quat.getW() * quat.getX();

      mat00 = (1.0 - yy2 - zz2) * scale;
      mat01 = (xy2 - wz2) * scale;
      mat02 = (xz2 + wy2) * scale;
      mat10 = (xy2 + wz2) * scale;
      mat11 = (1.0 - xx2 - zz2) * scale;
      mat12 = (yz2 - wx2) * scale;
      mat20 = (xz2 - wy2) * scale;
      mat21 = (yz2 + wx2) * scale;
      mat22 = (1.0 - xx2 - yy2) * scale;

      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Set this transform to have translation described in vector and a rotation
    * equal to the Quat4d quat.
    * 
    * @param quat
    */
   public final void set(Quat4d quat, Vector3d vector, Vector3d scales)
   {
      double yy2 = 2.0 * quat.getY() * quat.getY();
      double zz2 = 2.0 * quat.getZ() * quat.getZ();
      double xx2 = 2.0 * quat.getX() * quat.getX();
      double xy2 = 2.0 * quat.getX() * quat.getY();
      double wz2 = 2.0 * quat.getW() * quat.getZ();
      double xz2 = 2.0 * quat.getX() * quat.getZ();
      double wy2 = 2.0 * quat.getW() * quat.getY();
      double yz2 = 2.0 * quat.getY() * quat.getZ();
      double wx2 = 2.0 * quat.getW() * quat.getX();

      mat00 = (1.0 - yy2 - zz2) * scales.getX();
      mat01 = (xy2 - wz2) * scales.getY();
      mat02 = (xz2 + wy2) * scales.getZ();
      mat10 = (xy2 + wz2) * scales.getX();
      mat11 = (1.0 - xx2 - zz2) * scales.getY();
      mat12 = (yz2 - wx2) * scales.getZ();
      mat20 = (xz2 - wy2) * scales.getX();
      mat21 = (yz2 + wx2) * scales.getY();
      mat22 = (1.0 - xx2 - yy2) * scales.getZ();

      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Set this transform to have translation described in vector and a rotation
    * equal to the Quat4d quat.
    * 
    * @param quat
    */
   public final void set(Quat4d quat, Vector3d vector, double scalex, double scaley, double scalez)
   {
      double yy2 = 2.0 * quat.getY() * quat.getY();
      double zz2 = 2.0 * quat.getZ() * quat.getZ();
      double xx2 = 2.0 * quat.getX() * quat.getX();
      double xy2 = 2.0 * quat.getX() * quat.getY();
      double wz2 = 2.0 * quat.getW() * quat.getZ();
      double xz2 = 2.0 * quat.getX() * quat.getZ();
      double wy2 = 2.0 * quat.getW() * quat.getY();
      double yz2 = 2.0 * quat.getY() * quat.getZ();
      double wx2 = 2.0 * quat.getW() * quat.getX();

      mat00 = (1.0 - yy2 - zz2) * scalex;
      mat01 = (xy2 - wz2) * scaley;
      mat02 = (xz2 + wy2) * scalez;
      mat10 = (xy2 + wz2) * scalex;
      mat11 = (1.0 - xx2 - zz2) * scaley;
      mat12 = (yz2 - wx2) * scalez;
      mat20 = (xz2 - wy2) * scalex;
      mat21 = (yz2 + wx2) * scaley;
      mat22 = (1.0 - xx2 - yy2) * scalez;

      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Sets this transform to have rotation described by axisAngle and zero
    * translation.
    * 
    * @param axisAngle
    */
   public final void set(AxisAngle4d axisAngle, Vector3d vector, double scale)
   {
      double mag = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());

      if (Math.abs(mag) < 1.0e-5)
      {
         mat00 = scale;
         mat01 = 0.0;
         mat02 = 0.0;
         mat10 = 0.0;
         mat11 = scale;
         mat12 = 0.0;
         mat20 = 0.0;
         mat21 = 0.0;
         mat22 = scale;
      }
      else
      {
         double ax = axisAngle.getX() / mag;
         double ay = axisAngle.getY() / mag;
         double az = axisAngle.getZ() / mag;

         double sTheta = Math.sin(axisAngle.getAngle());
         double cTheta = Math.cos(axisAngle.getAngle());
         double t = 1.0 - cTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = scale * (t * ax * ax + cTheta);
         mat01 = scale * (t * xy - sTheta * az);
         mat02 = scale * (t * xz + sTheta * ay);
         mat10 = scale * (t * xy + sTheta * az);
         mat11 = scale * (t * ay * ay + cTheta);
         mat12 = scale * (t * yz - sTheta * ax);
         mat20 = scale * (t * xz - sTheta * ay);
         mat21 = scale * (t * yz + sTheta * ax);
         mat22 = scale * (t * az * az + cTheta);
      }
      
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Sets this transform to have rotation described by axisAngle and zero
    * translation.
    * 
    * @param axisAngle
    */
   public final void set(AxisAngle4d axisAngle, Vector3d vector, Vector3d scales)
   {
      double mag = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());

      if (Math.abs(mag) < 1.0e-5)
      {
         mat00 = scales.getX();
         mat01 = 0.0;
         mat02 = 0.0;
         mat10 = 0.0;
         mat11 = scales.getY();
         mat12 = 0.0;
         mat20 = 0.0;
         mat21 = 0.0;
         mat22 = scales.getZ();
      }
      else
      {
         double ax = axisAngle.getX() / mag;
         double ay = axisAngle.getY() / mag;
         double az = axisAngle.getZ() / mag;

         double sTheta = Math.sin(axisAngle.getAngle());
         double cTheta = Math.cos(axisAngle.getAngle());
         double t = 1.0 - cTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = scales.getX() * (t * ax * ax + cTheta);
         mat01 = scales.getY() * (t * xy - sTheta * az);
         mat02 = scales.getZ() * (t * xz + sTheta * ay);
         mat10 = scales.getX() * (t * xy + sTheta * az);
         mat11 = scales.getY() * (t * ay * ay + cTheta);
         mat12 = scales.getZ() * (t * yz - sTheta * ax);
         mat20 = scales.getX() * (t * xz - sTheta * ay);
         mat21 = scales.getY() * (t * yz + sTheta * ax);
         mat22 = scales.getZ() * (t * az * az + cTheta);
      }
      
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Sets this transform to have rotation described by axisAngle and zero
    * translation.
    * 
    * @param axisAngle
    */
   public final void set(AxisAngle4d axisAngle, Vector3d vector, double scalex, double scaley, double scalez)
   {
      double mag = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());

      if (Math.abs(mag) < 1.0e-5)
      {
         mat00 = scalex;
         mat01 = 0.0;
         mat02 = 0.0;
         mat10 = 0.0;
         mat11 = scaley;
         mat12 = 0.0;
         mat20 = 0.0;
         mat21 = 0.0;
         mat22 = scalez;
      }
      else
      {
         double ax = axisAngle.getX() / mag;
         double ay = axisAngle.getY() / mag;
         double az = axisAngle.getZ() / mag;

         double sTheta = Math.sin(axisAngle.getAngle());
         double cTheta = Math.cos(axisAngle.getAngle());
         double t = 1.0 - cTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = scalex * (t * ax * ax + cTheta);
         mat01 = scaley * (t * xy - sTheta * az);
         mat02 = scalez * (t * xz + sTheta * ay);
         mat10 = scalex * (t * xy + sTheta * az);
         mat11 = scaley * (t * ay * ay + cTheta);
         mat12 = scalez * (t * yz - sTheta * ax);
         mat20 = scalex * (t * xz - sTheta * ay);
         mat21 = scaley * (t * yz + sTheta * ax);
         mat22 = scalez * (t * az * az + cTheta);
      }
      
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Sets this transform to have rotation described by axisAngle and zero
    * translation.
    * 
    * @param axisAngle
    */
   public final void set(AxisAngle4f axisAngle, Vector3f vector, double scale)
   {
      double mag = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());

      if (Math.abs(mag) < 1.0e-5)
      {
         mat00 = scale;
         mat01 = 0.0;
         mat02 = 0.0;
         mat10 = 0.0;
         mat11 = scale;
         mat12 = 0.0;
         mat20 = 0.0;
         mat21 = 0.0;
         mat22 = scale;
      }
      else
      {
         double ax = axisAngle.getX() / mag;
         double ay = axisAngle.getY() / mag;
         double az = axisAngle.getZ() / mag;

         double sTheta = Math.sin(axisAngle.getAngle());
         double cTheta = Math.cos(axisAngle.getAngle());
         double t = 1.0 - cTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = scale * (t * ax * ax + cTheta);
         mat01 = scale * (t * xy - sTheta * az);
         mat02 = scale * (t * xz + sTheta * ay);
         mat10 = scale * (t * xy + sTheta * az);
         mat11 = scale * (t * ay * ay + cTheta);
         mat12 = scale * (t * yz - sTheta * ax);
         mat20 = scale * (t * xz - sTheta * ay);
         mat21 = scale * (t * yz + sTheta * ax);
         mat22 = scale * (t * az * az + cTheta);
      }
      
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Sets this transform to have rotation described by axisAngle and zero
    * translation.
    * 
    * @param axisAngle
    */
   public final void set(AxisAngle4f axisAngle, Vector3f vector, Vector3f scales)
   {
      double mag = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());

      if (Math.abs(mag) < 1.0e-5)
      {
         mat00 = scales.getX();
         mat01 = 0.0;
         mat02 = 0.0;
         mat10 = 0.0;
         mat11 = scales.getY();
         mat12 = 0.0;
         mat20 = 0.0;
         mat21 = 0.0;
         mat22 = scales.getZ();
      }
      else
      {
         double ax = axisAngle.getX() / mag;
         double ay = axisAngle.getY() / mag;
         double az = axisAngle.getZ() / mag;

         double sTheta = Math.sin(axisAngle.getAngle());
         double cTheta = Math.cos(axisAngle.getAngle());
         double t = 1.0 - cTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = scales.getX() * (t * ax * ax + cTheta);
         mat01 = scales.getY() * (t * xy - sTheta * az);
         mat02 = scales.getZ() * (t * xz + sTheta * ay);
         mat10 = scales.getX() * (t * xy + sTheta * az);
         mat11 = scales.getY() * (t * ay * ay + cTheta);
         mat12 = scales.getZ() * (t * yz - sTheta * ax);
         mat20 = scales.getX() * (t * xz - sTheta * ay);
         mat21 = scales.getY() * (t * yz + sTheta * ax);
         mat22 = scales.getZ() * (t * az * az + cTheta);
      }
      
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Sets this transform to have rotation described by axisAngle and zero
    * translation.
    * 
    * @param axisAngle
    */
   public final void set(AxisAngle4f axisAngle, Vector3f vector, double scalex, double scaley, double scalez)
   {
      double mag = Math.sqrt(axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ());

      if (Math.abs(mag) < 1.0e-5)
      {
         mat00 = scalex;
         mat01 = 0.0;
         mat02 = 0.0;
         mat10 = 0.0;
         mat11 = scaley;
         mat12 = 0.0;
         mat20 = 0.0;
         mat21 = 0.0;
         mat22 = scalez;
      }
      else
      {
         double ax = axisAngle.getX() / mag;
         double ay = axisAngle.getY() / mag;
         double az = axisAngle.getZ() / mag;

         double sTheta = Math.sin(axisAngle.getAngle());
         double cTheta = Math.cos(axisAngle.getAngle());
         double t = 1.0 - cTheta;

         double xz = ax * az;
         double xy = ax * ay;
         double yz = ay * az;

         mat00 = scalex * (t * ax * ax + cTheta);
         mat01 = scaley * (t * xy - sTheta * az);
         mat02 = scalez * (t * xz + sTheta * ay);
         mat10 = scalex * (t * xy + sTheta * az);
         mat11 = scaley * (t * ay * ay + cTheta);
         mat12 = scalez * (t * yz - sTheta * ax);
         mat20 = scalex * (t * xz - sTheta * ay);
         mat21 = scaley * (t * yz + sTheta * ax);
         mat22 = scalez * (t * az * az + cTheta);
      }
      
      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Set this transform to have translation described in vector and a rotation
    * equal to the Quat4f quat.
    * 
    * @param quat
    */
   public final void set(Quat4f quat, Vector3f vector, double scale)
   {
      double yy2 = 2.0 * quat.getY() * quat.getY();
      double zz2 = 2.0 * quat.getZ() * quat.getZ();
      double xx2 = 2.0 * quat.getX() * quat.getX();
      double xy2 = 2.0 * quat.getX() * quat.getY();
      double wz2 = 2.0 * quat.getW() * quat.getZ();
      double xz2 = 2.0 * quat.getX() * quat.getZ();
      double wy2 = 2.0 * quat.getW() * quat.getY();
      double yz2 = 2.0 * quat.getY() * quat.getZ();
      double wx2 = 2.0 * quat.getW() * quat.getX();

      mat00 = (1.0 - yy2 - zz2) * scale;
      mat01 = (xy2 - wz2) * scale;
      mat02 = (xz2 + wy2) * scale;
      mat10 = (xy2 + wz2) * scale;
      mat11 = (1.0 - xx2 - zz2) * scale;
      mat12 = (yz2 - wx2) * scale;
      mat20 = (xz2 - wy2) * scale;
      mat21 = (yz2 + wx2) * scale;
      mat22 = (1.0 - xx2 - yy2) * scale;

      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Set this transform to have translation described in vector and a rotation
    * equal to the Quat4f quat.
    * 
    * @param quat
    */
   public final void set(Quat4f quat, Vector3f vector, Vector3f scales)
   {
      double yy2 = 2.0 * quat.getY() * quat.getY();
      double zz2 = 2.0 * quat.getZ() * quat.getZ();
      double xx2 = 2.0 * quat.getX() * quat.getX();
      double xy2 = 2.0 * quat.getX() * quat.getY();
      double wz2 = 2.0 * quat.getW() * quat.getZ();
      double xz2 = 2.0 * quat.getX() * quat.getZ();
      double wy2 = 2.0 * quat.getW() * quat.getY();
      double yz2 = 2.0 * quat.getY() * quat.getZ();
      double wx2 = 2.0 * quat.getW() * quat.getX();

      mat00 = (1.0 - yy2 - zz2) * scales.getX();
      mat01 = (xy2 - wz2) * scales.getY();
      mat02 = (xz2 + wy2) * scales.getZ();
      mat10 = (xy2 + wz2) * scales.getX();
      mat11 = (1.0 - xx2 - zz2) * scales.getY();
      mat12 = (yz2 - wx2) * scales.getZ();
      mat20 = (xz2 - wy2) * scales.getX();
      mat21 = (yz2 + wx2) * scales.getY();
      mat22 = (1.0 - xx2 - yy2) * scales.getZ();

      setTranslation(vector.getX(),vector.getY(),vector.getZ());
   }

   /**
    * Set this transform to have translation described in vector and a rotation
    * equal to the Quat4f quat.
    * 
    * @param quat
    */
   public final void set(Quat4f quat, Vector3f vector, double scalex, double scaley, double scalez)
   {
      double yy2 = 2.0 * quat.getY() * quat.getY();
      double zz2 = 2.0 * quat.getZ() * quat.getZ();
      double xx2 = 2.0 * quat.getX() * quat.getX();
      double xy2 = 2.0 * quat.getX() * quat.getY();
      double wz2 = 2.0 * quat.getW() * quat.getZ();
      double xz2 = 2.0 * quat.getX() * quat.getZ();
      double wy2 = 2.0 * quat.getW() * quat.getY();
      double yz2 = 2.0 * quat.getY() * quat.getZ();
      double wx2 = 2.0 * quat.getW() * quat.getX();

      mat00 = (1.0 - yy2 - zz2) * scalex;
      mat01 = (xy2 - wz2) * scaley;
      mat02 = (xz2 + wy2) * scalez;
      mat10 = (xy2 + wz2) * scalex;
      mat11 = (1.0 - xx2 - zz2) * scaley;
      mat12 = (yz2 - wx2) * scalez;
      mat20 = (xz2 - wy2) * scalex;
      mat21 = (yz2 + wx2) * scaley;
      mat22 = (1.0 - xx2 - yy2) * scalez;

      mat03 = vector.getX();
      mat13 = vector.getY();
      mat23 = vector.getZ();
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3f matrix.
    * 
    * @param matrix
    */
   public final void set(Matrix3f matrix, Vector3f vector, double scale)
   {
      mat00 = scale * matrix.getM00();
      mat01 = scale * matrix.getM01();
      mat02 = scale * matrix.getM02();
      mat10 = scale * matrix.getM10();
      mat11 = scale * matrix.getM11();
      mat12 = scale * matrix.getM12();
      mat20 = scale * matrix.getM20();
      mat21 = scale * matrix.getM21();
      mat22 = scale * matrix.getM22();

      mat03 = vector.getX();
      mat13 = vector.getY();
      mat23 = vector.getZ();
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3f matrix.
    * 
    * @param matrix
    */
   public final void set(Matrix3f matrix, Vector3f vector, Vector3f scales)
   {
      mat00 = scales.getX() * matrix.getM00();
      mat01 = scales.getY() * matrix.getM01();
      mat02 = scales.getZ() * matrix.getM02();
      mat10 = scales.getX() * matrix.getM10();
      mat11 = scales.getY() * matrix.getM11();
      mat12 = scales.getZ() * matrix.getM12();
      mat20 = scales.getX() * matrix.getM20();
      mat21 = scales.getY() * matrix.getM21();
      mat22 = scales.getZ() * matrix.getM22();

      mat03 = vector.getX();
      mat13 = vector.getY();
      mat23 = vector.getZ();
   }

   /**
    * Set this transform to have zero translation and a rotation equal to the
    * Matrix3f matrix.
    * 
    * @param matrix
    */
   public final void set(Matrix3f matrix, Vector3f vector, double scalex, double scaley, double scalez)
   {
      mat00 = scalex * matrix.getM00();
      mat01 = scaley * matrix.getM01();
      mat02 = scalez * matrix.getM02();
      mat10 = scalex * matrix.getM10();
      mat11 = scaley * matrix.getM11();
      mat12 = scalez * matrix.getM12();
      mat20 = scalex * matrix.getM20();
      mat21 = scaley * matrix.getM21();
      mat22 = scalez * matrix.getM22();

      mat03 = vector.getX();
      mat13 = vector.getY();
      mat23 = vector.getZ();
   }

   /**
    * Set elements of this transform equal to the elements of matrix.
    * 
    * @param matrix
    */
   public final void set(DenseMatrix64F matrix, Vector3d vector, double scale)
   {
      if (matrix.numCols != 3 && matrix.numRows != 3)
      {
         throw new RuntimeException("Incorrectly sized matrix. Matrix must be 4x4");
      }

      mat00 = scale * matrix.get(0, 0);
      mat01 = scale * matrix.get(0, 1);
      mat02 = scale * matrix.get(0, 2);
      mat10 = scale * matrix.get(1, 0);
      mat11 = scale * matrix.get(1, 1);
      mat12 = scale * matrix.get(1, 2);
      mat20 = scale * matrix.get(2, 0);
      mat21 = scale * matrix.get(2, 1);
      mat22 = scale * matrix.get(2, 2);
      mat03 = vector.getX();
      mat13 = vector.getY();
      mat23 = vector.getZ();
   }

   /**
    * Set elements of this transform equal to the elements of matrix.
    * 
    * @param matrix
    */
   public final void set(DenseMatrix64F matrix, Vector3d vector, Vector3d scales)
   {
      if (matrix.numCols != 3 && matrix.numRows != 3)
      {
         throw new RuntimeException("Incorrectly sized matrix. Matrix must be 4x4");
      }

      mat00 = scales.getX() * matrix.get(0, 0);
      mat01 = scales.getY() * matrix.get(0, 1);
      mat02 = scales.getZ() * matrix.get(0, 2);
      mat10 = scales.getX() * matrix.get(1, 0);
      mat11 = scales.getY() * matrix.get(1, 1);
      mat12 = scales.getZ() * matrix.get(1, 2);
      mat20 = scales.getX() * matrix.get(2, 0);
      mat21 = scales.getY() * matrix.get(2, 1);
      mat22 = scales.getZ() * matrix.get(2, 2);
      mat03 = vector.getX();
      mat13 = vector.getY();
      mat23 = vector.getZ();
   }

   /**
    * Set elements of this transform equal to the elements of matrix.
    * 
    * @param matrix
    */
   public final void set(DenseMatrix64F matrix, Vector3d vector, double scalex, double scaley, double scalez)
   {
      if (matrix.numCols != 3 && matrix.numRows != 3)
      {
         throw new RuntimeException("Incorrectly sized matrix. Matrix must be 4x4");
      }

      mat00 = scalex * matrix.get(0, 0);
      mat01 = scaley * matrix.get(0, 1);
      mat02 = scalez * matrix.get(0, 2);
      mat10 = scalex * matrix.get(1, 0);
      mat11 = scaley * matrix.get(1, 1);
      mat12 = scalez * matrix.get(1, 2);
      mat20 = scalex * matrix.get(2, 0);
      mat21 = scaley * matrix.get(2, 1);
      mat22 = scalez * matrix.get(2, 2);
      mat03 = vector.getX();
      mat13 = vector.getY();
      mat23 = vector.getZ();
   }

   /**
    * Computes the scale factors and returns the 
    * max of the three scale factors.
    * @return
    */
   public final double getScale()
   {
      computeScale();

      return max3(scale1, scale2, scale3);
   }

   /**
    * Computes and returns the scale.
    * 
    * @return
    */
   public final void getScale(Vector3d scaleToPack)
   {
      computeScale();

      scaleToPack.setX(scale1);
      scaleToPack.setY(scale2);
      scaleToPack.setZ(scale3);
   }

   /**
    * Computes and returns the scale.
    * 
    * @return
    */
   public final void getScale(Vector3f scaleToPack)
   {
      computeScale();

      scaleToPack.setX((float) scale1);
      scaleToPack.setY((float) scale2);
      scaleToPack.setZ((float) scale3);
   }

   /**
    * Set the scale factors of this transform.
    * @param x
    * @param y
    * @param z
    */
   public final void setScale(double x, double y, double z)
   {
      computeRotationScale();

      this.scale1 = x;
      this.scale2 = y;
      this.scale3 = z;

      mat00 = rot00 * scale1;
      mat01 = rot01 * scale2;
      mat02 = rot02 * scale3;
      mat10 = rot10 * scale1;
      mat11 = rot11 * scale2;
      mat12 = rot12 * scale3;
      mat20 = rot20 * scale1;
      mat21 = rot21 * scale2;
      mat22 = rot22 * scale3;
   }

   /**
    * Factors out the current scale and changes it to the one provided as an
    * agrument.
    * 
    * @param scale
    */
   public void setScale(double scale)
   {
      computeRotationScale();

      setScale(scale, scale, scale);
   }

   public void setScale(Vector3f scales)
   {
      setScale(scales.getX(), scales.getY(), scales.getZ());
   }

   public void setScale(Vector3d scales)
   {
      setScale(scales.getX(), scales.getY(), scales.getZ());
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
      computeRotationScale();
      vector.setX(Math.atan2(rot21, rot22));
      vector.setY(Math.atan2(-rot20, Math.sqrt(rot21 * rot21 + rot22 * rot22)));
      vector.setZ(Math.atan2(rot10, rot00));
   }

   /**
    * Return rotation matrix of type Matrix3d
    * 
    * @param matrix
    */
   @Override
   public final void getRotation(Matrix3d matrix)
   {
      computeRotationScale();

      matrix.setM00(rot00);
      matrix.setM01(rot01);
      matrix.setM02(rot02);
      matrix.setM10(rot10);
      matrix.setM11(rot11);
      matrix.setM12(rot12);
      matrix.setM20(rot20);
      matrix.setM21(rot21);
      matrix.setM22(rot22);
   }

   /**
    * Return scaled rotation matrix of type Matrix3d
    * 
    * @param matrix
    */
   public final void getRotationScale(Matrix3d matrix)
   {
      matrix.setM00(mat00);
      matrix.setM01(mat01);
      matrix.setM02(mat02);
      matrix.setM10(mat10);
      matrix.setM11(mat11);
      matrix.setM12(mat12);
      matrix.setM20(mat20);
      matrix.setM21(mat21);
      matrix.setM22(mat22);
   }

   /**
    * Return rotation matrix of type Matrix3f
    * 
    * @param matrix
    */
   @Override
   public final void getRotation(Matrix3f matrix)
   {
      computeRotationScale();

      matrix.setM00((float) rot00);
      matrix.setM01((float) rot01);
      matrix.setM02((float) rot02);
      matrix.setM10((float) rot10);
      matrix.setM11((float) rot11);
      matrix.setM12((float) rot12);
      matrix.setM20((float) rot20);
      matrix.setM21((float) rot21);
      matrix.setM22((float) rot22);
   }

   /**
    * Return scaled rotation matrix of type Matrix3f
    * 
    * @param matrix
    */
   public final void getRotationScale(Matrix3f matrix)
   {
      matrix.setM00((float) (mat00));
      matrix.setM01((float) (mat01));
      matrix.setM02((float) (mat02));
      matrix.setM10((float) (mat10));
      matrix.setM11((float) (mat11));
      matrix.setM12((float) (mat12));
      matrix.setM20((float) (mat20));
      matrix.setM21((float) (mat21));
      matrix.setM22((float) (mat22));
   }

   /**
    * Return rotation matrix of type DenseMatrix64F
    * 
    * @param matrix
    */
   @Override
   public final void getRotation(DenseMatrix64F matrix)
   {
      computeRotationScale();

      matrix.set(0, 0, rot00);
      matrix.set(0, 1, rot01);
      matrix.set(0, 2, rot02);
      matrix.set(1, 0, rot10);
      matrix.set(1, 1, rot11);
      matrix.set(1, 2, rot12);
      matrix.set(2, 0, rot20);
      matrix.set(2, 1, rot21);
      matrix.set(2, 2, rot22);
   }

   /**
    * Return rotation in quaternion form.
    * 
    * @param quat
    */
   @Override
   public final void getRotation(Quat4d quat)
   {
      computeRotationScale();

      double trace = rot00 + rot11 + rot22;
      double val;

      if (trace > 0.0)
      {
         val = Math.sqrt(trace + 1.0) * 2.0;
         quat.setX((rot21 - rot12) / val);
         quat.setY((rot02 - rot20) / val);
         quat.setZ((rot10 - rot01) / val);
         quat.setW(0.25 * val);
      }
      else if (rot11 > rot22)
      {
         val = Math.sqrt(1.0 + rot11 - rot00 - rot22) * 2.0;
         quat.setX((rot01 + rot10) / val);
         quat.setY(0.25 * val);
         quat.setZ((rot12 + rot21) / val);
         quat.setW((rot02 - rot20) / val);
      }
      else if ((rot00 > rot11) && (rot00 > rot22))
      {
         val = Math.sqrt(1.0 + rot00 - rot11 - rot22) * 2.0;
         quat.setX(0.25 * val);
         quat.setY((rot01 + rot10) / val);
         quat.setZ((rot02 + rot20) / val);
         quat.setW((rot21 - rot12) / val);
      }
      else
      {
         val = Math.sqrt(1.0 + rot22 - rot00 - rot11) * 2.0;
         quat.setX((rot02 + rot20) / val);
         quat.setY((rot12 + rot21) / val);
         quat.setZ(0.25 * val);
         quat.setW((rot10 - rot01) / val);
      }
   }

   /**
    * Return rotation in quaternion form.
    * 
    * @param quat
    */
   @Override
   public final void getRotation(Quat4f quat)
   {
      computeRotationScale();

      double trace = rot00 + rot11 + rot22;
      double val;
      if (trace > 0.0)
      {
         val = Math.sqrt(trace + 1.0) * 2.0;
         quat.setX((float) ((rot21 - rot12) / val));
         quat.setY((float) ((rot02 - rot20) / val));
         quat.setZ((float) ((rot10 - rot01) / val));
         quat.setW((float) (0.25 * val));
      }
      else if (rot11 > rot22)
      {
         val = Math.sqrt(1.0 + rot11 - rot00 - rot22) * 2.0;
         quat.setX((float) ((rot01 + rot10) / val));
         quat.setY((float) (0.25 * val));
         quat.setZ((float) ((rot12 + rot21) / val));
         quat.setW((float) ((rot02 - rot20) / val));
      }
      else if ((rot00 > rot11) && (rot00 > rot22))
      {
         val = Math.sqrt(1.0 + rot00 - rot11 - rot22) * 2.0;
         quat.setX((float) (0.25 * val));
         quat.setY((float) ((rot01 + rot10) / val));
         quat.setZ((float) ((rot02 + rot20) / val));
         quat.setW((float) ((rot21 - rot12) / val));
      }
      else
      {
         val = Math.sqrt(1.0 + rot22 - rot00 - rot11) * 2.0;
         quat.setX((float) ((rot02 + rot20) / val));
         quat.setY((float) ((rot12 + rot21) / val));
         quat.setZ((float) (0.25 * val));
         quat.setW((float) ((rot10 - rot01) / val));
      }
   }

   /**
    * Return rotation in AxisAngle form.
    * 
    * @param axisAngle
    */
   @Override
   public final void getRotation(AxisAngle4d axisAngle)
   {
      computeRotationScale();

      axisAngle.setX(rot21 - rot12);
      axisAngle.setY(rot02 - rot20);
      axisAngle.setZ(rot10 - rot01);
      double mag = axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ();

      if (mag > 1.0e-12)
      {
         mag = Math.sqrt(mag);
         double sin = 0.5 * mag;
         double cos = 0.5 * (rot00 + rot11 + rot22 - 1.0);

         axisAngle.setAngle(Math.atan2(sin, cos));

         double invMag = 1.0 / mag;
         axisAngle.setX(axisAngle.getX() * invMag);
         axisAngle.setY(axisAngle.getY() * invMag);
         axisAngle.setZ(axisAngle.getZ() * invMag);
      }
      else
      {
         axisAngle.setX(0.0);
         axisAngle.setY(1.0);
         axisAngle.setZ(0.0);
         axisAngle.setAngle(0.0);
      }
   }

   /**
    * Return rotation in AxisAngle form.
    * 
    * @param axisAngle
    */
   @Override
   public final void getRotation(AxisAngle4f axisAngle)
   {
      computeRotationScale();

      axisAngle.setX((float) (rot21 - rot12));
      axisAngle.setY((float) (rot02 - rot20));
      axisAngle.setZ((float) (rot10 - rot01));
      double mag = axisAngle.getX() * axisAngle.getX() + axisAngle.getY() * axisAngle.getY() + axisAngle.getZ() * axisAngle.getZ();

      if (mag > 1.0e-12)
      {
         mag = Math.sqrt(mag);
         double sin = 0.5 * mag;
         double cos = 0.5 * (rot00 + rot11 + rot22 - 1.0);

         axisAngle.setAngle((float) Math.atan2(sin, cos));

         double invMag = 1.0 / mag;
         axisAngle.setX((float) (axisAngle.getX() * invMag));
         axisAngle.setY((float) (axisAngle.getY() * invMag));
         axisAngle.setZ((float) (axisAngle.getZ() * invMag));
      }
      else
      {
         axisAngle.setX((float) 0.0);
         axisAngle.setY((float) 1.0);
         axisAngle.setZ((float) 0.0);
         axisAngle.setAngle((float) 0.0);
      }
   }

   /**
    * Pack rotation part into Matrix3d and translation part into Tuple3d
    * 
    * @param matrixToPack
    * @param translationToPack
    */
   @Override
   public final void get(Matrix3d matrixToPack, Tuple3d translationToPack)
   {
      getRotation(matrixToPack);
      getTranslation(translationToPack);
   }

   /**
    * Pack rotation part into Matrix3f and translation part into Tuple3f
    * 
    * @param matrixToPack
    * @param translationToPack
    */
   @Override
   public final void get(Matrix3f matrixToPack, Tuple3f translationToPack)
   {
      getRotation(matrixToPack);
      getTranslation(translationToPack);
   }

   /**
    * Convert and pack rotation part of transform into Quat4d and pack
    * translation into Tuple3d.
    * 
    * @param quaternionToPack
    * @param translationToPack
    */
   @Override
   public final void get(Quat4d quaternionToPack, Tuple3d translationToPack)
   {
      getRotation(quaternionToPack);
      getTranslation(translationToPack);
   }

   /**
    * Convert and pack rotation part of transform into Quat4f and pack
    * translation into Tuple3f.
    * 
    * @param quaternionToPack
    * @param translationToPack
    */
   @Override
   public final void get(Quat4f quaternionToPack, Tuple3f translationToPack)
   {
      getRotation(quaternionToPack);
      getTranslation(translationToPack);
   }

   /**
    * Compute the inverse of the transform passed in as an 
    * argument and store the result in this.
    * @param transform
    */
   public void invert(Transform3d transform)
   {
      if (transform == this)
      {
         invert();
      }
      else
      {
         transform.computeScale();

         if (Math.abs(transform.scale1 - 1.0) < 1e-6 && Math.abs(transform.scale2 - 1.0) < 1e-6 && Math.abs(transform.scale3 - 1.0) < 1e-6)
         {
            super.invert(transform);
         }
         else
         {
            invertAffine(transform);
         }
      }
   }

   /**
    * Compute the inverse of this, first checking if the scales are near unity 
    * so an efficient inverse can be used.
    */
   @Override
   public void invert()
   {
      computeScale();

      if (Math.abs(scale1 - 1.0) < 1e-6 && Math.abs(scale2 - 1.0) < 1e-6 && Math.abs(scale3 - 1.0) < 1e-6)
      {
         super.invert();
      }
      else
      {
         invertAffine();
      }
   }

   /**
    * Invert this without assuming the rotation portion is orthogonal.
    */
   private final void invertAffine(Transform3d transform)
   {
      double det = transform.determinantRotationPart();
      if (det == 0)
      {
         throw new RuntimeException("Matrix is singular.");
      }

      double val = (transform.mat00 * transform.mat00 + transform.mat01 * transform.mat01 + transform.mat02 * transform.mat02 + transform.mat03
            * transform.mat03)
            * (transform.mat10 * transform.mat10 + transform.mat11 * transform.mat11 + transform.mat12 * transform.mat12 + transform.mat13 * transform.mat13)
            * (transform.mat20 * transform.mat20 + transform.mat21 * transform.mat21 + transform.mat22 * transform.mat22 + transform.mat23 * transform.mat23);

      if ((det * det) >= (1.110223024E-16 * val))
      {
         mat00 = (transform.mat11 * transform.mat22 - transform.mat21 * transform.mat12) / det;
         mat01 = -(transform.mat01 * transform.mat22 - transform.mat21 * transform.mat02) / det;
         mat02 = (transform.mat01 * transform.mat12 - transform.mat11 * transform.mat02) / det;
         mat10 = -(transform.mat10 * transform.mat22 - transform.mat20 * transform.mat12) / det;
         mat11 = (transform.mat00 * transform.mat22 - transform.mat20 * transform.mat02) / det;
         mat12 = -(transform.mat00 * transform.mat12 - transform.mat10 * transform.mat02) / det;
         mat20 = (transform.mat10 * transform.mat21 - transform.mat20 * transform.mat11) / det;
         mat21 = -(transform.mat00 * transform.mat21 - transform.mat20 * transform.mat01) / det;
         mat22 = (transform.mat00 * transform.mat11 - transform.mat10 * transform.mat01) / det;
         mat03 = -(transform.mat03 * mat00 + transform.mat13 * mat01 + transform.mat23 * mat02);
         mat13 = -(transform.mat03 * mat10 + transform.mat13 * mat11 + transform.mat23 * mat12);
         mat23 = -(transform.mat03 * mat20 + transform.mat13 * mat21 + transform.mat23 * mat22);
      }
      else
      {
         tmp[0] = transform.mat00;
         tmp[1] = transform.mat01;
         tmp[2] = transform.mat02;
         tmp[3] = transform.mat03;
         tmp[4] = transform.mat10;
         tmp[5] = transform.mat11;
         tmp[6] = transform.mat12;
         tmp[7] = transform.mat13;
         tmp[8] = transform.mat20;
         tmp[9] = transform.mat21;
         tmp[10] = transform.mat22;
         tmp[11] = transform.mat23;
         tmp[12] = 0.0;
         tmp[13] = 0.0;
         tmp[14] = 0.0;
         tmp[15] = 1.0;

         // Calculate LU decomposition: Is the matrix singular?
         if (!luDecomposition(tmp, row_perm))
         {
            // Matrix has no inverse
            throw new RuntimeException("Matrix is singular");
         }

         // Perform back substitution on the identity matrix
         // luDecomposition will set rot[] & scales[] for use
         // in luBacksubstituation
         tmpMat[0] = 1.0;
         tmpMat[1] = 0.0;
         tmpMat[2] = 0.0;
         tmpMat[3] = 0.0;
         tmpMat[4] = 0.0;
         tmpMat[5] = 1.0;
         tmpMat[6] = 0.0;
         tmpMat[7] = 0.0;
         tmpMat[8] = 0.0;
         tmpMat[9] = 0.0;
         tmpMat[10] = 1.0;
         tmpMat[11] = 0.0;

         luBacksubstitution(tmp, row_perm, tmpMat);

         mat00 = tmpMat[0];
         mat01 = tmpMat[1];
         mat02 = tmpMat[2];
         mat03 = tmpMat[3];
         mat10 = tmpMat[4];
         mat11 = tmpMat[5];
         mat12 = tmpMat[6];
         mat13 = tmpMat[7];
         mat20 = tmpMat[8];
         mat21 = tmpMat[9];
         mat22 = tmpMat[10];
         mat23 = tmpMat[11];
      }
   }

   /**
    * Invert this without assuming the rotation portion is orthogonal.
    */
   private final void invertAffine()
   {
      double det = this.determinantRotationPart();
      if (det == 0.0)
      {
         throw new RuntimeException("Matrix is singular.");
      }

      double val = (mat00 * mat00 + mat01 * mat01 + mat02 * mat02 + mat03 * mat03) * (mat10 * mat10 + mat11 * mat11 + mat12 * mat12 + mat13 * mat13)
            * (mat20 * mat20 + mat21 * mat21 + mat22 * mat22 + mat23 * mat23);

      if ((det * det) >= (1.110223024E-16 * val))
      {
         double tmp0 = (mat11 * mat22 - mat21 * mat12) / det;
         double tmp1 = -(mat01 * mat22 - mat21 * mat02) / det;
         double tmp2 = (mat01 * mat12 - mat11 * mat02) / det;
         double tmp4 = -(mat10 * mat22 - mat20 * mat12) / det;
         double tmp5 = (mat00 * mat22 - mat20 * mat02) / det;
         double tmp6 = -(mat00 * mat12 - mat10 * mat02) / det;
         double tmp8 = (mat10 * mat21 - mat20 * mat11) / det;
         double tmp9 = -(mat00 * mat21 - mat20 * mat01) / det;
         double tmp10 = (mat00 * mat11 - mat10 * mat01) / det;
         double tmp3 = -(mat03 * tmp0 + mat13 * tmp1 + mat23 * tmp2);
         double tmp7 = -(mat03 * tmp4 + mat13 * tmp5 + mat23 * tmp6);
         mat23 = -(mat03 * tmp8 + mat13 * tmp9 + mat23 * tmp10);

         mat00 = tmp0;
         mat01 = tmp1;
         mat02 = tmp2;
         mat03 = tmp3;
         mat10 = tmp4;
         mat11 = tmp5;
         mat12 = tmp6;
         mat13 = tmp7;
         mat20 = tmp8;
         mat21 = tmp9;
         mat22 = tmp10;
      }
      else
      {
         tmp[0] = mat00;
         tmp[1] = mat01;
         tmp[2] = mat02;
         tmp[3] = mat03;
         tmp[4] = mat10;
         tmp[5] = mat11;
         tmp[6] = mat12;
         tmp[7] = mat13;
         tmp[8] = mat20;
         tmp[9] = mat21;
         tmp[10] = mat22;
         tmp[11] = mat23;
         tmp[12] = 0.0;
         tmp[13] = 0.0;
         tmp[14] = 0.0;
         tmp[15] = 1.0;

         // Calculate LU decomposition: Is the matrix singular?
         if (!luDecomposition(tmp, row_perm))
         {
            // Matrix has no inverse
            throw new RuntimeException("Matrix is singular");
         }

         // Perform back substitution on the identity matrix
         // luDecomposition will set rot[] & scales[] for use
         // in luBacksubstituation
         tmpMat[0] = 1.0;
         tmpMat[1] = 0.0;
         tmpMat[2] = 0.0;
         tmpMat[3] = 0.0;
         tmpMat[4] = 0.0;
         tmpMat[5] = 1.0;
         tmpMat[6] = 0.0;
         tmpMat[7] = 0.0;
         tmpMat[8] = 0.0;
         tmpMat[9] = 0.0;
         tmpMat[10] = 1.0;
         tmpMat[11] = 0.0;

         luBacksubstitution(tmp, row_perm, tmpMat);

         mat00 = tmpMat[0];
         mat01 = tmpMat[1];
         mat02 = tmpMat[2];
         mat03 = tmpMat[3];
         mat10 = tmpMat[4];
         mat11 = tmpMat[5];
         mat12 = tmpMat[6];
         mat13 = tmpMat[7];
         mat20 = tmpMat[8];
         mat21 = tmpMat[9];
         mat22 = tmpMat[10];
         mat23 = tmpMat[11];
      }
   }

   //
   // Reference: Press, Flannery, Teukolsky, Vetterling,
   // _Numerical_Recipes_in_C_, Cambridge University Press,
   // 1988, pp 44-45.
   //
   private void luBacksubstitution(double[] matrix1, int[] row_perm, double[] matrix2)
   {
      int i, ii, ip, j;
      int cv;

      // For each column vector of matrix2 ...
      for (cv = 0; cv < 4; cv++)
      {
         ii = -1;

         // Forward substitution
         for (i = 0; i < 3; i++)
         {
            double sum;

            ip = row_perm[i];
            sum = matrix2[cv + 4 * ip];
            matrix2[cv + 4 * ip] = matrix2[cv + 4 * i];
            if (ii >= 0)
            {
               for (j = ii; j <= i - 1; j++)
               {
                  sum -= matrix1[4 * i + j] * matrix2[cv + 4 * j];
               }
            }
            else if (sum != 0.0)
            {
               ii = i;
            }
            matrix2[cv + 4 * i] = sum;
         }

         matrix2[cv + 8] = (matrix2[cv + 8]) / matrix1[10];
         matrix2[cv + 4] = (matrix2[cv + 4] - matrix1[6] * matrix2[cv + 8]) / matrix1[5];
         matrix2[cv] = (matrix2[cv] - matrix1[1] * matrix2[cv + 4] - matrix1[2] * matrix2[cv + 8]) / matrix1[0];

      }

      matrix2[11] = (matrix2[11] - matrix1[11]) / matrix1[10];
      matrix2[7] = (matrix2[7] - matrix1[6] * matrix2[11] - matrix1[7]) / matrix1[5];
      matrix2[3] = (matrix2[3] - matrix1[1] * matrix2[7] - matrix1[2] * matrix2[11] - matrix1[3]) / matrix1[0];
   }

   // Reference: Press, Flannery, Teukolsky, Vetterling,
   // _Numerical_Recipes_in_C_, Cambridge University Press,
   // 1988, pp 40-45.
   //
   private boolean luDecomposition(double[] matrix0, int[] row_perm)
   {

      // Determine implicit scaling information by looping over rows
      int i, j;
      int ptr, rs;
      double big, temp;

      ptr = 0;
      rs = 0;

      // For each row ...
      i = 3;
      while (i-- != 0)
      {
         big = 0.0;

         // For each column, find the largest element in the row
         j = 4;
         while (j-- != 0)
         {
            temp = matrix0[ptr++];
            temp = Math.abs(temp);
            if (temp > big)
            {
               big = temp;
            }
         }

         // Is the matrix singular?
         if (big == 0.0)
         {
            return false;
         }
         row_scale[rs++] = 1.0 / big;
      }
      row_scale[3] = 1.0;

      int imax, k;
      int target, p1, p2;
      double sum;
      // For all columns, execute Crout's method
      for (j = 0; j < 4; j++)
      {

         // Determine elements of upper diagonal matrix U
         for (i = 0; i < j; i++)
         {
            target = (4 * i) + j;
            sum = matrix0[target];

            k = i;
            p1 = (4 * i);
            p2 = j;
            while (k-- != 0)
            {
               sum -= matrix0[p1] * matrix0[p2];
               p1++;
               p2 += 4;
            }
            matrix0[target] = sum;
         }

         big = 0.0;
         imax = -1;
         for (i = j; i < 4; i++)
         {
            target = (4 * i) + j;
            sum = matrix0[target];
            k = j;
            p1 = (4 * i);
            p2 = j;
            while (k-- != 0)
            {
               sum -= matrix0[p1] * matrix0[p2];
               p1++;
               p2 += 4;
            }
            matrix0[target] = sum;

            // Is this the best pivot so far?
            if ((temp = row_scale[i] * Math.abs(sum)) >= big)
            {
               big = temp;
               imax = i;
            }
         }

         if (imax < 0)
         {
            return false;
         }

         // Is a row exchange necessary?
         if (j != imax)
         {
            // Yes: exchange rows
            k = 4;
            p1 = (4 * imax);
            p2 = (4 * j);
            while (k-- != 0)
            {
               temp = matrix0[p1];
               matrix0[p1++] = matrix0[p2];
               matrix0[p2++] = temp;
            }

            // Record change in scale factor
            row_scale[imax] = row_scale[j];
         }

         // Record row permutation
         row_perm[j] = imax;

         // Is the matrix singular
         if (matrix0[((4 * j) + j)] == 0.0)
         {
            return false;
         }

         // Divide elements of lower diagonal matrix L by pivot
         if (j != (4 - 1))
         {
            target = (4 * (j + 1)) + j;
            i = 3 - j;
            while (i-- != 0)
            {
               matrix0[target] *= 1.0 / (matrix0[((4 * j) + j)]);
               target += 4;
            }
         }
      }

      return true;
   }

   /**
    * Create Transform3d with zero translation and the rotation matrix being a
    * rotation about the x-axis by angle. Sets scale factors to unity.
    * 
    * @param angle
    */
   @Override
   public final void setRotationRollAndZeroTranslation(double angle)
   {
      mat00 = 1.0;
      mat01 = 0.0;
      mat02 = 0.0;
      mat03 = 0.0;
      mat10 = 0.0;
      mat11 = Math.cos(angle);
      mat12 = -Math.sin(angle);
      mat13 = 0.0;
      mat20 = 0.0;
      mat21 = Math.sin(angle);
      mat22 = Math.cos(angle);
      mat23 = 0.0;

      scale1 = 1.0;
      scale2 = 1.0;
      scale3 = 1.0;
   }

   /**
    * Create Transform3d with zero translation and the rotation matrix being a
    * rotation about the y-axis by angle. Sets scale factors to unity.
    * 
    * @param angle
    */
   @Override
   public final void setRotationPitchAndZeroTranslation(double angle)
   {
      mat00 = Math.cos(angle);
      mat01 = 0.0;
      mat02 = Math.sin(angle);
      mat03 = 0.0;
      mat10 = 0.0;
      mat11 = 1.0;
      mat12 = 0.0;
      mat13 = 0.0;
      mat20 = -Math.sin(angle);
      mat21 = 0.0;
      mat22 = Math.cos(angle);
      mat23 = 0.0;

      scale1 = 1.0;
      scale2 = 1.0;
      scale3 = 1.0;
   }

   /**
    * Create Transform3d with zero translation and the rotation matrix being a
    * rotation about the z-axis by angle. Sets scale factors to unity.
    * 
    * @param angle
    */
   @Override
   public final void setRotationYawAndZeroTranslation(double angle)
   {
      mat00 = Math.cos(angle);
      mat01 = -Math.sin(angle);
      mat02 = 0.0;
      mat03 = 0.0;
      mat10 = Math.sin(angle);
      mat11 = Math.cos(angle);
      mat12 = 0.0;
      mat13 = 0.0;
      mat20 = 0.0;
      mat21 = 0.0;
      mat22 = 1.0;
      mat23 = 0.0;

      scale1 = 1.0;
      scale2 = 1.0;
      scale3 = 1.0;
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
   public boolean equals(Object object1)
   {
      return (object1 instanceof Transform3d) && equals((Transform3d) object1);
   }

   /**
    * Orthonormalization of the rotation matrix using Gram-Schmidt method.
    */
   @Override
   public final void normalizeRotationPart()
   {

      computeRotationScale();
      computeScale();

      double xdoty = rot00 * rot01 + rot10 * rot11 + rot20 * rot21;
      double xdotx = rot00 * rot00 + rot10 * rot10 + rot20 * rot20;
      double tmp = xdoty / xdotx;

      rot01 -= tmp * rot00;
      rot11 -= tmp * rot10;
      rot21 -= tmp * rot20;

      double zdoty = rot02 * rot01 + rot12 * rot11 + rot22 * rot21;
      double zdotx = rot02 * rot00 + rot12 * rot10 + rot22 * rot20;
      double ydoty = rot01 * rot01 + rot11 * rot11 + rot21 * rot21;

      tmp = zdotx / xdotx;
      double tmp1 = zdoty / ydoty;

      rot02 = rot02 - (tmp * rot00 + tmp1 * rot01);
      rot12 = rot12 - (tmp * rot10 + tmp1 * rot11);
      rot22 = rot22 - (tmp * rot20 + tmp1 * rot21);

      // Compute orthogonalized vector magnitudes and normalize
      double magX = Math.sqrt(rot00 * rot00 + rot10 * rot10 + rot20 * rot20);
      double magY = Math.sqrt(rot01 * rot01 + rot11 * rot11 + rot21 * rot21);
      double magZ = Math.sqrt(rot02 * rot02 + rot12 * rot12 + rot22 * rot22);

      rot00 = rot00 / magX;
      rot10 = rot10 / magX;
      rot20 = rot20 / magX;
      rot01 = rot01 / magY;
      rot11 = rot11 / magY;
      rot21 = rot21 / magY;
      rot02 = rot02 / magZ;
      rot12 = rot12 / magZ;
      rot22 = rot22 / magZ;

      mat00 = rot00 * scale1;
      mat01 = rot01 * scale2;
      mat02 = rot02 * scale3;
      mat10 = rot10 * scale1;
      mat11 = rot11 * scale2;
      mat12 = rot12 * scale3;
      mat20 = rot20 * scale1;
      mat21 = rot21 * scale2;
      mat22 = rot22 * scale3;
   }

   /**
    * Computes the scale frome the upper left 3x3 of this transform.
    */
   private final void computeScale()
   {
      scale1 = Math.sqrt(mat00 * mat00 + mat10 * mat10 + mat20 * mat20);
      scale2 = Math.sqrt(mat01 * mat01 + mat11 * mat11 + mat21 * mat21);
      scale3 = Math.sqrt(mat02 * mat02 + mat12 * mat12 + mat22 * mat22);

      if (scale1 == 0 && scale2 == 0 && scale3 == 0)
         compute_svd();
   }

   /**
    * Computes the rotation matrix by factoring out the scale by rot_ij =
    * mat_ij/scale
    */
   private final void computeRotationScale()
   {
      double tmp;

      scale1 = Math.sqrt(mat00 * mat00 + mat10 * mat10 + mat20 * mat20);
      scale2 = Math.sqrt(mat01 * mat01 + mat11 * mat11 + mat21 * mat21);
      scale3 = Math.sqrt(mat02 * mat02 + mat12 * mat12 + mat22 * mat22);

      if ((scale1 == 0) || (scale2 == 0) || (scale3 == 0))
      {
         compute_svd();
         return;
      }
      tmp = 1.0 / scale1;
      rot00 = mat00 * tmp;
      rot10 = mat10 * tmp;
      rot20 = mat20 * tmp;
      tmp = 1.0 / scale2;
      rot01 = mat01 * tmp;
      rot11 = mat11 * tmp;
      rot21 = mat21 * tmp;
      tmp = 1.0 / scale3;
      rot02 = mat02 * tmp;
      rot12 = mat12 * tmp;
      rot22 = mat22 * tmp;

      rot00 = mat00 / this.scale1;
      rot01 = mat01 / this.scale2;
      rot02 = mat02 / this.scale3;
      rot10 = mat10 / this.scale1;
      rot11 = mat11 / this.scale2;
      rot12 = mat12 / this.scale3;
      rot20 = mat20 / this.scale1;
      rot21 = mat21 / this.scale2;
      rot22 = mat22 / this.scale3;
   }

   private void compute_svd()
   {

      int i;
      double g;
      double m0, m1, m2, m3, m4, m5, m6, m7, m8 = 0.0;
      double u0, u1, u2, u3, u4, u5, u6, u7, u8 = 0.0;
      double v0, v1, v2, v3, v4, v5, v6, v7, v8 = 0.0;

      double e1, e2 = 0.0;
      int negCnt = 0;
      double c1, c2, c3, c4;
      double s1, s2, s3, s4;
      int svdOut0, svdOut1, svdOut2;

      svdRot[0] = m0 = mat00;
      svdRot[1] = m1 = mat01;
      svdRot[2] = m2 = mat02;
      svdRot[3] = m3 = mat10;
      svdRot[4] = m4 = mat11;
      svdRot[5] = m5 = mat12;
      svdRot[6] = m6 = mat20;
      svdRot[7] = m7 = mat21;
      svdRot[8] = m8 = mat22;

      // u1

      if (m3 * m3 < 1.110223024E-16)
      {
         u0 = 1.0;
         u1 = 0.0;
         u2 = 0.0;
         u3 = 0.0;
         u4 = 1.0;
         u5 = 0.0;
         u6 = 0.0;
         u7 = 0.0;
         u8 = 1.0;
      }
      else if (m0 * m0 < 1.110223024E-16)
      {
         t1[0] = m0;
         t1[1] = m1;
         t1[2] = m2;
         m0 = m3;
         m1 = m4;
         m2 = m5;

         m3 = -t1[0]; // zero
         m4 = -t1[1];
         m5 = -t1[2];

         u0 = 0.0;
         u1 = 1.0;
         u2 = 0.0;
         u3 = -1.0;
         u4 = 0.0;
         u5 = 0.0;
         u6 = 0.0;
         u7 = 0.0;
         u8 = 1.0;
      }
      else
      {
         g = 1.0 / Math.sqrt(m0 * m0 + m3 * m3);
         c1 = m0 * g;
         s1 = m3 * g;
         t1[0] = c1 * m0 + s1 * m3;
         t1[1] = c1 * m1 + s1 * m4;
         t1[2] = c1 * m2 + s1 * m5;

         m3 = -s1 * m0 + c1 * m3; // zero
         m4 = -s1 * m1 + c1 * m4;
         m5 = -s1 * m2 + c1 * m5;

         m0 = t1[0];
         m1 = t1[1];
         m2 = t1[2];
         u0 = c1;
         u1 = s1;
         u2 = 0.0;
         u3 = -s1;
         u4 = c1;
         u5 = 0.0;
         u6 = 0.0;
         u7 = 0.0;
         u8 = 1.0;
      }

      // u2

      if (m6 * m6 < 1.110223024E-16)
      {
      }
      else if (m0 * m0 < 1.110223024E-16)
      {
         t1[0] = m0;
         t1[1] = m1;
         t1[2] = m2;
         m0 = m6;
         m1 = m7;
         m2 = m8;

         m6 = -t1[0]; // zero
         m7 = -t1[1];
         m8 = -t1[2];

         t1[0] = u0;
         t1[1] = u1;
         t1[2] = u2;
         u0 = u6;
         u1 = u7;
         u2 = u8;

         u6 = -t1[0]; // zero
         u7 = -t1[1];
         u8 = -t1[2];
      }
      else
      {
         g = 1.0 / Math.sqrt(m0 * m0 + m6 * m6);
         c2 = m0 * g;
         s2 = m6 * g;
         t1[0] = c2 * m0 + s2 * m6;
         t1[1] = c2 * m1 + s2 * m7;
         t1[2] = c2 * m2 + s2 * m8;

         m6 = -s2 * m0 + c2 * m6;
         m7 = -s2 * m1 + c2 * m7;
         m8 = -s2 * m2 + c2 * m8;
         m0 = t1[0];
         m1 = t1[1];
         m2 = t1[2];

         t1[0] = c2 * u0;
         t1[1] = c2 * u1;
         u2 = s2;

         t1[6] = -u0 * s2;
         t1[7] = -u1 * s2;
         u8 = c2;
         u0 = t1[0];
         u1 = t1[1];
         u6 = t1[6];
         u7 = t1[7];
      }

      // v1

      if (m2 * m2 < 1.110223024E-16)
      {
         v0 = 1.0;
         v1 = 0.0;
         v2 = 0.0;
         v3 = 0.0;
         v4 = 1.0;
         v5 = 0.0;
         v6 = 0.0;
         v7 = 0.0;
         v8 = 1.0;
      }
      else if (m1 * m1 < 1.110223024E-16)
      {
         t1[2] = m2;
         t1[5] = m5;
         t1[8] = m8;
         m2 = -m1;
         m5 = -m4;
         m8 = -m7;

         m1 = t1[2]; // zero
         m4 = t1[5];
         m7 = t1[8];

         v0 = 1.0;
         v1 = 0.0;
         v2 = 0.0;
         v3 = 0.0;
         v4 = 0.0;
         v5 = -1.0;
         v6 = 0.0;
         v7 = 1.0;
         v8 = 0.0;
      }
      else
      {
         g = 1.0 / Math.sqrt(m1 * m1 + m2 * m2);
         c3 = m1 * g;
         s3 = m2 * g;
         t1[1] = c3 * m1 + s3 * m2; // can assign to m1?
         m2 = -s3 * m1 + c3 * m2; // zero
         m1 = t1[1];

         t1[4] = c3 * m4 + s3 * m5;
         m5 = -s3 * m4 + c3 * m5;
         m4 = t1[4];

         t1[7] = c3 * m7 + s3 * m8;
         m8 = -s3 * m7 + c3 * m8;
         m7 = t1[7];

         v0 = 1.0;
         v1 = 0.0;
         v2 = 0.0;
         v3 = 0.0;
         v4 = c3;
         v5 = -s3;
         v6 = 0.0;
         v7 = s3;
         v8 = c3;
      }

      // u3

      if (m7 * m7 < 1.110223024E-16)
      {
      }
      else if (m4 * m4 < 1.110223024E-16)
      {
         t1[3] = m3;
         t1[4] = m4;
         t1[5] = m5;
         m3 = m6; // zero
         m4 = m7;
         m5 = m8;

         m6 = -t1[3]; // zero
         m7 = -t1[4]; // zero
         m8 = -t1[5];

         t1[3] = u3;
         t1[4] = u4;
         t1[5] = u5;
         u3 = u6;
         u4 = u7;
         u5 = u8;

         u6 = -t1[3]; // zero
         u7 = -t1[4];
         u8 = -t1[5];

      }
      else
      {
         g = 1.0 / Math.sqrt(m4 * m4 + m7 * m7);
         c4 = m4 * g;
         s4 = m7 * g;
         t1[3] = c4 * m3 + s4 * m6;
         m6 = -s4 * m3 + c4 * m6; // zero
         m3 = t1[3];

         t1[4] = c4 * m4 + s4 * m7;
         m7 = -s4 * m4 + c4 * m7;
         m4 = t1[4];

         t1[5] = c4 * m5 + s4 * m8;
         m8 = -s4 * m5 + c4 * m8;
         m5 = t1[5];

         t1[3] = c4 * u3 + s4 * u6;
         u6 = -s4 * u3 + c4 * u6;
         u3 = t1[3];

         t1[4] = c4 * u4 + s4 * u7;
         u7 = -s4 * u4 + c4 * u7;
         u4 = t1[4];

         t1[5] = c4 * u5 + s4 * u8;
         u8 = -s4 * u5 + c4 * u8;
         u5 = t1[5];
      }

      t2[0] = m0;
      t2[1] = m4;
      t2[2] = m8;
      e1 = m1;
      e2 = m5;

      if (e1 * e1 > 1.110223024E-16 || e2 * e2 > 1.110223024E-16)
      {
         compute_qr(t2, e1, e2, u0, u1, u2, u3, u4, u5, u6, u7, u8, v0, v1, v2, v3, v4, v5, v6, v7, v8);
      }

      svdScales[0] = t2[0];
      svdScales[1] = t2[1];
      svdScales[2] = t2[2];

      // Do some optimization here. If scale is unity, simply return the
      // rotation matric.
      if (almostOne(Math.abs(svdScales[0])) && almostOne(Math.abs(svdScales[1])) && almostOne(Math.abs(svdScales[2])))
      {

         for (i = 0; i < 3; i++)
            if (svdScales[i] < 0.0)
               negCnt++;

         if ((negCnt == 0) || (negCnt == 2))
         {
            // System.err.println("Optimize!!");
            scale1 = scale2 = scale3 = 1.0;

            rot00 = svdRot[0];
            rot01 = svdRot[1];
            rot02 = svdRot[2];
            rot10 = svdRot[3];
            rot11 = svdRot[4];
            rot12 = svdRot[5];
            rot20 = svdRot[6];
            rot21 = svdRot[7];
            rot22 = svdRot[8];

            return;
         }
      }

      transpose_mat(u0, u1, u2, u3, u4, u5, u6, u7, u8, t1);
      transpose_mat(v0, v1, v2, v3, v4, v5, v6, v7, v8, t2);

      // svdReorder(m0,m1,m2,m3,m4,m5,m6,m7,m8, t1, t2, svdRot, svdScales);
      int in0, in1, in2;
      double svdMag0 = 0;
      double svdMag1 = 0;
      double svdMag2 = 0;

      // check for rotation information in the scales
      if (svdScales[0] < 0.0)
      { // move the rotation info to rotation matrix
         svdScales[0] = -svdScales[0];
         t2[0] = -t2[0];
         t2[1] = -t2[1];
         t2[2] = -t2[2];
      }
      if (svdScales[1] < 0.0)
      { // move the rotation info to rotation matrix
         svdScales[1] = -svdScales[1];
         t2[3] = -t2[3];
         t2[4] = -t2[4];
         t2[5] = -t2[5];
      }
      if (svdScales[2] < 0.0)
      { // move the rotation info to rotation matrix
         svdScales[2] = -svdScales[2];
         t2[6] = -t2[6];
         t2[7] = -t2[7];
         t2[8] = -t2[8];
      }

      mat_mul(t1, t2, svdRot);

      // check for equal scales case and do not reorder
      if (almostEqual(Math.abs(svdScales[0]), Math.abs(svdScales[1])) && almostEqual(Math.abs(svdScales[1]), Math.abs(svdScales[2])))
      {
         rot00 = svdRot[0];
         rot01 = svdRot[1];
         rot02 = svdRot[2];
         rot10 = svdRot[3];
         rot11 = svdRot[4];
         rot12 = svdRot[5];
         rot20 = svdRot[6];
         rot21 = svdRot[7];
         rot22 = svdRot[8];

         scale1 = svdScales[0];
         scale2 = svdScales[1];
         scale3 = svdScales[2];

      }
      else
      {

         // sort the order of the results of SVD
         if (svdScales[0] > svdScales[1])
         {
            if (svdScales[0] > svdScales[2])
            {
               if (svdScales[2] > svdScales[1])
               {
                  svdOut0 = 0;
                  svdOut1 = 2;
                  svdOut2 = 1;
               }
               else
               {
                  svdOut0 = 0;
                  svdOut1 = 1;
                  svdOut2 = 2;
               }
            }
            else
            {
               svdOut0 = 2;
               svdOut1 = 0;
               svdOut2 = 1;
            }
         }
         else
         { // y > x
            if (svdScales[1] > svdScales[2])
            {
               if (svdScales[2] > svdScales[0])
               {
                  svdOut0 = 1;
                  svdOut1 = 2;
                  svdOut2 = 0;
               }
               else
               {
                  svdOut0 = 1;
                  svdOut1 = 0;
                  svdOut2 = 2;
               }
            }
            else
            {
               svdOut0 = 2;
               svdOut1 = 1;
               svdOut2 = 0;
            }
         }

         // sort the order of the input matrix
         svdMag0 = (m0 * m0 + m1 * m1 + m2 * m2);
         svdMag1 = (m3 * m3 + m4 * m4 + m5 * m5);
         svdMag2 = (m6 * m6 + m7 * m7 + m8 * m8);

         if (svdMag0 > svdMag1)
         {
            if (svdMag0 > svdMag2)
            {
               if (svdMag2 > svdMag1)
               {
                  in0 = svdOut0;
                  in2 = svdOut1;
                  in1 = svdOut2;// xzy
               }
               else
               {
                  in0 = svdOut0;
                  in1 = svdOut1;
                  in2 = svdOut2; // xyz
               }
            }
            else
            {
               in2 = svdOut0;
               in0 = svdOut1;
               in1 = svdOut2; // zxy
            }
         }
         else
         {
            if (svdMag1 > svdMag2)
            {
               if (svdMag2 > svdMag0)
               {
                  in1 = svdOut0;
                  in2 = svdOut1;
                  in0 = svdOut2;
               }
               else
               {
                  in1 = svdOut0;
                  in0 = svdOut1;
                  in2 = svdOut2;
               }
            }
            else
            {
               in2 = svdOut0;
               in1 = svdOut1;
               in0 = svdOut2;
            }
         }

         scale1 = svdScales[in0];
         scale2 = svdScales[in1];
         scale3 = svdScales[in2];
         rot00 = svdRot[in0];
         rot10 = svdRot[in0 + 3];
         rot20 = svdRot[in0 + 6];
         rot01 = svdRot[in1];
         rot11 = svdRot[in1 + 3];
         rot21 = svdRot[in1 + 6];
         rot02 = svdRot[in2];
         rot12 = svdRot[in2 + 3];
         rot22 = svdRot[in2 + 6];
      }
   }

   private int compute_qr(double[] s, double e1, double e2, double u0, double u1, double u2, double u3, double u4, double u5, double u6, double u7, double u8,
         double v0, double v1, double v2, double v3, double v4, double v5, double v6, double v7, double v8)
   {
      int k;
      boolean converged;
      double shift, r;

      double utemp, vtemp;
      double f, g;

      final int MAX_INTERATIONS = 10;
      final double CONVERGE_TOL = 4.89E-15;
      Double sinr0, sinl0, cosr0, cosl0;
      Double sinr1, sinl1, cosr1, cosl1;
      sinr0 = sinl0 = cosr0 = cosl0 = 0.0;
      sinr1 = sinl1 = cosr1 = cosl1 = 0.0;

      double c_b48 = 1.;
      converged = false;

      if (Math.abs(e2) < CONVERGE_TOL || Math.abs(e1) < CONVERGE_TOL)
         converged = true;

      for (k = 0; k < MAX_INTERATIONS && !converged; k++)
      {
         shift = compute_shift(s[1], e2, s[2]);
         f = (Math.abs(s[0]) - shift) * (d_sign(c_b48, s[0]) + shift / s[0]);
         g = e1;
         r = compute_rot(f, g, sinr0, cosr0);
         f = cosr0 * s[0] + sinr0 * e1;
         e1 = cosr0 * e1 - sinr0 * s[0];
         g = sinr0 * s[1];
         s[1] = cosr0 * s[1];

         r = compute_rot(f, g, sinl0, cosl0);
         s[0] = r;
         f = cosl0 * e1 + sinl0 * s[1];
         s[1] = cosl0 * s[1] - sinl0 * e1;
         g = sinl0 * e2;
         e2 = cosl0 * e2;

         r = compute_rot(f, g, sinr1, cosr1);
         e1 = r;
         f = cosr1 * s[1] + sinr1 * e2;
         e2 = cosr1 * e2 - sinr1 * s[1];
         g = sinr1 * s[2];
         s[2] = cosr1 * s[2];

         r = compute_rot(f, g, sinl1, cosl1);
         s[1] = r;
         f = cosl1 * e2 + sinl1 * s[2];
         s[2] = cosl1 * s[2] - sinl1 * e2;
         e2 = f;

         // update u matrices
         utemp = u0;
         u0 = cosl0 * utemp + sinl0 * u3;
         u3 = -sinl0 * utemp + cosl0 * u3;
         utemp = u1;
         u1 = cosl0 * utemp + sinl0 * u4;
         u4 = -sinl0 * utemp + cosl0 * u4;
         utemp = u2;
         u2 = cosl0 * utemp + sinl0 * u5;
         u5 = -sinl0 * utemp + cosl0 * u5;

         utemp = u3;
         u3 = cosl1 * utemp + sinl1 * u6;
         u6 = -sinl1 * utemp + cosl1 * u6;
         utemp = u4;
         u4 = cosl1 * utemp + sinl1 * u7;
         u7 = -sinl1 * utemp + cosl1 * u7;
         utemp = u5;
         u5 = cosl1 * utemp + sinl1 * u8;
         u8 = -sinl1 * utemp + cosl1 * u8;

         vtemp = v0;
         v0 = cosr0 * vtemp + sinr0 * v1;
         v1 = -sinr0 * vtemp + cosr0 * v1;
         vtemp = v3;
         v3 = cosr0 * vtemp + sinr0 * v4;
         v4 = -sinr0 * vtemp + cosr0 * v4;
         vtemp = v6;
         v6 = cosr0 * vtemp + sinr0 * v7;
         v7 = -sinr0 * vtemp + cosr0 * v7;

         vtemp = v1;
         v1 = cosr1 * vtemp + sinr1 * v2;
         v2 = -sinr1 * vtemp + cosr1 * v2;
         vtemp = v4;
         v4 = cosr1 * vtemp + sinr1 * v5;
         v5 = -sinr1 * vtemp + cosr1 * v5;
         vtemp = v7;
         v7 = cosr1 * vtemp + sinr1 * v8;
         v8 = -sinr1 * vtemp + cosr1 * v8;

         if (Math.abs(e2) < CONVERGE_TOL || Math.abs(e1) < CONVERGE_TOL)
            converged = true;
      }

      if (Math.abs(e2) < CONVERGE_TOL)
      {
         compute_2X2(e1, s, sinl0, cosl0, sinr0, cosr0, 0);

         utemp = u0;
         u0 = cosl0 * utemp + sinl0 * u3;
         u3 = -sinl0 * utemp + cosl0 * u3;
         utemp = u0;
         u0 = cosl0 * utemp + sinl0 * u4;
         u4 = -sinl0 * utemp + cosl0 * u4;
         utemp = u2;
         u2 = cosl0 * utemp + sinl0 * u5;
         u5 = -sinl0 * utemp + cosl0 * u5;

         // update v matrices

         vtemp = v0;
         v0 = cosr0 * vtemp + sinr0 * v1;
         v1 = -sinr0 * vtemp + cosr0 * v1;
         vtemp = v3;
         v3 = cosr0 * vtemp + sinr0 * v4;
         v4 = -sinr0 * vtemp + cosr0 * v4;
         vtemp = v6;
         v6 = cosr0 * vtemp + sinr0 * v7;
         v7 = -sinr0 * vtemp + cosr0 * v7;
      }
      else
      {
         compute_2X2(e2, s, sinl0, cosl0, sinr0, cosr0, 1);

         utemp = u3;
         u3 = cosl0 * utemp + sinl0 * u6;
         u6 = -sinl0 * utemp + cosl0 * u6;
         utemp = u4;
         u4 = cosl0 * utemp + sinl0 * u7;
         u7 = -sinl0 * utemp + cosl0 * u7;
         utemp = u5;
         u5 = cosl0 * utemp + sinl0 * u8;
         u8 = -sinl0 * utemp + cosl0 * u8;

         // update v matrices

         vtemp = v1;
         v1 = cosr0 * vtemp + sinr0 * v2;
         v2 = -sinr0 * vtemp + cosr0 * v2;
         vtemp = v4;
         v4 = cosr0 * vtemp + sinr0 * v5;
         v5 = -sinr0 * vtemp + cosr0 * v5;
         vtemp = v7;
         v7 = cosr0 * vtemp + sinr0 * v8;
         v8 = -sinr0 * vtemp + cosr0 * v8;
      }

      return (0);
   }

   /**
    * Return max of the arguments.
    * @param a
    * @param b
    * @return
    */
   static final double max(double a, double b)
   {
      return (a > b ? a : b);
   }

   /**
    * Return the min of the two arguments.
    * @param a
    * @param b
    * @return
    */
   static final double min(double a, double b)
   {
      return (a < b ? a : b);
   }

   static final double d_sign(double a, double b)
   {
      double x = (a >= 0 ? a : -a);
      return (b >= 0 ? x : -x);
   }

   static final double compute_shift(double f, double g, double h)
   {
      double d__1, d__2;
      double fhmn, fhmx, c, fa, ga, ha, as, at, au;
      double ssmin;

      fa = Math.abs(f);
      ga = Math.abs(g);
      ha = Math.abs(h);
      fhmn = min(fa, ha);
      fhmx = max(fa, ha);
      if (fhmn == 0.)
      {
         ssmin = 0.;
         if (fhmx == 0.)
         {
         }
         else
         {
            d__1 = min(fhmx, ga) / max(fhmx, ga);
         }
      }
      else
      {
         if (ga < fhmx)
         {
            as = fhmn / fhmx + 1.;
            at = (fhmx - fhmn) / fhmx;
            d__1 = ga / fhmx;
            au = d__1 * d__1;
            c = 2. / (Math.sqrt(as * as + au) + Math.sqrt(at * at + au));
            ssmin = fhmn * c;
         }
         else
         {
            au = fhmx / ga;
            if (au == 0.)
            {

               ssmin = fhmn * fhmx / ga;
            }
            else
            {
               as = fhmn / fhmx + 1.;
               at = (fhmx - fhmn) / fhmx;
               d__1 = as * au;
               d__2 = at * au;
               c = 1. / (Math.sqrt(d__1 * d__1 + 1.) + Math.sqrt(d__2 * d__2 + 1.));
               ssmin = fhmn * c * au;
               ssmin += ssmin;
            }
         }
      }

      return (ssmin);
   }

   static int compute_2X2(double g, double[] s, Double snl, Double csl, Double snr, Double csr, int index)
   {
      double f = s[0];
      double h = s[1];
      double c_b3 = 2.0;
      double c_b4 = 1.0;

      double d__1;
      int pmax;
      double temp;
      boolean swap;
      double a, d, l, m, r, ss, t, tsign, fa, ga, ha;
      double ft, gt, ht, mm;
      boolean gasmal;
      double tt, clt, crt, slt, srt;
      double ssmin, ssmax;

      ssmax = f;
      ssmin = h;
      clt = 0.0;
      crt = 0.0;
      slt = 0.0;
      srt = 0.0;
      tsign = 0.0;

      ft = f;
      fa = Math.abs(ft);
      ht = h;
      ha = Math.abs(h);

      pmax = 1;
      if (ha > fa)
         swap = true;
      else
         swap = false;

      if (swap)
      {
         pmax = 3;
         temp = ft;
         ft = ht;
         ht = temp;
         temp = fa;
         fa = ha;
         ha = temp;

      }
      gt = g;
      ga = Math.abs(gt);
      if (ga == 0.)
      {

         s[1] = ha;
         s[0] = fa;
         clt = 1.0;
         crt = 1.0;
         slt = 0.0;
         srt = 0.0;
      }
      else
      {
         gasmal = true;

         if (ga > fa)
         {
            pmax = 2;
            if (fa / ga < 1.110223024E-16)
            {

               gasmal = false;
               ssmax = ga;
               if (ha > 1.)
               {
                  ssmin = fa / (ga / ha);
               }
               else
               {
                  ssmin = fa / ga * ha;
               }
               clt = 1.;
               slt = ht / gt;
               srt = 1.;
               crt = ft / gt;
            }
         }
         if (gasmal)
         {

            d = fa - ha;
            if (d == fa)
            {

               l = 1.;
            }
            else
            {
               l = d / fa;
            }

            m = gt / ft;

            t = 2.0 - l;

            mm = m * m;
            tt = t * t;
            ss = Math.sqrt(tt + mm);

            if (l == 0.0)
            {
               r = Math.abs(m);
            }
            else
            {
               r = Math.sqrt(l * l + mm);
            }

            a = (ss + r) * 0.5;

            if (ga > fa)
            {
               pmax = 2;
               if (fa / ga < 1.110223024E-16)
               {

                  gasmal = false;
                  ssmax = ga;
                  if (ha > 1.)
                  {
                     ssmin = fa / (ga / ha);
                  }
                  else
                  {
                     ssmin = fa / ga * ha;
                  }
                  clt = 1.;
                  slt = ht / gt;
                  srt = 1.;
                  crt = ft / gt;
               }
            }
            if (gasmal)
            {

               d = fa - ha;
               if (d == fa)
               {

                  l = 1.;
               }
               else
               {
                  l = d / fa;
               }

               m = gt / ft;

               t = 2. - l;

               mm = m * m;
               tt = t * t;
               ss = Math.sqrt(tt + mm);

               if (l == 0.)
               {
                  r = Math.abs(m);
               }
               else
               {
                  r = Math.sqrt(l * l + mm);
               }

               a = (ss + r) * .5;

               ssmin = ha / a;
               ssmax = fa * a;
               if (mm == 0.)
               {

                  if (l == 0.)
                  {
                     t = d_sign(c_b3, ft) * d_sign(c_b4, gt);
                  }
                  else
                  {
                     t = gt / d_sign(d, ft) + m / t;
                  }
               }
               else
               {
                  t = (m / (ss + t) + m / (r + l)) * (a + 1.);
               }
               l = Math.sqrt(t * t + 4.);
               crt = 2. / l;
               srt = t / l;
               clt = (crt + srt * m) / a;
               slt = ht / ft * srt / a;
            }
         }
         if (swap)
         {
            csl = srt;
            snl = crt;
            csr = slt;
            snr = clt;
         }
         else
         {
            csl = clt;
            snl = slt;
            csr = crt;
            snr = srt;
         }

         if (pmax == 1)
         {
            tsign = d_sign(c_b4, csr) * d_sign(c_b4, csl) * d_sign(c_b4, f);
         }
         if (pmax == 2)
         {
            tsign = d_sign(c_b4, snr) * d_sign(c_b4, csl) * d_sign(c_b4, g);
         }
         if (pmax == 3)
         {
            tsign = d_sign(c_b4, snr) * d_sign(c_b4, snl) * d_sign(c_b4, h);
         }
         s[index] = d_sign(ssmax, tsign);
         d__1 = tsign * d_sign(c_b4, f) * d_sign(c_b4, h);
         s[index + 1] = d_sign(ssmin, d__1);

      }
      return 0;
   }

   static double compute_rot(double f, double g, Double sin, Double cos)
   {

      double cs, sn;
      int i;
      double scale;
      int count;
      double f1, g1;
      double r;
      final double safmn2 = 2.002083095183101E-146;
      final double safmx2 = 4.994797680505588E+145;

      if (g == 0.)
      {
         cs = 1.;
         sn = 0.;
         r = f;
      }
      else if (f == 0.)
      {
         cs = 0.;
         sn = 1.;
         r = g;
      }
      else
      {
         f1 = f;
         g1 = g;
         scale = max(Math.abs(f1), Math.abs(g1));
         if (scale >= safmx2)
         {
            count = 0;
            while (scale >= safmx2)
            {
               ++count;
               f1 *= safmn2;
               g1 *= safmn2;
               scale = max(Math.abs(f1), Math.abs(g1));
            }
            r = Math.sqrt(f1 * f1 + g1 * g1);
            cs = f1 / r;
            sn = g1 / r;

            for (i = 1; i <= count; ++i)
            {
               r *= safmx2;
            }
         }
         else if (scale <= safmn2)
         {
            count = 0;
            while (scale <= safmn2)
            {
               ++count;
               f1 *= safmx2;
               g1 *= safmx2;
               scale = max(Math.abs(f1), Math.abs(g1));
            }
            r = Math.sqrt(f1 * f1 + g1 * g1);
            cs = f1 / r;
            sn = g1 / r;

            for (i = 1; i <= count; ++i)
            {
               r *= safmn2;
            }
         }
         else
         {
            r = Math.sqrt(f1 * f1 + g1 * g1);
            cs = f1 / r;
            sn = g1 / r;
         }
         if (Math.abs(f) > Math.abs(g) && cs < 0.)
         {
            cs = -cs;
            sn = -sn;
            r = -r;
         }
      }
      sin = sn;
      cos = cs;
      return r;

   }

   /**
    * Matrix multiplication of two double arrays. Result is stored in m3
    * 
    * @param m1
    * @param m2
    * @param m3
    */
   static private void mat_mul(double[] m1, double[] m2, double[] m3)
   {
      double result0 = m1[0] * m2[0] + m1[1] * m2[3] + m1[2] * m2[6];
      double result1 = m1[0] * m2[1] + m1[1] * m2[4] + m1[2] * m2[7];
      double result2 = m1[0] * m2[2] + m1[1] * m2[5] + m1[2] * m2[8];

      double result3 = m1[3] * m2[0] + m1[4] * m2[3] + m1[5] * m2[6];
      double result4 = m1[3] * m2[1] + m1[4] * m2[4] + m1[5] * m2[7];
      double result5 = m1[3] * m2[2] + m1[4] * m2[5] + m1[5] * m2[8];

      double result6 = m1[6] * m2[0] + m1[7] * m2[3] + m1[8] * m2[6];
      double result7 = m1[6] * m2[1] + m1[7] * m2[4] + m1[8] * m2[7];
      double result8 = m1[6] * m2[2] + m1[7] * m2[5] + m1[8] * m2[8];

      m3[0] = result0;
      m3[1] = result1;
      m3[2] = result2;
      m3[3] = result3;
      m3[4] = result4;
      m3[5] = result5;
      m3[6] = result6;
      m3[7] = result7;
      m3[8] = result8;

   }

   /**
    * Transpose a double array and pack it into the double array out.
    * 
    * @param in
    * @param out
    */
   static private void transpose_mat(double in0, double in1, double in2, double in3, double in4, double in5, double in6, double in7, double in8, double[] out)
   {
      out[0] = in0;
      out[1] = in3;
      out[2] = in6;

      out[3] = in1;
      out[4] = in4;
      out[5] = in7;

      out[6] = in2;
      out[7] = in5;
      out[8] = in8;
   }

   /**
    * Check if the argument a is within 1e-10 of 1.
    * 
    * @param a
    * @return
    */
   private static final boolean almostOne(double a)
   {
      return ((a < 1 + 1.0e-10) && (a > 1 - 1.0e-10));
   }

   /**
    * Check if the two argements are equal to eachother within a tolerance.
    * 
    * @param a
    * @param b
    * @return
    */
   private static final boolean almostEqual(double a, double b)
   {
      double diff = a - b;

      if (diff >= 0)
      {
         if (diff < 1.0e-10)
         {
            return true;
         }
         // a > b
         if ((b > 0) || (a > -b))
         {
            return (diff < 1.0e-4 * a);
         }
         else
         {
            return (diff < -1.0e-4 * b);
         }

      }
      else
      {
         if (diff > -1.0e-10)
         {
            return true;
         }
         if ((b < 0) || (-a > b))
         {
            return (diff > 1.0e-4 * a);
         }
         else
         {
            return (diff > -1.0e-4 * b);
         }
      }
   }

   /**
    * Find and return max of the three arguments.
    * @param val1
    * @param val2
    * @param val3
    * @return
    */
   static final double max3(double val1, double val2, double val3)
   {
      if (val1 > val2)
      {
         if (val1 > val3)
            return (val1);
         else
            return (val3);
      }
      else
      {
         if (val2 > val3)
            return (val2);
         else
            return (val3);
      }
   }

   private static final boolean almostZero(double a)
   {
      return ((a < 1.0e-5) && (a > -1.0e-5));
   }

   /**
    * Returns the matrix elements of this transform as a string.
    * 
    * @return the matrix elements of this transform
    */
   public String toString()
   {
      return mat00 + ", " + mat01 + ", " + mat02 + ", " + mat03 + "\n" + mat10 + ", " + mat11 + ", " + mat12 + ", " + mat13 + "\n" + mat20 + ", " + mat21
            + ", " + mat22 + ", " + mat23 + "\n" + "0" + ", " + "0" + ", " + "0" + ", " + "1" + "\n";
   }
}