package us.ihmc.sensorProcessing.imu;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;

import us.ihmc.tools.calibration.CalibrationProperties;

public class IMUCalibrationProperties extends CalibrationProperties
{
   private static final int MATRIX_SIZE = 3;

   public IMUCalibrationProperties(int index)
   {
      super("./calibration", "imu" + index + "Calibration.property");
   }

   private String getOrientationOffsetPropertyName(int row, int column)
   {
      return ("m" + row) + column;
   }

   private String getAccelerationOffsetPropertyName(int row)
   {
      return "accel" + row;
   }

   public void setOrientationOffset(Matrix3d orientationOffset)
   {
      for (int row = 0; row < MATRIX_SIZE; row++)
      {
         for (int column = 0; column < MATRIX_SIZE; column++)
         {
            String name = getOrientationOffsetPropertyName(row, column);
            setProperty(name, orientationOffset.getElement(row, column));
         }
      }
   }

   public Matrix3d getOrientationOffset()
   {
      Matrix3d ret = new Matrix3d();
      for (int row = 0; row < MATRIX_SIZE; row++)
      {
         for (int column = 0; column < MATRIX_SIZE; column++)
         {
            String name = getOrientationOffsetPropertyName(row, column);
            ret.setElement(row, column, getDoubleProperty(name));
         }
      }
      
      if (Math.abs(Math.abs(ret.determinant()) - 1.0) > 1e-6)
      {
         System.err.println("Not a valid rotation matrix in " + getClass().getName() + ":");
         System.err.println(ret);
         
         throw new InvalidRotationMatrix(getClass().getName(), ret);
      }

      return ret;
   }

   public void setAccelerationOffset(Vector3d accelerationOffset)
   {
      setProperty(getAccelerationOffsetPropertyName(0), accelerationOffset.getX());
      setProperty(getAccelerationOffsetPropertyName(1), accelerationOffset.getY());
      setProperty(getAccelerationOffsetPropertyName(2), accelerationOffset.getZ());
   }

   public Vector3d getAccelerationOffset()
   {
      Vector3d ret = new Vector3d();
      double[] accelerationOffsetArray = new double[MATRIX_SIZE];
      for (int row = 0; row < MATRIX_SIZE; row++)
      {
         String name = getAccelerationOffsetPropertyName(row);
         accelerationOffsetArray[row] = getDoubleProperty(name);
      }
      ret.set(accelerationOffsetArray);
      return ret;
   }
   
   public class InvalidRotationMatrix extends RuntimeException
   {
      private static final long serialVersionUID = -4680703803078778082L;
      public InvalidRotationMatrix(String originatingClass, Matrix3d invalidRotationMatrix)
      {
         super("Not a valid rotation matrix in " + originatingClass + ":" + invalidRotationMatrix);
      }
   }
   
}
