package us.ihmc.sensorProcessing.imu;

import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tuple3D.Vector3D;
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

   public void setOrientationOffset(RotationMatrix orientationOffset)
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

   public RotationMatrix getOrientationOffset()
   {
      Matrix3D ret = new Matrix3D();
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

      return new RotationMatrix(ret);
   }

   public void setAccelerationOffset(Vector3D accelerationOffset)
   {
      setProperty(getAccelerationOffsetPropertyName(0), accelerationOffset.getX());
      setProperty(getAccelerationOffsetPropertyName(1), accelerationOffset.getY());
      setProperty(getAccelerationOffsetPropertyName(2), accelerationOffset.getZ());
   }

   public Vector3D getAccelerationOffset()
   {
      Vector3D ret = new Vector3D();
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

      public InvalidRotationMatrix(String originatingClass, RotationMatrix invalidRotationMatrix)
      {
         super("Not a valid rotation matrix in " + originatingClass + ":" + invalidRotationMatrix);
      }

      public InvalidRotationMatrix(String originatingClass, Matrix3D invalidRotationMatrix)
      {
         super("Not a valid rotation matrix in " + originatingClass + ":" + invalidRotationMatrix);
      }
   }

}
